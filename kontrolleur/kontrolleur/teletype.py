"""Teletype: its scenes as text files (from a USB stick), what their scripts send through the i2c2midi,
and a live view of what actually arrives from it at the Pi.

The Teletype has no connection to a computer. When a USB stick goes in, its USB menu (firmware 4+) offers
WRITE TO USB (every saved scene to tt00s.txt … tt31s.txt), READ FROM USB (tt00.txt … tt31.txt become scenes
0–31) or both; older firmware does both by itself. The scene file: description lines, then "#1" … "#8",
"#M", "#I" with up to 6 command lines each, then "#P" (patterns) and "#G" (grid), kept here as they are.

A line is split the way the Teletype does it (src/teletype.c validate / process_command): an optional mod
before ": " (PROB 100: …), commands separated by "; ", each command read right to left with the arity of
its ops (tt_ops.py), so `I2M.CC# 32 8 RND 127` is I2M.CC#(32, 8, RND(127)).

i2c2midi channels: 1–16 = its TRS MIDI out, 17–32 = its USB port as ch 1–16. Only the USB port is
connected (XIAO-TT bridge → Pi → MIDI hub → SSP / NerdSEQ). I2M ops without # use the channel last set
with I2M.CH (default 1); the Teletype keeps it across scripts and scene loads until it is set again."""
from __future__ import annotations

import os
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from . import store
from .tt_ops import MODS, OPS

SCRIPTS = ["1", "2", "3", "4", "5", "6", "7", "8", "M", "I"]
SCRIPT_TITLE = {**{s: f"Script {s}" for s in "12345678"}, "M": "Metro", "I": "Init"}
MAX_LINES = 6
MAX_CHARS = 31          # the Teletype's line editor (LINE_EDITOR_SIZE 32 incl. the terminator)
MAX_WORDS = 16          # COMMAND_MAX_LENGTH
SCENES = 32

_NUM = re.compile(r"^(-?\d+|X[0-9A-F]+|[BR][01]+)$")


def number(tok: str) -> Optional[int]:
    """A Teletype number literal: 12, -3, XFF (hex), B101 (binary), R101 (binary, bits reversed)."""
    if not _NUM.match(tok):
        return None
    if tok[0] == "X":
        v = int(tok[1:], 16)
    elif tok[0] == "B":
        v = int(tok[1:], 2)
    elif tok[0] == "R":
        v = int(tok[1:][::-1], 2)
    else:
        return max(-32768, min(32767, int(tok)))
    return v - 65536 if v > 32767 else v


# ── parsing a line ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Node:
    tok: str
    args: List["Node"] = field(default_factory=list)
    value: Optional[int] = None          # a number literal
    is_set: bool = False                 # I2M.CH 19 (set) vs I2M.CH (get)
    start: int = 0                       # token span in the command, for the text
    end: int = 0

    def walk(self):
        yield self
        for a in self.args:
            yield from a.walk()


@dataclass
class Command:
    tokens: List[str]
    root: Optional[Node]
    error: str = ""
    ignored: Optional[Node] = None       # one value left over: the Teletype accepts the line and drops it

    def text(self, n: Optional[Node]) -> str:
        return " ".join(self.tokens[n.start:n.end]) if n is not None else ""


@dataclass
class Line:
    text: str
    mod: Optional[Command]               # PROB 100 / IF X / L 1 4 …
    cmds: List[Command]
    errors: List[str]


def _split(text: str) -> Tuple[Optional[str], List[str]]:
    if ":" in text:
        pre, post = text.split(":", 1)
        return pre.strip(), [c.strip() for c in post.split(";")]
    return None, [c.strip() for c in text.split(";")]


def _reduce(toks: List[str], first_is_cmd: bool) -> Tuple[List[Node], List[str]]:
    """Right to left with a value stack, like the Teletype: what's left on the stack, and errors."""
    errors = []
    stack: List[Node] = []
    for idx in range(len(toks) - 1, -1, -1):
        tok = toks[idx]
        v = number(tok)
        if v is not None:
            stack.append(Node(tok, value=v, start=idx, end=idx + 1))
            continue
        op = OPS.get(tok)
        if op is None and tok in MODS:
            errors.append(f"{tok} needs a ':' after it")
            op = (MODS[tok], 1, 0)
        if op is None:
            errors.append(f"unknown op {tok}")
            stack.append(Node(tok, start=idx, end=idx + 1))
            continue
        params, returns, settable = op
        first = idx == 0 and first_is_cmd
        n = params + (1 if first and settable and len(stack) > params else 0)
        if len(stack) < params:
            errors.append(f"{tok} needs {params} argument{'s' if params != 1 else ''}")
        args = [stack.pop() for _ in range(min(n, len(stack)))]
        node = Node(tok, args, is_set=n > params, start=idx, end=max([idx + 1] + [a.end for a in args]))
        if returns or first:
            stack.append(node)
        elif not first:
            errors.append(f"{tok} gives no value")
    return stack, errors


def _returns(n: Node) -> bool:
    return n.value is not None or bool(OPS.get(n.tok, (0, 1, 0))[1])


def parse_command(text: str, first_is_cmd: bool = True) -> Command:
    """Like the Teletype's validate(): a command may leave one value over (it is dropped), not two."""
    toks = text.upper().split()
    stack, errors = _reduce(toks, first_is_cmd)
    root = stack[-1] if stack else None
    left = stack[:-1] + ([root] if root is not None and _returns(root) else [])
    ignored = None
    if len(left) > 1:
        errors.append("too many values")
    elif len(left) == 1 and root is not None and not _returns(root):
        ignored = left[0]
    return Command(toks, root, "; ".join(errors), ignored)


def parse_line(text: str) -> Line:
    errors = []
    t = text.strip()
    if len(t) > MAX_CHARS:
        errors.append(f"longer than {MAX_CHARS} characters (the Teletype cuts it)")
    pre, subs = _split(t)
    mod = None
    if pre is not None:
        mt = pre.upper().split()
        if not mt or mt[0] not in MODS:
            errors.append(f"{mt[0] if mt else ':'} isn't a mod (PROB, IF, L, EVERY …)")
        else:
            stack, errs = _reduce(mt[1:], False)
            want = MODS[mt[0]]
            if errs:
                errors.append(f"{mt[0]}: " + "; ".join(errs))
            elif len(stack) != want:
                errors.append(f"{mt[0]} takes {want} argument{'s' if want != 1 else ''}")
            mod = Command(mt, None, "; ".join(errs))
    cmds = []
    for c in subs:
        if not c:
            continue
        cmd = parse_command(c)
        if len(cmd.tokens) > MAX_WORDS:
            errors.append(f"more than {MAX_WORDS} words")
        if cmd.error:
            errors.append(cmd.error)
        cmds.append(cmd)
    return Line(t, mod, cmds, errors)


# ── what a scene does with the i2c2midi and the ER-301 ─────────────────────────────────────────

# I2M ops that send: op → (kind, names of the arguments after the channel)
I2M_SEND = {
    "I2M.N": ("note", "note vel"), "I2M.NOTE": ("note", "note vel"), "I2M.NT": ("note", "note vel dur"),
    "I2M.NO": ("note off", "note"), "I2M.NOTE.O": ("note off", "note"),
    "I2M.CC": ("cc", "cc val"), "I2M.CCV": ("cc", "cc volts"), "I2M.CC.SET": ("cc", "cc val"),
    "I2M.NRPN": ("nrpn", "msb lsb val"), "I2M.NRPN.SET": ("nrpn", "msb lsb val"),
    "I2M.C": ("chord", "chord root vel"), "I2M.CHORD": ("chord", "chord root vel"),
    "I2M.PRG": ("program", "prg"), "I2M.PB": ("pitch bend", "val"), "I2M.AT": ("aftertouch", "val"),
}
for _k in ("I2M.N", "I2M.NT", "I2M.NO", "I2M.CC", "I2M.CCV", "I2M.CC.SET", "I2M.NRPN", "I2M.NRPN.SET", "I2M.C"):
    I2M_SEND[_k + "#"] = I2M_SEND[_k]                 # the same with the channel as first argument
I2M_REALTIME = {"I2M.CLK": "clock", "I2M.START": "start", "I2M.STOP": "stop", "I2M.CONT": "continue"}
I2M_QUERY = {"I2M.Q.NOTE": "note", "I2M.Q.N": "note", "I2M.Q.VEL": "note", "I2M.Q.V": "note", "I2M.Q.CC": "cc",
             "I2M.Q.LCH": "any", "I2M.Q.LN": "note", "I2M.Q.LV": "note", "I2M.Q.LO": "note", "I2M.Q.LC": "cc",
             "I2M.Q.LCC": "cc"}
SC_OPS = {"SC.CV": "CV", "SC.CV.SLEW": "CV", "SC.CV.SET": "CV", "SC.CV.OFF": "CV",
          "SC.TR": "TR", "SC.TR.TOG": "TR", "SC.TR.PULSE": "TR", "SC.TR.P": "TR", "SC.TR.TIME": "TR", "SC.TR.POL": "TR"}


@dataclass
class Where:
    script: str
    line: int                 # 0-based
    text: str

    @property
    def title(self) -> str:
        return f"{SCRIPT_TITLE[self.script]} line {self.line + 1}"


@dataclass
class Send:
    where: Where
    op: str
    kind: str                 # note, note off, cc, nrpn, chord, program, pitch bend, aftertouch, clock …
    ch: Optional[int]         # i2c2midi channel 1–32, None = can't tell
    ch_text: str              # how the channel is given ("32", "from I2M.CH in Init", "I2M.CH X" …)
    num: Optional[int]        # CC / note number, None = changes or n/a
    num_text: str
    value_text: str
    guarded: bool             # behind a mod (PROB / IF / EVERY …): not every time

    @property
    def usb(self) -> Optional[int]:
        """MIDI channel on the i2c2midi's USB port (1–16), None for TRS or unknown."""
        return self.ch - 16 if self.ch is not None and 17 <= self.ch <= 32 else None

    @property
    def trs(self) -> bool:
        return self.ch is not None and 1 <= self.ch <= 16

    @property
    def bad_channel(self) -> bool:
        return self.ch is not None and not 1 <= self.ch <= 32

    @property
    def hub_kind(self) -> Optional[str]:
        """'cc' / 'note' as the hub and the SSP see it."""
        return {"cc": "cc", "note": "note", "note off": "note", "chord": "note"}.get(self.kind)

    def label(self) -> str:
        k = {"cc": "CC", "nrpn": "NRPN", "note off": "note off"}.get(self.kind, self.kind)
        n = f" {self.num}" if self.num is not None else (f" {self.num_text}" if self.num_text else "")
        return f"{k}{n}"


@dataclass
class PortUse:
    where: Where
    op: str
    kind: str                 # CV / TR
    port: Optional[int]
    port_text: str


@dataclass
class Listen:
    where: Where
    op: str
    kind: str                 # note / cc / any
    num: Optional[int]
    num_text: str


@dataclass
class ChannelSet:
    where: Where
    ch: Optional[int]
    text: str
    query: bool = False       # I2M.Q.CH (what I2M.Q.* listen to)


@dataclass
class Scene:
    number: int
    text: List[str]                                   # description; the first line is the name
    scripts: Dict[str, List[str]]
    rest: str = ""                                    # #P and #G, written back as they were
    source: str = ""                                  # the file it came from

    @property
    def name(self) -> str:
        return next((t for t in self.text if t.strip()), "").strip()

    @property
    def empty(self) -> bool:
        return not self.name and not any(self.scripts.get(s) for s in SCRIPTS)

    def lines(self, script: str) -> List[Line]:
        return [parse_line(t) for t in self.scripts.get(script, [])]

    def serialize(self) -> str:
        """As the Teletype writes it (scene_serialization.c), patterns and grid untouched."""
        out = []
        blank = False
        for t in self.text:
            if t:
                out.append(t + "\n")
                blank = False
            elif not blank:
                out.append("\n")
                blank = True
        for s in SCRIPTS:
            out.append(f"\n\n#{s}")
            for t in self.scripts.get(s, [])[:MAX_LINES]:
                out.append("\n" + t.strip().upper())
        out.append("\n\n" + (self.rest.lstrip("\n") if self.rest.strip() else "#P\n"))
        return "".join(out)


def parse_scene(text: str, number: int = 0, source: str = "") -> Scene:
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    desc: List[str] = []
    scripts: Dict[str, List[str]] = {s: [] for s in SCRIPTS}
    rest_at = None
    cur = None
    lines = text.split("\n")
    pos = 0
    for raw in lines:
        if raw.startswith("#"):
            key = raw[1:2].upper()
            if key in ("P", "G"):
                rest_at = pos
                break
            cur = key if key in scripts else "_ignore"
        elif cur is None:
            desc.append(raw)
        elif cur != "_ignore" and raw.strip() and len(scripts[cur]) < MAX_LINES:
            scripts[cur].append(raw.strip().upper())
        pos += len(raw) + 1
    for _ in range(2):                     # the writer puts "\n\n" before "#1"
        if desc and desc[-1] == "":
            desc.pop()
    rest = text[rest_at:] if rest_at is not None else ""
    return Scene(number, desc, scripts, rest, source)


@dataclass
class Analysis:
    sends: List[Send]
    ports: List[PortUse]
    listens: List[Listen]
    channel_sets: List[ChannelSet]
    errors: List[Tuple[Where, str]]

    def by_line(self, script: str, line: int):
        sends = [s for s in self.sends if s.where.script == script and s.where.line == line]
        ports = [u for u in self.ports if u.where.script == script and u.where.line == line]
        listens = [q for q in self.listens if q.where.script == script and q.where.line == line]
        chs = [c for c in self.channel_sets if c.where.script == script and c.where.line == line]
        return sends, ports, listens, chs


def _lit(n: Optional[Node]) -> Optional[int]:
    return n.value if n is not None else None


VARIABLES = ("A", "B", "C", "D", "X", "Y", "Z", "T")      # the scene-wide ones (J, K, I are per script / loop)


def variable_values(scene: "Scene") -> Dict[str, Optional[List[Tuple[int, "Where"]]]]:
    """For each variable: the numbers the scene sets it to, or None when it is also set from an expression
    (or changed by an op like X ADD X 1, DRUNK …), so its value can't be told from the text."""
    out: Dict[str, Optional[List[Tuple[int, Where]]]] = {}
    for s in SCRIPTS:
        for li, ln in enumerate(scene.lines(s)):
            for cmd in ln.cmds:
                r = cmd.root
                if r is None or r.tok not in VARIABLES or not r.is_set:
                    continue
                v = r.args[0].value if r.args else None
                if v is None or out.get(r.tok, []) is None:
                    out[r.tok] = None
                else:
                    out.setdefault(r.tok, []).append((v, Where(s, li, ln.text)))
    return out


def analyse(scene: Scene) -> Analysis:
    sends, ports, listens, chsets, errors = [], [], [], [], []
    var = variable_values(scene)

    def val(n: Optional[Node]) -> Tuple[List[Optional[int]], str]:
        """The value(s) an argument can have: a number, or a variable the scene only sets to numbers."""
        if n is None:
            return [None], ""
        if n.value is not None:
            return [n.value], ""
        known = var.get(n.tok) if n.tok in VARIABLES and not n.args else None
        if known:
            vals = sorted({v for v, _ in known})
            where = ", ".join(sorted({w.title.replace(" line ", " ").replace("Script ", "") for _, w in known}))
            return vals, f"{n.tok} = {' or '.join(map(str, vals))} (set in {where})"
        return [None], ""

    pending = []                      # sends without # wait for the I2M.CH values of the whole scene
    for s in SCRIPTS:
        for li, ln in enumerate(scene.lines(s)):
            w = Where(s, li, ln.text)
            for e in ln.errors:
                errors.append((w, e))
            guarded = ln.mod is not None
            for cmd in ln.cmds:
                if cmd.root is None:
                    continue
                for node in cmd.root.walk():
                    t = node.tok
                    if t in ("I2M.CH", "I2M.#", "I2M.Q.CH", "I2M.Q.#") and node.is_set and node is cmd.root:
                        a = node.args[0]
                        vs, how = val(a)
                        for v in vs:
                            chsets.append(ChannelSet(w, v, how or cmd.text(a), query=".Q." in t))
                    elif t in I2M_QUERY and node.args:
                        a = node.args[0]
                        listens.append(Listen(w, t, I2M_QUERY[t], _lit(a), cmd.text(a)))
                    elif t in I2M_QUERY:
                        listens.append(Listen(w, t, I2M_QUERY[t], None, ""))
                    elif t in SC_OPS and node.args:
                        a = node.args[0]
                        for v in val(a)[0]:
                            ports.append(PortUse(w, t, SC_OPS[t], v, cmd.text(a)))
                    elif t in I2M_REALTIME:
                        pending.append((w, t, I2M_REALTIME[t], None, None, "", guarded))
                    elif t in I2M_SEND and node is cmd.root:
                        kind, names = I2M_SEND[t]
                        args = list(node.args)
                        chs = [(None, None)]
                        if t.endswith("#"):
                            if not args:
                                continue
                            cha = args.pop(0)
                            vs, how = val(cha)
                            chs = [(v, how or cmd.text(cha)) for v in vs]
                        names = names.split()
                        num_node = None
                        if kind in ("note", "note off", "cc", "nrpn") and args:
                            num_node = args[0]
                        elif kind == "chord" and len(args) > 1:
                            num_node = None            # a chord plays several notes
                        vtext = " ".join(cmd.text(a) for a in args[1:]) if kind not in ("program", "pitch bend", "aftertouch") \
                            else " ".join(cmd.text(a) for a in args)
                        nvs, nhow = val(num_node)
                        num_text = (nhow or cmd.text(num_node)) if num_node is not None else ""
                        if kind == "nrpn" and len(args) >= 2 and args[0].value is not None and args[1].value is not None:
                            nvs = [args[0].value * 128 + args[1].value]
                            num_text = f"{args[0].value}/{args[1].value}"
                            vtext = " ".join(cmd.text(a) for a in args[2:])
                        for ch, ch_text in chs:
                            for num in nvs:
                                pending.append((w, t, kind, ch, ch_text, (num, num_text, vtext), guarded))
    # channels for the ops without #: every value set with I2M.CH in the scene, or 1 (the i2c2midi's default)
    set_chs = [c for c in chsets if not c.query]
    for w, t, kind, ch, ch_text, nv, guarded in pending:
        num, num_text, vtext = nv if nv else (None, "", "")
        if ch_text is not None:
            sends.append(Send(w, t, kind, ch, ch_text, num, num_text, vtext, guarded))
            continue
        if not set_chs:
            sends.append(Send(w, t, kind, 1, "1 (I2M.CH not set in this scene: the last value set, 1 after power-up)",
                              num, num_text, vtext, guarded))
            continue
        seen = set()
        for c in set_chs:
            key = c.ch if c.ch is not None else c.text
            if key in seen:
                continue
            seen.add(key)
            src = f"I2M.CH {c.text} in {c.where.title}"
            sends.append(Send(w, t, kind, c.ch, src, num, num_text, vtext, guarded))
    return Analysis(sends, ports, listens, chsets, errors)


def ignored_note(cmd: Command) -> Optional[str]:
    """What the Teletype makes of a command with one value too many (it runs it and drops the last value)."""
    r, extra = cmd.root, cmd.ignored
    if r is None or extra is None:
        return None
    names = []
    if r.tok in I2M_SEND:
        names = (["ch"] if r.tok.endswith("#") else []) + I2M_SEND[r.tok][1].split()
    parts = [f"{names[k] if k < len(names) else 'arg ' + str(k + 1)} = {cmd.text(a)}" for k, a in enumerate(r.args)]
    msg = f"{r.tok} takes {len(r.args)} value{'s' if len(r.args) != 1 else ''}: " + ", ".join(parts) + \
          f"; the {cmd.text(extra)} at the end is dropped."
    for k, a in enumerate(r.args):           # RND 80 120 meant as a range?
        if a.tok in ("RND", "RAND") and a.args and k + 1 < len(r.args):
            nxt = r.args[k + 1]
            msg += f" For a random value between {cmd.text(a.args[0])} and {cmd.text(nxt)} write RRND {cmd.text(a.args[0])} " \
                   f"{cmd.text(nxt)}."
            break
    return msg


def warnings(scene: Scene, an: Analysis) -> List[Tuple[Optional[Where], str]]:
    """Things that look like mistakes, worded for the panel."""
    out: List[Tuple[Optional[Where], str]] = []
    for w, e in an.errors:
        out.append((w, e))
    for s in SCRIPTS:
        for li, ln in enumerate(scene.lines(s)):
            for cmd in ln.cmds:
                note = ignored_note(cmd)
                if note:
                    out.append((Where(s, li, ln.text), note))
    for s in an.sends:
        if s.trs:
            out.append((s.where, f"{s.op} on ch {s.ch} goes to the i2c2midi's TRS jack, not over USB: nothing reaches "
                                 f"the Pi. Ch {s.ch + 16} is USB ch {s.ch}."))
        elif s.bad_channel:
            out.append((s.where, f"{s.op}: channel {s.ch} doesn't exist (1–16 TRS, 17–32 USB)."))
    for c in an.channel_sets:
        if c.where.script == "I" and not c.query:
            out.append((c.where, "I2M.CH in Init runs only when the scene is loaded (or Init is run by hand)."))
    no_hash = [s for s in an.sends if s.ch_text.startswith("1 (I2M.CH")]
    if no_hash:
        out.append((no_hash[0].where, "This scene sends without a channel and never sets I2M.CH: it uses whatever was "
                                      "set last (1 after power-up = TRS)."))
    return out


# ── the scenes kept by the app ─────────────────────────────────────────────────────────────────

_FILE = re.compile(r"^tt(\d\d)s?\.txt$", re.I)


def cache_dir() -> Path:
    d = store.data_dir() / "teletype"
    d.mkdir(parents=True, exist_ok=True)
    return d


def stick_candidates() -> List[Path]:
    """Mounted drives that look like a stick written by the Teletype (tt00s.txt …), newest first."""
    roots = []
    if sys.platform == "darwin":
        roots = [Path("/Volumes")]
    else:
        user = os.environ.get("USER", "")
        roots = [Path("/run/media") / user, Path("/media") / user, Path("/media"), Path("/mnt")]
    found = []
    for r in roots:
        try:
            for v in r.iterdir():
                try:
                    if v.is_dir() and any(_FILE.match(f.name) and f.name.lower().endswith("s.txt") for f in v.iterdir()):
                        found.append(v)
                except OSError:
                    pass
        except OSError:
            pass
    return sorted(found, key=lambda p: max((f.stat().st_mtime for f in p.glob("tt*s.txt")), default=0), reverse=True)


class TeletypeState:
    def __init__(self):
        self.scenes: Dict[int, Scene] = {}
        self.imported_from = ""
        self.imported_at = 0.0
        self._an: Dict[int, Tuple[str, Analysis]] = {}
        self.load_cache()

    @property
    def loaded(self) -> bool:
        return bool(self.scenes)

    def load_cache(self):
        self.scenes = {}
        d = cache_dir()
        for f in sorted(d.glob("tt*s.txt")):
            m = _FILE.match(f.name)
            if m:
                try:
                    self.scenes[int(m.group(1))] = parse_scene(f.read_text(errors="replace"), int(m.group(1)), f.name)
                except OSError:
                    pass
        meta = d / "source.txt"
        if meta.exists():
            try:
                src, at = meta.read_text().split("\n", 1)
                self.imported_from, self.imported_at = src, float(at or 0)
            except (OSError, ValueError):
                pass

    def import_folder(self, folder: Path) -> int:
        """Copy tt##s.txt (what the Teletype wrote) from a stick or folder into the app's store."""
        files = [f for f in folder.iterdir() if _FILE.match(f.name) and f.name.lower().endswith("s.txt")]
        if not files:
            raise ValueError(f"No Teletype scenes (tt00s.txt …) in {folder}. On the Teletype: put the stick in "
                             "and choose WRITE TO USB first.")
        d = cache_dir()
        for old in d.glob("tt*s.txt"):
            old.unlink()
        for f in files:
            (d / f.name.lower()).write_bytes(f.read_bytes())
        (d / "source.txt").write_text(f"{folder}\n{time.time()}")
        self._an.clear()
        self.load_cache()
        return len(files)

    def scene(self, n: int) -> Optional[Scene]:
        return self.scenes.get(n)

    def analysis(self, n: int) -> Optional[Analysis]:
        sc = self.scenes.get(n)
        if sc is None:
            return None
        key = sc.serialize()
        hit = self._an.get(n)
        if hit is None or hit[0] != key:
            hit = (key, analyse(sc))
            self._an[n] = hit
        return hit[1]

    def set_script(self, n: int, script: str, lines: List[str]):
        sc = self.scenes.get(n)
        if sc is None:
            sc = self.scenes[n] = Scene(n, [], {s: [] for s in SCRIPTS})
        sc.scripts[script] = [t.strip().upper() for t in lines if t.strip()][:MAX_LINES]
        self.save(n)

    def set_text(self, n: int, lines: List[str]):
        sc = self.scenes.get(n)
        if sc is None:
            return
        sc.text = [t[:32] for t in lines][:32]
        self.save(n)

    def save(self, n: int):
        sc = self.scenes[n]
        (cache_dir() / f"tt{n:02d}s.txt").write_text(sc.serialize())

    def write_for_teletype(self, n: int, folder: Path) -> Path:
        """tt##.txt: what READ FROM USB loads into scene ## (and overwrites it there)."""
        path = folder / f"tt{n:02d}.txt"
        path.write_text(self.scenes[n].serialize())
        return path


# ── what arrives from the i2c2midi, live (scripts/midi_monitor.py on the Pi) ───────────────────

@dataclass
class Seen:
    kind: str                 # cc, note, program, pitch bend, aftertouch, poly AT, start …
    ch: int                   # MIDI channel 1–16 as the hub sees it (USB)
    num: Optional[int]
    value: int = 0
    count: int = 0
    first: float = 0.0
    last: float = 0.0


def decode(m: List[int]) -> Optional[Tuple[str, int, Optional[int], int]]:
    """(kind, channel 1–16, number, value) of a MIDI message; note offs come back as ('note off', …)."""
    if not m:
        return None
    st = m[0]
    if st >= 0xF0:
        name = {0xFA: "start", 0xFB: "continue", 0xFC: "stop", 0xF8: "clock"}.get(st)
        return (name, 0, None, 0) if name else None
    kind, ch = st & 0xF0, (st & 0x0F) + 1
    d1 = m[1] if len(m) > 1 else 0
    d2 = m[2] if len(m) > 2 else 0
    if kind == 0x90 and d2 > 0:
        return "note", ch, d1, d2
    if kind in (0x80, 0x90):
        return "note off", ch, d1, 0
    if kind == 0xB0:
        return "cc", ch, d1, d2
    if kind == 0xC0:
        return "program", ch, None, d1
    if kind == 0xE0:
        return "pitch bend", ch, None, (d2 << 7) | d1
    if kind == 0xD0:
        return "aftertouch", ch, None, d1
    if kind == 0xA0:
        return "poly AT", ch, d1, d2
    return None


class Monitor:
    """Runs midi_monitor.py on the Pi over ssh and keeps what came in. Qt-driven (QProcess)."""

    def __init__(self, on_change):
        self.on_change = on_change
        self.proc = None
        self.running = False
        self.port: Optional[str] = None       # the XIAO-TT port name on the Pi, None = not plugged in
        self.alive_at = 0.0                   # last heartbeat (monotonic)
        self.error = ""
        self.seen: Dict[Tuple[str, int, Optional[int]], Seen] = {}
        self.log = deque(maxlen=200)          # (time, kind, ch, num, value)
        self.started_at = 0.0
        self._buf = b""
        self._dirty = False
        self._timer = None

    def clear(self):
        self.seen.clear()
        self.log.clear()
        self._notify(now=True)

    def start(self, host: str, identity: str):
        from PySide6.QtCore import QProcess, QTimer
        from . import clients
        self.stop()
        self.error = ""
        self.port = None
        self.alive_at = 0.0
        try:
            src = (clients.SCRIPTS / "midi_monitor.py").read_text()
        except OSError:
            self.error = f"midi_monitor.py is missing in {clients.SCRIPTS} (update the app: git pull)."
            self._notify(now=True)
            return
        args = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=6", "-o", "StrictHostKeyChecking=accept-new",
                "-o", "ServerAliveInterval=5"]
        if identity.strip():
            args += ["-o", "IdentitiesOnly=yes", "-i", os.path.expanduser(identity.strip())]
        args += [host, "python3 -u - --port XIAO-TT"]
        p = QProcess()
        p.setProgram("ssh")
        p.setArguments(args)
        p.readyReadStandardOutput.connect(self._read)
        p.finished.connect(self._finished)
        p.errorOccurred.connect(self._failed)
        self.proc = p
        self.running = True
        self.started_at = time.monotonic()
        p.start()
        p.write(src.encode())
        p.closeWriteChannel()
        if self._timer is None:
            self._timer = QTimer()
            self._timer.setInterval(120)          # repaint at most ~8× a second, however busy the Teletype is
            self._timer.timeout.connect(self._tick)
        self._timer.start()
        self._notify(now=True)

    def stop(self):
        p, self.proc = self.proc, None
        self.running = False
        if p is not None:
            try:
                p.finished.disconnect()
                p.errorOccurred.disconnect()
            except (RuntimeError, TypeError):
                pass
            p.kill()
            p.waitForFinished(1000)
        if self._timer is not None:
            self._timer.stop()
        self._notify(now=True)

    @property
    def connected(self) -> bool:
        return self.running and self.alive_at > 0 and time.monotonic() - self.alive_at < 4

    def _read(self):
        import json
        if self.proc is None:
            return
        self._buf += bytes(self.proc.readAllStandardOutput())
        *lines, self._buf = self._buf.split(b"\n")
        now = time.monotonic()
        for raw in lines:
            try:
                d = json.loads(raw)
            except ValueError:
                continue
            if d.get("hb"):
                self.alive_at = now
                if d.get("port") != self.port:
                    self.port = d.get("port")
                    self._dirty = True
                continue
            msg = decode(d.get("m") or [])
            if msg is None:
                continue
            kind, ch, num, val = msg
            self.log.append((now, kind, ch, num, val))
            if kind in ("note off", "clock"):
                continue
            key = (kind, ch, num)
            s = self.seen.get(key)
            if s is None:
                s = self.seen[key] = Seen(kind, ch, num, first=now)
            s.value, s.count, s.last = val, s.count + 1, now
            self._dirty = True
        self.alive_at = max(self.alive_at, now if lines else self.alive_at)

    def _tick(self):
        if self.running and self.alive_at and time.monotonic() - self.alive_at > 4 and not self.error:
            self.error = "No answer from the Pi for a few seconds."
            self._dirty = True
        if self._dirty:
            self._notify(now=True)

    def _notify(self, now=False):
        self._dirty = False
        try:
            self.on_change()
        except RuntimeError:
            pass

    def _finished(self, code, _status=None):
        err = ""
        if self.proc is not None:
            err = bytes(self.proc.readAllStandardError()).decode(errors="replace").strip()
        self.running = False
        self.proc = None
        if self._timer is not None:
            self._timer.stop()
        self.error = (err.splitlines()[-1] if err else f"ssh ended (code {code}).")
        self._notify(now=True)

    def _failed(self, _e):
        if self.proc is not None and self.proc.state() == 0:
            self.running = False
            self.error = "Couldn't start ssh."
            self._notify(now=True)

    def matches(self, s: Send) -> Optional[Seen]:
        """What arrived for one send of the scene (same USB channel, kind and number)."""
        if s.usb is None or s.hub_kind is None:
            return None
        kind = "note" if s.hub_kind == "note" else s.kind
        best = None
        for (k, ch, num), v in self.seen.items():
            if k == kind and ch == s.usb and (s.num is None or num == s.num):
                if best is None or v.last > best.last:
                    best = v
        return best
