"""
DualSense routing and button functions: what the app edits, and the two files it writes on the
SSH host (src/config/er301_routes.py and src/config/controls.py). Port of the Mac app's Model.swift;
the generated files are byte-identical to the Mac app's.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ── ER-301 routing roles (what the i2c2midi does with a control's MIDI) ─────────────────────────

OFF, CV, TRIGGER, NOTE_TRIGGER, LFO_RATE, LFO_RESET, OCTAVE = (
    "off", "cv", "trigger", "noteTrigger", "lfoRate", "lfoReset", "octave")
ROLES = [OFF, CV, TRIGGER, NOTE_TRIGGER, LFO_RATE, LFO_RESET, OCTAVE]
ROLE_TITLE = {OFF: "Off", CV: "CV", TRIGGER: "Trigger", NOTE_TRIGGER: "Trigger",
              LFO_RATE: "LFO rate", LFO_RESET: "LFO reset", OCTAVE: "Octave"}


def role_uses_port(r: str) -> bool:
    return r in (CV, TRIGGER, NOTE_TRIGGER)


def role_exclusive(r: str) -> bool:
    return r in (LFO_RATE, LFO_RESET, OCTAVE)


# ── Button functions (config/controls.py) ───────────────────────────────────────────────────────

ACTIONS = ["none", "midi", "loop_record", "loop_play", "loop_clear", "motion", "block_up", "block_down",
           "block_reset", "section_up", "section_down", "strum", "freeze_left", "freeze_right", "quantize"]
ACTION_TITLE = {
    "none": "Nothing", "midi": "MIDI → ER-301", "loop_record": "Loop: record start / stop",
    "loop_play": "Loop: play / stop", "loop_clear": "Loop: delete", "motion": "Motion control on / off",
    "block_up": "Slice block +128", "block_down": "Slice block −128", "block_reset": "Slice block → 0",
    "section_up": "Slice section +16", "section_down": "Slice section −16", "strum": "Harmonic strum",
    "freeze_left": "Freeze L2 / R2 / left stick", "freeze_right": "Freeze right stick",
    "quantize": "Loop quantize on / off",
}
ACTION_TOKEN = {
    "none": "—", "midi": "MIDI", "loop_record": "REC", "loop_play": "PLAY", "loop_clear": "DELETE",
    "motion": "MOTION", "block_up": "BLOCK +", "block_down": "BLOCK −", "block_reset": "BLOCK 0",
    "section_up": "SECTION +", "section_down": "SECTION −", "strum": "STRUM", "freeze_left": "FREEZE L",
    "freeze_right": "FREEZE R", "quantize": "QUANTIZE",
}
HOLD_CHOICES = [a for a in ACTIONS if a != "midi"]
CHORD_CHOICES = [a for a in ACTIONS if a not in ("midi", "none")]


def fires_on_release(a: str) -> bool:
    return a in ("loop_record", "loop_play", "loop_clear")


@dataclass
class ButtonFunc:
    tap: str
    hold: str = "none"


@dataclass
class Chord:
    a: str
    b: str
    action: str
    id: str = field(default_factory=lambda: uuid.uuid4().hex)


BUTTONS: List[Tuple[str, str]] = [
    ("cross", "✕"), ("circle", "○"), ("triangle", "△"), ("square", "□"),
    ("l1", "L1"), ("r1", "R1"), ("l3", "L3"), ("r3", "R3"),
    ("create", "Create"), ("options", "Options"), ("ps", "PS"), ("mic", "Mic"), ("touchpad", "Touchpad click"),
]


def button_label(n: str) -> str:
    for name, label in BUTTONS:
        if name == n:
            return label
    return n


def default_funcs() -> Dict[str, ButtonFunc]:
    return {
        "cross": ButtonFunc("midi"), "circle": ButtonFunc("midi"), "triangle": ButtonFunc("midi"),
        "square": ButtonFunc("midi"), "l1": ButtonFunc("loop_clear"), "r1": ButtonFunc("loop_play", "loop_record"),
        "l3": ButtonFunc("freeze_left"), "r3": ButtonFunc("freeze_right"), "create": ButtonFunc("block_down"),
        "options": ButtonFunc("block_up"), "ps": ButtonFunc("strum"), "mic": ButtonFunc("none"),
        "touchpad": ButtonFunc("quantize"),
    }


def default_chords() -> List[Chord]:
    return [Chord("l1", "r1", "motion"), Chord("create", "options", "block_reset")]


# ── The controller ──────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class InputDef:
    id: str              # CC_MAP / NRPN_MAP name (nrpn_ prefix for NRPNs)
    label: str
    kind: str            # cc nrpn note
    name: Optional[str]  # key in CC_MAP / NRPN_MAP on the SSH host
    fallback: int        # number if the name is unknown there
    roles: Tuple[str, ...]
    button: Optional[str] = None   # controls.py button name for digital buttons

    @property
    def short(self) -> str:
        if self.id in ("left_stick_x", "right_stick_x", "touchpad_x"):
            return "X"
        if self.id in ("left_stick_y", "right_stick_y", "touchpad_y"):
            return "Y"
        if self.id in ("btn_touchpad", "btn_l3", "btn_r3"):
            return "CLICK"
        if self.id == "dpad_vertical":
            return "↕"
        if self.id == "dpad_horizontal":
            return "↔"
        return self.label.upper()


@dataclass(frozen=True)
class GroupDef:
    id: str
    title: str
    hint: str
    inputs: Tuple[InputDef, ...]


AXIS = (OFF, CV, LFO_RATE)
BUTTON = (OFF, TRIGGER, CV, LFO_RESET)


def _cc(name, n, label, roles):
    return InputDef(name, label, "cc", name, n, tuple(roles))


def _btn(name, n, button, label="Button"):
    return InputDef(name, label, "cc", name, n, BUTTON, button)


def _nrpn(name, n, label):
    return InputDef("nrpn_" + name, label, "nrpn", name, n, (OFF, CV))


BUTTON_HINT = "Tap and hold can do different things. Set the tap to “MIDI → ER-301” to route the button like a trigger."

GROUPS: List[GroupDef] = [
    GroupDef("l2", "L2", "Analog trigger: 0–127 → 0–10 V.", (_cc("l2_trigger", 7, "Pressure", AXIS),)),
    GroupDef("l1", "L1", BUTTON_HINT, (_btn("btn_l1", 24, "l1"),)),
    GroupDef("r2", "R2", "Analog trigger. As LFO rate: released = LFO stopped, fully pressed = fastest.",
             (_cc("r2_trigger", 10, "Pressure", AXIS),)),
    GroupDef("r1", "R1", BUTTON_HINT, (_btn("btn_r1", 25, "r1"),)),
    GroupDef("dpad", "D-pad", "↕ sends 8 steps (as Octave it shifts the pitch CV). ↔ picks the 16-slice section, or sends CC steps.",
             (_cc("dpad_vertical", 11, "Up / down", (OFF, CV, OCTAVE)),
              _cc("dpad_horizontal", 13, "Left / right", (OFF, CV)))),
    GroupDef("leftStick", "Left stick", "X picks the slice inside the 16-slice section, or sends a CC. Springs back to the centre (64 ≈ 5 V).",
             (_cc("left_stick_x", 1, "X axis", AXIS), _cc("left_stick_y", 2, "Y axis", AXIS),
              _btn("btn_l3", 20, "l3", "Click (L3)"))),
    GroupDef("rightStick", "Right stick", "Springs back to the centre (64 ≈ 5 V) when released.",
             (_cc("right_stick_x", 87, "X axis", AXIS), _cc("right_stick_y", 88, "Y axis", AXIS),
              _btn("btn_r3", 21, "r3", "Click (R3)"))),
    GroupDef("touchpad", "Touchpad", "X also scrubs a playing loop, Y picks the quantize grid. Both keep their value when you lift the finger.",
             (_cc("touchpad_x", 30, "X position", AXIS), _cc("touchpad_y", 31, "Y position", AXIS),
              _btn("btn_touchpad", 9, "touchpad", "Click"))),
    GroupDef("create", "Create", BUTTON_HINT, (_btn("btn_create", 26, "create"),)),
    GroupDef("options", "Options", BUTTON_HINT, (_btn("btn_options", 27, "options"),)),
    GroupDef("ps", "PS button", BUTTON_HINT, (_btn("btn_ps", 28, "ps"),)),
    GroupDef("mic", "Mic button", BUTTON_HINT + " (Read from the controller's HID report.)", (_btn("btn_mic", 29, "mic"),)),
    GroupDef("triangle", "△ Triangle", BUTTON_HINT, (_btn("btn_north", 22, "triangle"),)),
    GroupDef("circle", "○ Circle", BUTTON_HINT, (_btn("btn_east", 15, "circle"),)),
    GroupDef("cross", "✕ Cross", BUTTON_HINT, (_btn("btn_south", 14, "cross"),)),
    GroupDef("square", "□ Square", BUTTON_HINT, (_btn("btn_west", 23, "square"),)),
    GroupDef("motion", "Motion", "Gyro / tilt as 14-bit values → 0–10 V in fine steps. Sent while motion control is on.",
             (_nrpn("tilt_x", 0, "Tilt X"), _nrpn("tilt_y", 1, "Tilt Y"), _nrpn("twist", 2, "Twist"))),
    GroupDef("slice", "Grain slice", "One 14-bit CV for 1024 slices: block of 128 (Create / Options by default) + section of 16 (D-pad ↔) + slice (left stick X).",
             (_nrpn("grain_slice", 3, "Slice 0–1023"),)),
]
GROUP = {g.id: g for g in GROUPS}
ALL_INPUTS: List[InputDef] = [i for g in GROUPS for i in g.inputs]
INPUT = {i.id: i for i in ALL_INPUTS}
_GROUP_OF = {i.id: g for g in GROUPS for i in g.inputs}


def group_of(i: InputDef) -> GroupDef:
    return _GROUP_OF[i.id]


def input_for_button(b: str) -> Optional[InputDef]:
    for i in ALL_INPUTS:
        if i.button == b:
            return i
    return None


def input_title(i: InputDef) -> str:
    """'Left stick x axis' / 'L2' (for lists and file comments)."""
    g = group_of(i)
    return f"{g.title} {i.label.lower()}" if len(g.inputs) > 1 else g.title


# ── Routes from other MIDI sources (pedals, EC4, 16n …) ─────────────────────────────────────────

EXT_CV, EXT_NRPN, EXT_TRIGGER, EXT_NOTE = "CC → CV", "NRPN → CV", "CC → Trigger", "Note → Trigger"
EXT_KINDS = [EXT_CV, EXT_NRPN, EXT_TRIGGER, EXT_NOTE]


@dataclass
class ExternalRoute:
    kind: str = EXT_CV
    channel: int = 2
    control: str = "1"       # number, or a name from CC_MAP / NRPN_MAP
    port: int = 1
    id: str = field(default_factory=lambda: uuid.uuid4().hex)


@dataclass
class PortUse:
    kind: str           # CV / TR
    port: int
    users: List[str] = field(default_factory=list)
    refs: List[tuple] = field(default_factory=list)   # ('dualsense', label, inputID, trigger) | ('builtin', name) | ('external', kind, ch, control)

    @property
    def id(self) -> str:
        return f"{self.kind}{self.port}"


def _strip(s: str) -> str:
    return s.strip(" \t")


def _int_or_none(s: str) -> Optional[int]:
    try:
        return int(_strip(s))
    except (TypeError, ValueError):
        return None


class Mapping:
    def __init__(self):
        self.cc_names: Dict[str, int] = {}
        self.nrpn_names: Dict[str, int] = {}
        self.long_press = 1.0
        self.controls_on_pi = True     # False: the SSH host has no controls support yet (routing only)
        self.load_defaults()

    def load_defaults(self):
        self.roles: Dict[str, str] = {"l2_trigger": CV, "nrpn_grain_slice": CV, "btn_south": TRIGGER,
                                      "dpad_vertical": OCTAVE, "r2_trigger": LFO_RATE, "btn_west": LFO_RESET}
        self.ports: Dict[str, int] = {"l2_trigger": 8, "nrpn_grain_slice": 3, "btn_south": 2}
        self.pitch_enabled = True
        self.pitch_port = 5
        self.zero_note = 60
        self.semis_per_step = 12
        self.lfo_port = 2
        self.lfo_min_hz = 0.25
        self.lfo_max_hz = 40.0
        self.lfo_pulse_ms = 5
        self.triggers_as_cv = True
        self.channel = 2
        self.externals: List[ExternalRoute] = []
        self.funcs: Dict[str, ButtonFunc] = default_funcs()
        self.chords: List[Chord] = default_chords()
        self.dpad_horizontal = "slice_section"
        self.left_stick_x = "slice_fine"

    # ── functions ──
    def fn(self, button: str) -> ButtonFunc:
        f = self.funcs.get(button)
        return ButtonFunc(f.tap, f.hold) if f else ButtonFunc("none", "none")

    def set_tap(self, a: str, button: str):
        f = self.fn(button)
        f.tap = a
        self.funcs[button] = f

    def set_hold(self, a: str, button: str):
        f = self.fn(button)
        f.hold = a
        self.funcs[button] = f

    def sends_midi(self, i: InputDef) -> bool:
        if i.button:
            return self.fn(i.button).tap == "midi"
        if i.id == "dpad_horizontal":
            return self.dpad_horizontal == "midi"
        if i.id == "left_stick_x":
            return self.left_stick_x == "midi"
        return True

    # ── routing access ──
    def role(self, i: InputDef) -> str:
        return self.roles.get(i.id, OFF)

    def effective_role(self, i: InputDef) -> str:
        return self.role(i) if self.sends_midi(i) else OFF

    def port(self, i: InputDef) -> int:
        return self.ports.get(i.id, 1)

    def set_role(self, r: str, i: InputDef):
        if role_exclusive(r):
            for k, v in list(self.roles.items()):
                if v == r:
                    self.roles[k] = OFF
        self.roles[i.id] = r
        if role_uses_port(r) and i.id not in self.ports:
            self.ports[i.id] = self.first_free_port()

    def number(self, i: InputDef) -> int:
        if i.kind == "cc":
            return self.cc_names.get(i.name or "", i.fallback)
        if i.kind == "nrpn":
            return self.nrpn_names.get(i.name or "", i.fallback)
        return i.fallback

    def input_with(self, role: str) -> Optional[InputDef]:
        for i in ALL_INPUTS:
            if self.effective_role(i) == role:
                return i
        return None

    def first_free_port(self) -> int:
        used = {u.port for u in self.port_map()}
        for p in range(1, 100):
            if p not in used:
                return p
        return 1

    @property
    def trigger_port_name(self) -> str:
        return "SC.CV" if self.triggers_as_cv else "SC.TR"

    def route_token(self, i: InputDef) -> str:
        r = self.effective_role(i)
        if r == OFF:
            return "—"
        if r == CV:
            return f"CV {self.port(i)}"
        if r in (TRIGGER, NOTE_TRIGGER):
            return f"TRIG › CV {self.port(i)}" if self.triggers_as_cv else f"TR {self.port(i)}"
        return {LFO_RATE: "LFO RATE", LFO_RESET: "LFO RESET", OCTAVE: "OCTAVE"}[r]

    def tokens(self, i: InputDef) -> List[Tuple[str, tuple]]:
        """What a control does, for the drawing: [(text, kind)], kind = ('off',) | ('route', role) | ('function',)."""
        if i.button:
            f = self.fn(i.button)
            out = []
            if f.tap == "midi":
                r = self.effective_role(i)
                out.append((self.route_token(i), ("off",) if r == OFF else ("route", r)))
            else:
                out.append((ACTION_TOKEN[f.tap], ("off",) if f.tap == "none" else ("function",)))
            if f.hold != "none":
                out.append(("HOLD " + ACTION_TOKEN[f.hold], ("function",)))
            return out
        if i.id == "dpad_horizontal" and self.dpad_horizontal != "midi":
            return [("SECTION ±", ("function",))]
        if i.id == "left_stick_x" and self.left_stick_x != "midi":
            return [("SLICE", ("function",))]
        r = self.effective_role(i)
        return [(self.route_token(i), ("off",) if r == OFF else ("route", r))]

    # ── port overview ──
    def port_map(self) -> List[PortUse]:
        m: Dict[str, PortUse] = {}

        def add(kind, port, who, ref):
            k = f"{kind}{port}"
            if k not in m:
                m[k] = PortUse(kind, port)
            m[k].users.append(who)
            m[k].refs.append(ref)

        tk = "CV" if self.triggers_as_cv else "TR"
        for i in ALL_INPUTS:
            g = group_of(i)
            who = f"{g.title} {i.label}" if len(g.inputs) > 1 else g.title
            r = self.effective_role(i)
            if r == CV:
                add("CV", self.port(i), who, ("dualsense", who, i.id, False))
            elif r in (TRIGGER, NOTE_TRIGGER):
                add(tk, self.port(i), who + " (trigger)", ("dualsense", who, i.id, True))
        if self.pitch_enabled:
            add("CV", self.pitch_port, "Pitch V/Oct", ("builtin", "Pitch V/Oct"))
        if self.input_with(LFO_RATE) is not None:
            add(tk, self.lfo_port, "LFO", ("builtin", "LFO"))
        for e in self.externals:
            what = "note" if e.kind == EXT_NOTE else ("NRPN" if e.kind == EXT_NRPN else "CC")
            who = f"ch {e.channel} {what} {e.control}"
            k = "note" if e.kind == EXT_NOTE else "nrpn" if e.kind == EXT_NRPN else "trigger" if e.kind == EXT_TRIGGER else "cc"
            add("CV" if e.kind in (EXT_CV, EXT_NRPN) else tk, e.port, who, ("external", k, e.channel, _strip(e.control)))
        return sorted(m.values(), key=lambda u: (u.kind, u.port))

    # ── load from the SSH host (JSON from scripts/er301_routes_tool.py export) ──
    def apply(self, e: dict):
        if e.get("cc_names") is not None:
            self.cc_names = e["cc_names"]
        if e.get("nrpn_names") is not None:
            self.nrpn_names = e["nrpn_names"]
        if e.get("long_press") is not None:
            self.long_press = float(e["long_press"])
        self.channel = e.get("channel") or 2
        self.roles, self.ports, self.externals = {}, {}, []

        def resolve(c, kind):
            if isinstance(c, int):
                return c
            return (self.nrpn_names if kind == "nrpn" else self.cc_names).get(c)

        def text(c):
            return str(c)

        def claim(entry, kind, r) -> bool:
            if entry["channel"] != self.channel:
                return False
            num = resolve(entry["control"], kind)
            if num is None:
                return False
            for i in ALL_INPUTS:
                if i.kind == kind and self.number(i) == num and self.role(i) == OFF and r in i.roles:
                    self.roles[i.id] = r
                    if role_uses_port(r):
                        self.ports[i.id] = entry["port"]
                    return True
            return False

        for x in e.get("cv") or []:
            if not claim(x, "cc", CV):
                self.externals.append(ExternalRoute(EXT_CV, x["channel"], text(x["control"]), x["port"]))
        for x in e.get("nrpn_cv") or []:
            if not claim(x, "nrpn", CV):
                self.externals.append(ExternalRoute(EXT_NRPN, x["channel"], text(x["control"]), x["port"]))
        for x in e.get("triggers") or []:
            if not claim(x, "cc", TRIGGER):
                self.externals.append(ExternalRoute(EXT_TRIGGER, x["channel"], text(x["control"]), x["port"]))
        for x in e.get("note_triggers") or []:
            self.externals.append(ExternalRoute(EXT_NOTE, x["channel"], text(x["control"]), x["port"]))
        p = e.get("pitch")
        if p:
            self.pitch_enabled = True
            self.pitch_port = p["port"]
            self.zero_note = p.get("zero_note") if p.get("zero_note") is not None else 60
            self.semis_per_step = p.get("semitones_per_step") if p.get("semitones_per_step") is not None else 12
            if p.get("octave_control") is not None:
                claim({"channel": self.channel, "control": p["octave_control"], "port": 0}, "cc", OCTAVE)
        else:
            self.pitch_enabled = False
        lf = e.get("lfo")
        if lf:
            self.lfo_port = lf["port"]
            self.lfo_min_hz = float(lf["min_hz"]) if lf.get("min_hz") is not None else 0.25
            self.lfo_max_hz = float(lf["max_hz"]) if lf.get("max_hz") is not None else 40.0
            self.lfo_pulse_ms = lf.get("pulse_ms") if lf.get("pulse_ms") is not None else 5
            if lf.get("rate_control") is not None:
                claim({"channel": self.channel, "control": lf["rate_control"], "port": 0}, "cc", LFO_RATE)
            if lf.get("reset_control") is not None:
                claim({"channel": self.channel, "control": lf["reset_control"], "port": 0}, "cc", LFO_RESET)
        self.triggers_as_cv = e.get("triggers_as_cv", True) if e.get("triggers_as_cv") is not None else True

        c = e.get("controls")
        if c:
            self.controls_on_pi = True
            f = default_funcs()
            for name, v in (c.get("buttons") or {}).items():
                tap = v.get("tap")
                hold = v.get("hold")
                f[name] = ButtonFunc(tap if tap in ACTIONS else "none", hold if hold in ACTIONS else "none")
            self.funcs = f
            self.chords = [Chord(x[0], x[1], x[2]) for x in c.get("chords") or [] if len(x) == 3 and x[2] in ACTIONS]
            self.dpad_horizontal = c.get("dpad_horizontal", "slice_section")
            self.left_stick_x = c.get("left_stick_x", "slice_fine")
        else:
            self.controls_on_pi = e.get("controls_error") is not None
            self.funcs, self.chords = default_funcs(), default_chords()
            self.dpad_horizontal, self.left_stick_x = "slice_section", "slice_fine"

    # ── write er301_routes.py ──
    def _ctl(self, i: InputDef) -> str:
        names = self.nrpn_names if i.kind == "nrpn" else self.cc_names
        if i.name and i.name in names:
            return f"'{i.name}'"
        return str(self.number(i))

    @staticmethod
    def _ext_literal(s: str) -> str:
        t = _strip(s)
        n = _int_or_none(t)
        if n is not None and t.lstrip("+-").isdigit():
            return str(n)
        clean = "".join(ch for ch in t if ch.isalnum() or ch == "_")
        return f"'{clean or 'unknown'}'"

    @staticmethod
    def _line(body: str, comment: str) -> str:
        pad = max(1, 30 - len(body))
        return "    " + body + " " * pad + "# " + comment + "\n"

    def python_source(self, notes: Optional[Dict[str, str]] = None) -> str:
        notes = notes or {}

        def noted(c, key, port_key=None):
            n = notes.get(key)
            c = c if n is None else c + " · " + n.replace("\n", " ")
            d = notes.get("er:" + port_key) if port_key else None
            return c if d is None else c + " » " + d.replace("\n", " ")

        s = (
            '"""\n'
            "ER-301 routing inside the i2c2midi (DualSense → ER-301 directly, no Teletype script).\n"
            "\n"
            "Written by Kontrolleur. You can still edit it by hand; the app reads it back.\n"
            "Saving makes the MIDI hub (midi-passthrough service) send it to the i2c2midi within a\n"
            "second; the i2c2midi keeps it across power cycles. Check by hand:\n"
            "\n"
            "    python3 ~/dualsense-eurorack-midi/scripts/er301_push.py\n"
            "\n"
            "Ports are the ER-301 SC.CV / SC.TR numbers (1–99) chosen as the source of a unit's input.\n"
            "Entries are (control, port) on CHANNEL, or (channel, control, port) for another channel.\n"
            '"""\n'
            "\n"
            f"CHANNEL = {self.channel}            # DualSense instrument channel\n"
            "\n"
        )
        tp = self.trigger_port_name

        def section(header, name, rows):
            nonlocal s
            s += header + f"\n{name} = [\n" + "".join(rows) + "]\n\n"

        def ext_key(e):
            return f"ext:{e.channel}:{_strip(e.control)}"

        L, eff = self._line, self.effective_role
        tk = "CV" if self.triggers_as_cv else "TR"
        rows = [L(f"({self._ctl(i)}, {self.port(i)}),", noted(f"{input_title(i)} → SC.CV {self.port(i)}", "ds:" + i.id, f"CV{self.port(i)}"))
                for i in ALL_INPUTS if i.kind == "cc" and eff(i) == CV]
        rows += [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} CC {e.control} → SC.CV {e.port}", ext_key(e), f"CV{e.port}"))
                 for e in self.externals if e.kind == EXT_CV]
        section("# CC → SC.CV   (0–127 → 0–10 V)", "CV", rows)

        rows = [L(f"({self._ctl(i)}, {self.port(i)}),", noted(f"{input_title(i)} → SC.CV {self.port(i)}", "ds:" + i.id, f"CV{self.port(i)}"))
                for i in ALL_INPUTS if i.kind == "nrpn" and eff(i) == CV]
        rows += [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} NRPN {e.control} → SC.CV {e.port}", ext_key(e), f"CV{e.port}"))
                 for e in self.externals if e.kind == EXT_NRPN]
        section("# 14-bit NRPN → SC.CV   (0–16383 → 0–10 V, full resolution)", "NRPN_CV", rows)

        rows = [L(f"({self._ctl(i)}, {self.port(i)}),", noted(f"{input_title(i)} → {tp} {self.port(i)}", "ds:" + i.id, tk + str(self.port(i))))
                for i in ALL_INPUTS if i.kind == "cc" and eff(i) == TRIGGER]
        rows += [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} CC {e.control} → {tp} {e.port}", ext_key(e), tk + str(e.port)))
                 for e in self.externals if e.kind == EXT_TRIGGER]
        section("# CC as gate (≥ 64 = high) → trigger port", "TRIGGERS", rows)

        rows = [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} note {e.control} → {tp} {e.port}", ext_key(e), tk + str(e.port)))
                for e in self.externals if e.kind == EXT_NOTE]
        section("# Note held → trigger port. These notes do not change the pitch.", "NOTE_TRIGGERS", rows)

        s += "# Notes on CHANNEL → V/Oct. The octave control adds steps of semitones_per_step.\n"
        if self.pitch_enabled:
            oc = self.input_with(OCTAVE)
            ocs = self._ctl(oc) if oc else "None"
            s += f"PITCH = dict(port={self.pitch_port}, zero_note={self.zero_note}, octave_control={ocs}, semitones_per_step={self.semis_per_step})\n\n"
        else:
            s += "PITCH = None\n\n"

        s += "# Trigger LFO: rate control 0 = stopped, 1–127 = min_hz … max_hz (exponential).\n"
        rate = self.input_with(LFO_RATE)
        if rate:
            reset = self.input_with(LFO_RESET)
            rs = self._ctl(reset) if reset else "None"
            s += f"LFO = dict(port={self.lfo_port}, rate_control={self._ctl(rate)}, reset_control={rs},\n"
            s += f"           min_hz={'%g' % self.lfo_min_hz}, max_hz={'%g' % self.lfo_max_hz}, pulse_ms={self.lfo_pulse_ms})\n\n"
        else:
            s += "LFO = None\n\n"

        s += "# True: trigger ports are SC.CV ports (10 V pulse / gate). False: SC.TR ports.\n"
        s += f"TRIGGERS_AS_CV = {'True' if self.triggers_as_cv else 'False'}\n"
        return s

    # ── write controls.py ──
    def controls_source(self) -> str:
        s = (
            '"""\n'
            "What the DualSense buttons do. Read by the DualSense script; changes apply within a second.\n"
            "Written by Kontrolleur. You can still edit it by hand; the app reads it back.\n"
            "\n"
            "tap  = short press   (loop actions fire when you let go)\n"
            "hold = held for LONG_PRESS_DURATION (mappings.py) or longer, decided when you let go\n"
            "'midi' = the button sends its own CC (127 pressed / 0 released): route it to the ER-301\n"
            "         in er301_routes.py. Buttons: cross circle triangle square l1 r1 l3 r3 create\n"
            "         options ps mic touchpad (= touchpad click).\n"
            "Actions: none midi loop_record loop_play loop_clear motion block_up block_down block_reset\n"
            "         section_up section_down strum freeze_left freeze_right quantize\n"
            '"""\n'
            "\n"
            "BUTTONS = {\n"
        )
        for name, _ in BUTTONS:
            f = self.fn(name)
            key = f"'{name}':"
            body = f"dict(tap='{f.tap}'"
            if f.hold != "none":
                body += f", hold='{f.hold}'"
            body += "),"
            s += "    " + key + " " * max(1, 12 - len(key)) + body + "\n"
        s += "}\n\n# Two buttons pressed together. The buttons' own release actions are skipped then.\nCHORDS = [\n"
        for c in self.chords:
            s += f"    ('{c.a}', '{c.b}', '{c.action}'),\n"
        s += "]\n\n"
        s += f"DPAD_HORIZONTAL = '{self.dpad_horizontal}'   # 'slice_section' (16-slice sections) or 'midi' (CC steps)\n"
        s += f"LEFT_STICK_X = '{self.left_stick_x}'         # 'slice_fine' (slice within the section) or 'midi' (CC)\n"
        return s

    # ── CC → CV routes of other devices (16n faders, pedals, EC4 encoders) ──
    def _find_cc_route(self, channel: int, cc: int) -> int:
        for k, e in enumerate(self.externals):
            if e.kind == EXT_CV and e.channel == channel and _int_or_none(e.control) == cc:
                return k
        return -1

    def fader_port(self, channel: int, cc: int) -> Optional[int]:
        k = self._find_cc_route(channel, cc)
        return self.externals[k].port if k >= 0 else None

    def set_fader_port(self, channel: int, cc: int, port: Optional[int]):
        k = self._find_cc_route(channel, cc)
        if k >= 0:
            if port is not None:
                self.externals[k].port = port
            else:
                del self.externals[k]
        elif port is not None:
            self.externals.append(ExternalRoute(EXT_CV, channel, str(cc), port))

    def problems(self) -> List[str]:
        p = []
        if self.lfo_min_hz < 0.01 or self.lfo_max_hz > 163.83 or self.lfo_min_hz > self.lfo_max_hz:
            p.append("LFO range must be 0.01 ≤ min ≤ max ≤ 163.83 Hz.")
        for c in self.chords:
            if c.a == c.b:
                p.append(f"Combo {button_label(c.a)} + {button_label(c.b)} needs two different buttons.")
        for e in self.externals:
            t = _strip(e.control)
            n = _int_or_none(t)
            if n is not None:
                mx = 16383 if e.kind == EXT_NRPN else 127
                if n < 0 or n > mx:
                    p.append(f"{e.kind} ch {e.channel}: {n} is out of range 0–{mx}.")
            else:
                names = self.nrpn_names if e.kind == EXT_NRPN else self.cc_names
                if e.kind == EXT_NOTE or (names and t not in names):
                    what = "note number" if e.kind == EXT_NOTE else "control"
                    p.append(f"{e.kind} ch {e.channel}: “{t}” is not a known {what}.")
        return p

    # ── configuration files (same JSON as the Mac app's MappingSnapshot) ──
    def snapshot(self) -> dict:
        return {
            "channel": self.channel, "roles": dict(self.roles), "ports": dict(self.ports),
            "pitchEnabled": self.pitch_enabled, "pitchPort": self.pitch_port, "zeroNote": self.zero_note,
            "semisPerStep": self.semis_per_step, "lfoPort": self.lfo_port, "lfoMinHz": self.lfo_min_hz,
            "lfoMaxHz": self.lfo_max_hz, "lfoPulseMs": self.lfo_pulse_ms, "triggersAsCV": self.triggers_as_cv,
            "externals": [{"kind": e.kind, "channel": e.channel, "control": e.control, "port": e.port} for e in self.externals],
            "funcs": {k: {"tap": v.tap, "hold": v.hold} for k, v in self.funcs.items()},
            "chords": [{"a": c.a, "b": c.b, "action": c.action} for c in self.chords],
            "dpadHorizontal": self.dpad_horizontal, "leftStickX": self.left_stick_x, "longPress": self.long_press,
        }

    def restore(self, s: dict):
        self.channel = s["channel"]
        self.roles = {k: v for k, v in s["roles"].items() if v in ROLES}
        self.ports = dict(s["ports"])
        self.pitch_enabled, self.pitch_port = s["pitchEnabled"], s["pitchPort"]
        self.zero_note, self.semis_per_step = s["zeroNote"], s["semisPerStep"]
        self.lfo_port, self.lfo_min_hz, self.lfo_max_hz = s["lfoPort"], float(s["lfoMinHz"]), float(s["lfoMaxHz"])
        self.lfo_pulse_ms, self.triggers_as_cv = s["lfoPulseMs"], s["triggersAsCV"]
        self.externals = [ExternalRoute(e["kind"], e["channel"], e["control"], e["port"])
                          for e in s["externals"] if e["kind"] in EXT_KINDS]
        f = default_funcs()
        for k, v in s["funcs"].items():
            if v["tap"] in ACTIONS and v["hold"] in ACTIONS:
                f[k] = ButtonFunc(v["tap"], v["hold"])
        self.funcs = f
        self.chords = [Chord(c["a"], c["b"], c["action"]) for c in s["chords"] if c["action"] in ACTIONS]
        self.dpad_horizontal, self.left_stick_x = s["dpadHorizontal"], s["leftStickX"]
        self.long_press = float(s["longPress"])
