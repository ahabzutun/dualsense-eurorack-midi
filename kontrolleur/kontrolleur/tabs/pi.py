"""Routing tab: where the MIDI goes. The controllers (DualSense, 16n, pedals (MPC-20), EC4) on the left; the
Raspberry Pi with the DualSense script (its functions as modules) and the MIDI hub; the i2c2midi with its LFO on
the I2C bus with the ER-301 and the Teletype; the Percussa SSP. Boxes can be dragged and folded; SCRIPTS shows how
the two services on the Pi work together."""
from __future__ import annotations

import json
import os
from typing import List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath
from PySide6.QtWidgets import QLineEdit, QPushButton, QVBoxLayout, QWidget

from .. import draw as D
from .. import model as M
from .. import ssp as S
from .. import widgets as W
from ..devices import pedal_name
from ..stage import Drawing
from ..theme import BOLD, HEAVY, REGULAR, SEMIBOLD, Theme

# key, title, subtitle, colour, jacks
MODULES = [
    ("slice", "SLICE-SELEKTEUR", "Pi · grain slice 0–1023", "octave",
     [("slice", "SLICE"), ("section", "SECTION"), ("block", "BLOCK")]),
    ("loop", "LOOP-REKORDEUR", "Pi · per MIDI channel", "trig",
     [("rec", "REC"), ("play", "PLAY"), ("delete", "DELETE"), ("scrub", "SCRUB"), ("grid", "Q-GRID"), ("quantize", "QUANTIZE")]),
    ("lfo", "LFO", "i2c2midi · trigger LFO", "special", [("rate", "RATE"), ("reset", "RESET")]),
    ("extras", "DUALSENSE-EXTRAS", "Pi · DualSense script", "function",
     [("motion", "MOTION"), ("strum", "STRUM"), ("freeze_l", "FREEZE L"), ("freeze_r", "FREEZE R")]),
]
MOD_NAME = {"slice": "Slice-Selekteur", "loop": "Loop-Rekordeur", "lfo": "LFO", "extras": "DualSense-Extras",
            "ssp": "Percussa SSP"}
PORT_OF = {
    "slice_fine": ("slice", "slice"), "slice_section": ("slice", "section"), "section_up": ("slice", "section"),
    "section_down": ("slice", "section"), "slice_block": ("slice", "block"), "block_up": ("slice", "block"),
    "block_down": ("slice", "block"), "block_reset": ("slice", "block"),
    "loop_record": ("loop", "rec"), "loop_play": ("loop", "play"), "loop_clear": ("loop", "delete"),
    "loop_scrub": ("loop", "scrub"), "quantize_grid": ("loop", "grid"), "quantize": ("loop", "quantize"),
    "er_lfo_rate": ("lfo", "rate"), "er_lfo_reset": ("lfo", "reset"),
    "motion": ("extras", "motion"), "strum": ("extras", "strum"),
    "freeze_left": ("extras", "freeze_l"), "freeze_right": ("extras", "freeze_r"),
}
FUNCS_OF = {}
for _f, _jack in PORT_OF.items():
    FUNCS_OF.setdefault(_jack, []).append(_f)

SOURCES = [("dualsense", "DUALSENSE")] + [(d, t.upper()) for d, t in M.PI_DEVICES]
LFO_TOKEN = {"er_lfo_rate": "LFO RATE", "er_lfo_reset": "LFO RESET"}

SRC_X, SRC_W = 24.0, 330.0
MOD_W = 270.0


def _op(c: QColor, o: float) -> QColor:
    q = QColor(c)
    q.setAlphaF(max(0.0, min(1.0, c.alphaF() * o)))
    return q


SSP_COLOR = "note"
SSP_MAX_JACKS = 9


def modules(ctx) -> list:
    """The Pi modules + the SSP with one jack per module of its patch that listens to MIDI."""
    st = ctx.ssp
    p = st.active
    jacks = [(S.module_key(m), S.module_title(m).upper()) for m in S.midi_modules(p)]
    if len(jacks) > SSP_MAX_JACKS:
        jacks = jacks[:SSP_MAX_JACKS - 1] + [("_more", f"+ {len(jacks) - SSP_MAX_JACKS + 1} MORE")]
    if p:
        sub = f"patch {S.label(p)}" + ("" if st.pick or not st.current.get("file") else " · loaded")
    elif st.loaded:
        sub = "pick a patch" if st.presets else "no patches on the card"
    else:
        sub = "not loaded · " + ("connecting …" if st.loading else "click to connect")
    return MODULES + [("ssp", "PERCUSSA SSP", sub, SSP_COLOR, jacks)]


def module_color(mod: str) -> QColor:
    if mod == "ssp":
        return Theme.c(SSP_COLOR)
    return Theme.c(next(c for k, _t, _s, c, _p in MODULES if k == mod))


def token(f: str) -> str:
    if f.startswith("ssp:"):
        return f[4:]
    return LFO_TOKEN.get(f) or M.AXIS_TOKEN.get(f) or M.ACTION_TOKEN.get(f, f.upper())


def func_title(f: str) -> str:
    return M.FUNC_TITLE.get(f) or M.ACTION_TITLE.get(f, f)


# ── the controls of the other devices ──────────────────────────────────────────────────────────

def controls_of(ctx, device: str) -> List[Tuple[str, int, str, int]]:
    """(label, channel as sent, 'cc' | 'note', number) of the controls the app knows, without duplicates.
    EC4: the setup and group picked in the EC4 tab."""
    out, seen = [], set()

    def add(label, ch, kind, num):
        if (ch, kind, num) not in seen:
            seen.add((ch, kind, num))
            out.append((label, ch, kind, num))

    if device == "16n":
        for i, f in enumerate(ctx.bank.faders):
            add(f"Fader {i + 1}", f["usb_channel"], "cc", f["usb_cc"])
    elif device == "pedal":
        for inp in ctx.pedals.inputs:
            if inp["type_id"] == 0 and ctx.pedals.loaded:
                continue
            for s in inp["slots"]:
                if s["on"] and s["kind"] in ("cc", "note"):
                    add(f"{pedal_name(inp['input'])} · message {s['slot'] + 1}", s["channel"], s["kind"], s["number"])
    elif device == "ec4":
        if ctx.ec4.setups:
            g = ctx.ec4.setups[ctx.ec4_setup]["groups"][ctx.ec4_group]
            for k, e in enumerate(g["encoders"]):
                name = e["name"].strip()
                if e["type"] == "cc":
                    add(f"Encoder {k + 1}" + (f" “{name}”" if name else ""), e["channel"], "cc", e["number"])
            for k, e in enumerate(g["encoders"]):
                p = e["push"]
                if p["type"] in ("cc", "note"):
                    add(f"Push {k + 1}", p["channel"], p["type"], p["number"])
    return out


def source_label(ctx, device: str, ch: int, kind: str, num: int) -> str:
    for label, c, k, n in controls_of(ctx, device):
        if (c, k, n) == (ch, kind, num):
            return label
    if device == "ec4":                                   # another setup / group
        for si, su in enumerate(ctx.ec4.setups):
            for gi, g in enumerate(su["groups"]):
                for k, e in enumerate(g["encoders"]):
                    if kind == "cc" and e["type"] == "cc" and (e["channel"], e["number"]) == (ch, num):
                        return f"S{si + 1} G{gi + 1} encoder {k + 1}"
                    p = e["push"]
                    if p["type"] == kind and (p["channel"], p["number"]) == (ch, num):
                        return f"S{si + 1} G{gi + 1} push {k + 1}"
    return f"ch {ch} {'CC' if kind == 'cc' else 'note'} {num}"


def raw_label(ch: int, kind: str, num: int) -> str:
    return f"ch{ch} {'CC' if kind == 'cc' else 'N'}{num}"


# ── what is patched where ──────────────────────────────────────────────────────────────────────

class Patch:
    """One source → one function. kind: 'dualsense' | 'midi' | 'lfo'."""

    def __init__(self, device, label, func, ref, hold=False, jack=None):
        self.device, self.label, self.func, self.ref, self.hold = device, label, func, ref, hold
        self._jack = jack

    @property
    def jack(self):
        return self._jack or PORT_OF.get(self.func)


def patches(ctx) -> List[Patch]:
    from .dualsense import pill_keys, short_name
    m = ctx.mapping
    out = []
    for i in M.ALL_INPUTS:
        f = m.fn(i.button) if i.button else None
        for k in pill_keys(m, i):
            if k in PORT_OF:
                hold = f is not None and f.hold == k and f.tap != k
                out.append(Patch("dualsense", short_name(i) + (" hold" if hold else ""), k, ("input", i.id), hold))
    for c in m.chords:
        if c.action in PORT_OF:
            out.append(Patch("dualsense", f"{M.button_label(c.a)} + {M.button_label(c.b)}", c.action, ("chord", c.id)))
    for x in m.midi_inputs:
        out.append(Patch(x.device, source_label(ctx, x.device, x.channel, x.kind, x.number), x.function,
                         ("midi", x.device, x.channel, x.kind, x.number)))
    for f, src in (("er_lfo_rate", m.lfo_rate_src), ("er_lfo_reset", m.lfo_reset_src)):
        if src:
            out.append(Patch(src[0], source_label(ctx, src[0], src[1], "cc", src[2]), f, ("midi", src[0], src[1], "cc", src[2])))
    return out + ssp_patches(ctx)


def ssp_patches(ctx) -> List[Patch]:
    """Controllers whose MIDI the SSP's patch listens to (through the MIDI hub)."""
    mods = S.midi_modules(ctx.ssp.active)
    if not mods:
        return []
    snd = S.senders(ctx)
    keys = [S.module_key(m) for m in mods]
    shown = set(keys[:SSP_MAX_JACKS - 1] if len(keys) > SSP_MAX_JACKS else keys)
    groups = {}                     # (device, jack) → [labels], in order
    codes = {}
    for m in mods:
        k = S.module_key(m)
        jack = ("ssp", k if k in shown else "_more")
        codes[jack] = m["code"] if jack[1] != "_more" else "SSP"
        for i in m["inputs"]:
            for dev, lab in S.matches(snd, i):
                lst = groups.setdefault((dev, jack), [])
                if lab not in lst:
                    lst.append(lab)
    out = []
    for (dev, jack), labs in groups.items():
        if len(labs) > 2:           # e.g. every EC4 push that sends notes on that channel: one row
            out.append(Patch(dev, f"{labs[0]} + {len(labs) - 1} more", "ssp:" + codes[jack], ("ssp", jack[1]), jack=jack))
        else:
            out += [Patch(dev, lab, "ssp:" + codes[jack], ("ssp", jack[1]), jack=jack) for lab in labs]
    return out


def module_output(ctx, key: str) -> Tuple[str, bool, QColor]:
    """(text, on, colour) of where a module's result goes."""
    m = ctx.mapping
    if key == "slice":
        i = M.GROUP["slice"].inputs[0]
        on = m.effective_role(i) == M.CV
        return (f"SC.CV {m.port(i)}" if on else "not sent"), on, Theme.cv
    if key == "lfo":
        # coloured by the port it lands on, like the ER-301 tab: SC.CV = CV colour, SC.TR = trigger colour
        return ((f"SC.CV {m.lfo_port}" if m.triggers_as_cv else f"SC.TR {m.lfo_port}") if m.lfo_on else "off"), m.lfo_on, \
            (Theme.cv if m.triggers_as_cv else Theme.trig)
    if key == "loop":
        return f"MIDI ch {m.channel}", True, Theme.function
    if key == "extras":
        on = any(pt.jack in (("extras", "motion"), ("extras", "strum")) for pt in patches(ctx))
        return "motion · strum", on, Theme.function
    return "", False, Theme.dim


# ── drawing ────────────────────────────────────────────────────────────────────────────────────
# Groups, each moved by dragging it and folded with its ▾: the controllers · the Raspberry Pi (the
# DualSense script with the Pi functions + the MIDI hub; SCRIPTS opens up how the two work together) ·
# the i2c2midi (Teletype expander, with its LFO) · the ER-301 and the Teletype on its I2C bus · the SSP.
# MIDI for the i2c2midi and the SSP leaves the Pi through the hub (XIAO-TT / XIAO-SSP).

R_W = 420.0
ER_W, TT_W = 250.0, 152.0
ENV_HEAD = 34.0
HUB_W, HUB_WX = 214.0, 262.0
RT_W = 196.0
CARD_H = 62.0
MARGIN = 34.0
GROUPS = ("src", "pi", "i2c", "er", "tt", "ssp")
GROUP_NAME = {"src": "the controllers", "pi": "the Raspberry Pi", "i2c": "the i2c2midi", "er": "the ER-301",
              "tt": "the Teletype", "ssp": "the SSP"}
PI_MODS = ("slice", "loop", "extras")
ENV_OF = {"slice": "pi", "loop": "pi", "extras": "pi", "lfo": "i2c", "ssp": "ssp"}
DEV_HUB = {"dualsense": "script", "16n": "16n", "pedal": "pedal", "ec4": "ec4"}
HUB_IN = [("script", "DUALSENSE SCRIPT"), ("16n", "16N"), ("pedal", "PEDALS"), ("ec4", "EC4")]
HUB_IN_X = HUB_IN + [("nerdseq", "NERDSEQ")]
HUB_OUT = [("tt", "XIAO-TT", "→ i2c2midi"), ("ssp", "XIAO-SSP", "→ SSP")]
HUB_OUT_X = HUB_OUT + [("nerdseq", "NERDSEQ", "if plugged in")]
# what midi_passthrough does on the way (scripts view)
HUB_STEPS = [
    ("pedal", "pedals ch 1 → DualSense channel", "reads /tmp/dualsense_channel"),
    ("16n", "16n → SSP: fader values rescaled", "the other outputs get them as sent"),
    ("tt", "→ XIAO-TT: CC floods thinned out", "keeps the 115 200-baud link clear"),
    ("er", "ER-301 routes → i2c2midi setup", "config/er301_routes.py, re-read live"),
    ("osc", "OSC tap → OpenMW on the Mac", "UDP, adds no MIDI port"),
]
# the parts of the DualSense script next to its routers (scripts view)
SCRIPT_CARDS = [
    ("chan", "CHANNELS", "ch 1–3 · freeze · a loop each", "Create / Options switch"),
    ("clock", "CLOCK", "NerdSEQ clock, else 120 BPM", "times loop quantize"),
    ("hid", "LED · RUMBLE", "back to the DualSense", "channel colour, dots, haptics"),
]
FOLD_KEYS = ["src_" + d for d, _ in SOURCES] + ["pi", "i2c", "er", "tt", "ssp"]


def _opaque(p, path, tint: QColor, alpha: float):
    p.fillPath(path, Theme.bg)
    p.fillPath(path, _op(tint, alpha))


def _curve(a: QPointF, b: QPointF, bend: float = 0.45) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(a)
    dx = max(50.0, abs(b.x() - a.x()) * bend)
    path.cubicTo(QPointF(a.x() + dx, a.y()), QPointF(b.x() - dx, b.y()), b)
    return path


def _short(a: QPointF, b: QPointF, k: float = 0.5) -> QPainterPath:
    """A short cable inside a box (no minimum bend)."""
    path = QPainterPath()
    path.moveTo(a)
    dx = abs(b.x() - a.x()) * k
    path.cubicTo(QPointF(a.x() + dx, a.y()), QPointF(b.x() - dx, b.y()), b)
    return path


def _vcurve(a: QPointF, b: QPointF) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(a)
    dy = max(24.0, abs(b.y() - a.y()) * 0.5)
    path.cubicTo(QPointF(a.x(), a.y() + dy), QPointF(b.x(), b.y() - dy), b)
    return path


def _line(x1, y1, x2, y2) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(x1, y1)
    path.lineTo(x2, y2)
    return path


def _jack(p, c: QPointF, col: QColor, on: bool, d: float = 13):
    D.circle(p, c, d, col if on else Theme.bg, col if on else Theme.c("body", 0.5), 1.4)
    D.circle(p, c, d * 0.38, Theme.bg if on else None, None if on else Theme.c("body", 0.4), 1)


def _chevron(p, c: QPointF, open_: bool, col: QColor):
    path = QPainterPath()
    s = 4.2
    if open_:
        path.moveTo(c.x() - s, c.y() - s * 0.5)
        path.lineTo(c.x(), c.y() + s * 0.6)
        path.lineTo(c.x() + s, c.y() - s * 0.5)
    else:
        path.moveTo(c.x() - s * 0.5, c.y() - s)
        path.lineTo(c.x() + s * 0.6, c.y())
        path.lineTo(c.x() - s * 0.5, c.y() + s)
    p.strokePath(path, D.pen(col, 1.8))


def _arrow(p, tip: QPointF, towards_left: bool, col: QColor):
    s = 6.0
    path = QPainterPath()
    k = 1 if towards_left else -1
    path.moveTo(tip)
    path.lineTo(tip.x() + k * s, tip.y() - s * 0.6)
    path.lineTo(tip.x() + k * s, tip.y() + s * 0.6)
    path.closeSubpath()
    p.fillPath(path, col)


def _mod_jacks(r: QRectF, ports, rj: float) -> dict:
    return {pk: QPointF(r.left(), r.top() + 44 + rj * (k + 0.5)) for k, (pk, _l) in enumerate(ports)}


class PiDrawing(Drawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self.view = self._load()
        self._S = None
        self._frozen = None            # (base, shift) while a group is dragged

    # ── the view: positions, folds, scripts ──
    def _load(self) -> dict:
        raw = os.environ.get("KONTROLLEUR_ROUTING") or self.ctx.get("routingView", "")
        try:
            v = json.loads(raw) if raw else {}
        except (TypeError, ValueError):
            v = {}
        off = {}
        for k, x in (v.get("off") or {}).items():
            try:
                if k in GROUPS:
                    off[k] = [float(x[0]), float(x[1])]
            except (TypeError, ValueError, IndexError):
                pass
        return {"off": off, "fold": {k for k in (v.get("fold") or []) if k in FOLD_KEYS}, "scripts": bool(v.get("scripts"))}

    def _save(self):
        self.ctx.s.setValue("routingView", json.dumps({"off": self.view["off"], "fold": sorted(self.view["fold"]),
                                                       "scripts": self.view["scripts"]}))

    def _changed(self):
        self._save()
        self.update()
        try:
            if self.ctx.mode == "pi":
                self.ctx.inspector.rebuild()
        except AttributeError:
            pass

    def folded(self, key: str) -> bool:
        return key in self.view["fold"]

    def toggle_fold(self, key: str):
        self.view["fold"] ^= {key}
        self._changed()

    def set_scripts(self, on: bool):
        self.view["scripts"] = bool(on)
        self.view["fold"].discard("pi")
        self._changed()

    def fold_all(self, fold: bool):
        self.view["fold"] = set(FOLD_KEYS) if fold else set()
        self._changed()

    def reset_layout(self):
        self.view["off"] = {}
        self._changed()

    @property
    def any_open(self) -> bool:
        return len(self.view["fold"]) < len(FOLD_KEYS)

    def actions(self):
        return [("fold", "Fold all" if self.any_open else "Unfold all",
                 "Fold every box to its title (or open them all again). ▾ on a box folds just that one.",
                 lambda: self.fold_all(self.any_open)),
                ("reset", "Reset layout", "Put the boxes back where they started (drag a box to move it)",
                 self.reset_layout)]

    def drag_at(self, pt: QPointF):
        S = self._S
        if not S:
            return None
        for g in reversed(GROUPS):
            if S["g"][g].contains(pt):
                return self._mover(g)
        return None

    def _mover(self, g: str):
        start = list(self.view["off"].get(g, [0.0, 0.0]))

        def move(d):
            if d is None:
                self._frozen = None
                self.clip = True
                self._save()
                self.update()
                return
            if self._frozen is None:
                S = self._S or self._scene()
                self._frozen = (S["size"], S["shift"])
                self.clip = False
            self.view["off"][g] = [round(start[0] + d.x(), 1), round(start[1] + d.y(), 1)]
            self.update()
        return move

    @property
    def sel(self) -> str:
        return self.ctx.pi_sel

    def _row_h(self) -> float:
        return max(22.0, Theme.font(10.5, SEMIBOLD).pixelSize() * 1.25 + 8)

    def _mod_h(self, ports) -> float:
        return 44 + (self._row_h() + 2) * max(1, len(ports)) + 8

    def _er_rows(self) -> List[Tuple[str, str, QColor, bool]]:
        """ER-301 ports reached through the i2c2midi: (port, what, colour, from the LFO?)."""
        m = self.ctx.mapping
        rows = []
        text, on, col = module_output(self.ctx, "slice")
        if on:
            rows.append((text, "grain slice (Pi)", col, False))
        text, on, col = module_output(self.ctx, "lfo")
        if on:
            rows.append((text, "LFO pulses (i2c2midi)", col, True))
        n = len(m.port_map()) - len(rows)
        rows.append((f"+ {n}" if n > 0 else "—", "more ports routed (ER-301 tab)" if n > 0 else "no other routes", Theme.dim, False))
        return rows

    @property
    def base(self):
        if self._frozen is not None:
            return self._frozen[0]
        return self._scene()["size"]

    # ── geometry: every box and jack, before anything is painted ──
    def _scene(self) -> dict:
        ctx, view = self.ctx, self.view
        rh = self._row_h()
        rj = rh + 2
        head = 44 * Theme.grow ** 0.5
        step_h = rh * 1.6
        mods = {m[0]: m for m in modules(ctx)}
        pts = patches(ctx)
        by_dev = {}
        for pt in pts:
            by_dev.setdefault(pt.device, []).append(pt)
        S = {"mods": mods, "pts": pts, "by_dev": by_dev, "used": {pt.jack for pt in pts}, "rh": rh, "rj": rj,
             "head": head, "step_h": step_h, "jacks": {}, "outs": {}}

        def mod_h(k):
            return self._mod_h(mods[k][4])

        # sizes
        cards, y = [], 24.0
        for dev, title in SOURCES:
            fold = self.folded("src_" + dev)
            n = len(by_dev.get(dev, []))
            h = head + 8 if fold else head + rh * max(1, n) + 8
            cards.append((dev, title, y, h, fold))
            y += h + 14
        size = {"src": (SRC_W, y - 14)}

        mode = "fold" if self.folded("pi") else ("scripts" if view["scripts"] else "normal")
        mods_h = sum(mod_h(k) for k in PI_MODS) + 18 * (len(PI_MODS) - 1)
        if mode == "fold":
            pw, ph = 296.0, ENV_HEAD + 8 + 4 * rj + rh + 18
            aw = ah = hw = hub_h = col1 = 0.0
        else:
            if mode == "normal":
                aw, ah, col1 = 16 + MOD_W + 52, 34 + mods_h + 16, 0.0
                hw, nin, nout, nsteps = HUB_W, len(HUB_IN), len(HUB_OUT), 0
            else:
                rt_h = 44 + 4 * rj + 34
                col1 = rt_h + len(SCRIPT_CARDS) * (CARD_H + 12)
                aw = 16 + RT_W + 54 + MOD_W + 52
                ah = 34 + max(col1, mods_h) + 16 + 26           # + the lane for the channel file
                hw, nin, nout, nsteps = HUB_WX, len(HUB_IN_X), len(HUB_OUT_X), len(HUB_STEPS)
            hub_h = 44 + rj * nin + (28 + nsteps * step_h if nsteps else 0) + 30 + rj * nout + 10
            pw = 14 + aw + 60 + hw + 16
            ph = ENV_HEAD + 6 + max(ah, hub_h) + 14
        size["pi"] = (pw, ph)
        er_rows = self._er_rows()
        S["er_rows"] = er_rows
        size["i2c"] = (R_W, ENV_HEAD + 8) if self.folded("i2c") else (R_W, ENV_HEAD + mod_h("lfo") + 18)
        size["er"] = (ER_W, 40.0) if self.folded("er") else (ER_W, 40 + rh * len(er_rows) + 10)
        size["tt"] = (TT_W, 40.0) if self.folded("tt") else (TT_W, 78.0)
        size["ssp"] = (R_W, ENV_HEAD + 8) if self.folded("ssp") else (R_W, ENV_HEAD + mod_h("ssp") + 18)

        # default places, then where the user dragged them
        right_h = size["i2c"][1] + 44 + max(size["er"][1], size["tt"][1]) + 30 + size["ssp"][1]
        cy = max(size["src"][1], ph, right_h) / 2
        rx = SRC_W + 50 + pw + 74
        top = cy - right_h / 2
        pos = {"src": (0.0, cy - size["src"][1] / 2), "pi": (SRC_W + 50, cy - ph / 2), "i2c": (rx, top)}
        er_y = top + size["i2c"][1] + 44
        pos["er"] = (rx, er_y)
        pos["tt"] = (rx + R_W - TT_W, er_y)
        pos["ssp"] = (rx, er_y + max(size["er"][1], size["tt"][1]) + 30)
        g = {}
        for k in GROUPS:
            ox, oy = view["off"].get(k, (0.0, 0.0))
            g[k] = QRectF(pos[k][0] + ox, pos[k][1] + oy, *size[k])
        box = QRectF(g["src"])
        for r in g.values():
            box = box.united(r)
        if self._frozen is not None:
            S["size"], (sx, sy) = self._frozen
        else:
            W_, H_ = max(1000.0, box.width() + 2 * MARGIN), max(620.0, box.height() + 2 * MARGIN)
            S["size"] = (W_, H_)
            sx, sy = (W_ - box.width()) / 2 - box.left(), (H_ - box.height()) / 2 - box.top()
        S["shift"] = (sx, sy)
        for k in GROUPS:
            g[k].translate(sx, sy)
        S["g"] = g

        # the controllers' cards and the jacks of what is patched
        src = g["src"]
        S["cards"], S["src_jacks"] = [], []
        for dev, title, cyy, h, fold in cards:
            r = QRectF(src.left(), src.top() + cyy, SRC_W, h)
            items = by_dev.get(dev, [])
            S["cards"].append((dev, title, r, fold, items))
            ry = r.top() + head
            for pt in items:
                if fold:
                    S["src_jacks"].append((QPointF(r.right(), r.top() + h / 2), pt))
                else:
                    S["src_jacks"].append((QPointF(r.right(), ry + rh / 2), pt))
                    ry += rh

        # the Raspberry Pi
        pr = g["pi"]
        pi = {"mode": mode, "r": pr}
        if mode == "fold":
            y0 = pr.top() + ENV_HEAD + 8
            pi["in"] = {d: QPointF(pr.left(), y0 + rj * (k + 0.5)) for k, (d, _t) in enumerate(SOURCES)}
            pi["out"] = {k: QPointF(pr.right(), y0 + rj * (i + 0.5)) for i, (k, *_r) in enumerate(HUB_OUT)}
        else:
            body = ph - ENV_HEAD - 6 - 14
            ar = QRectF(pr.left() + 14, pr.top() + ENV_HEAD + 6 + (body - ah) / 2, aw, ah)
            pi["A"] = ar
            inner = ah - 34 - 16 - (26 if mode == "scripts" else 0)
            if mode == "normal":
                mx, my = ar.left() + 16, ar.top() + 34
            else:
                mx = ar.left() + 16 + RT_W + 54
                my = ar.top() + 34 + max(0.0, (inner - mods_h) / 2)
                ry = ar.top() + 34 + max(0.0, (inner - col1) / 2)
                rt = QRectF(ar.left() + 16, ry, RT_W, 44 + 4 * rj + 34)
                pi["rt"] = rt
                pi["rt_ds"] = QPointF(rt.left(), rt.top() + 44 + rj * 0.5)
                pi["rt_dev"] = QPointF(rt.left(), rt.top() + 44 + rj * 1.5)
                pi["rt_fn"] = QPointF(rt.right(), rt.top() + 44 + rj * 2.5)
                pi["rt_cc"] = QPointF(rt.right(), rt.top() + 44 + rj * 3.5)
                cy2 = rt.bottom() + 12
                pi["cards"] = {}
                for key, *_r in SCRIPT_CARDS:
                    pi["cards"][key] = QRectF(rt.left(), cy2, RT_W, CARD_H)
                    cy2 += CARD_H + 12
                pi["lane_y"] = ar.bottom() - 16
            mrects = {}
            yy = my
            for k in PI_MODS:
                r = QRectF(mx, yy, MOD_W, mod_h(k))
                mrects[k] = r
                for pk, c in _mod_jacks(r, mods[k][4], rj).items():
                    S["jacks"][(k, pk)] = c
                text, on, oc = module_output(ctx, k)
                if text:
                    S["outs"][k] = (QPointF(r.right(), r.top() + 44 + rj / 2), on, oc)
                yy += r.height() + 18
            pi["mods"] = mrects
            pi["A_out"] = QPointF(ar.right(), my + mods_h / 2 if mode == "normal" else my + mods_h / 2)
            hr = QRectF(ar.right() + 60, pr.top() + ENV_HEAD + 6 + (body - hub_h) / 2, hw, hub_h)
            pi["hub"] = hr
            ins, outs, steps = (HUB_IN, HUB_OUT, []) if mode == "normal" else (HUB_IN_X, HUB_OUT_X, HUB_STEPS)
            yy = hr.top() + 44
            pi["hub_in"] = {}
            for k, _t in ins:
                pi["hub_in"][k] = QPointF(hr.left(), yy + rj / 2)
                yy += rj
            pi["steps"] = {}
            if steps:
                pi["steps_y0"] = yy + 6
                yy += 28
                for k, *_r in steps:
                    pi["steps"][k] = QRectF(hr.left(), yy, hw, step_h)
                    yy += step_h
            pi["hub_out_y0"] = yy + 8
            yy += 30
            pi["hub_out"] = {}
            for k, *_r in outs:
                pi["hub_out"][k] = QPointF(hr.right(), yy + rj / 2)
                yy += rj
        S["pi"] = pi

        # right: i2c2midi (LFO), ER-301 + Teletype, SSP
        ir = g["i2c"]
        S["i2c_in"] = QPointF(ir.left(), ir.top() + ENV_HEAD / 2 + (6 if not self.folded("i2c") else 4))
        S["i2c_bus"] = QPointF(ir.center().x(), ir.bottom())
        if not self.folded("i2c"):
            r = QRectF(ir.left() + 18, ir.top() + ENV_HEAD, MOD_W, mod_h("lfo"))
            S["lfo_r"] = r
            for pk, c in _mod_jacks(r, mods["lfo"][4], rj).items():
                S["jacks"][("lfo", pk)] = c
            text, on, oc = module_output(ctx, "lfo")
            S["outs"]["lfo"] = (QPointF(r.right(), r.top() + 44 + rj / 2), on, oc)
        sr = g["ssp"]
        S["ssp_in"] = QPointF(sr.left(), sr.top() + ENV_HEAD / 2 + (6 if not self.folded("ssp") else 4))
        if not self.folded("ssp"):
            r = QRectF(sr.left() + 18, sr.top() + ENV_HEAD, R_W - 36, mod_h("ssp"))
            S["ssp_r"] = r
            for pk, c in _mod_jacks(r, mods["ssp"][4], rj).items():
                S["jacks"][("ssp", pk)] = c
        S["er_i2c"] = QPointF(g["er"].center().x(), g["er"].top())
        S["tt_i2c"] = QPointF(g["tt"].center().x(), g["tt"].top())
        return S

    # ── painting ──
    def paint(self, p: QPainter, w, h):
        S = self._scene()
        self._S = S
        self._environments(p, S)
        self._back_cables(p, S)
        self._sources(p, S)
        self._pi(p, S)
        self._right(p, S)
        self._front_cables(p, S)
        self._chrome(p, S)

    # ── the boxes around things (dashed) ──
    def _env(self, p, r: QRectF, title: str, sub: str, col: QColor, key: Optional[str] = None,
             strong: bool = False, chrome: float = 40):
        path = QPainterPath()
        path.addRoundedRect(r, 14, 14)
        if strong:
            D.glow(p, path, _op(col, 0.5), 10)
        p.fillPath(path, _op(col, 0.035))
        p.strokePath(path, D.pen(Theme.ink if strong else _op(col, 0.45), 1.8 if strong else 1.3, [5, 4]))
        t = D.Text(title, Theme.font(10, HEAVY, "mono"), _op(col, 0.95), tracking=1.4)
        t.paint(p, r.left() + 14, r.top() + 8)
        D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=max(20.0, r.width() - t.w - 40 - chrome), min_scale=0.7).paint(
            p, r.left() + 24 + t.w, r.top() + 8 + (t.h - Theme.font(9, SEMIBOLD, "mono").pixelSize() * 1.2) / 2)
        if key:
            self.hits.add(QRectF(r.left(), r.top(), r.width() - chrome, ENV_HEAD), lambda: self.ctx.select("pi_sel", key))

    def _environments(self, p, S):
        pi = S["pi"]
        if pi["mode"] != "fold":
            sub = "DualSense script · MIDI hub" if pi["mode"] == "normal" else \
                "two services · Kontrolleur writes their config over SSH, both re-read it live"
            self._env(p, pi["r"], "RASPBERRY PI", sub, Theme.function, "env_pi", self.sel == "env_pi", chrome=130)
            ar = pi["A"]
            path = QPainterPath()
            path.addRoundedRect(ar, 11, 11)
            p.fillPath(path, _op(Theme.function, 0.03))
            p.strokePath(path, D.pen(_op(Theme.function, 0.35), 1.1, [3, 3]))
            t = D.Text("DUALSENSE SCRIPT", Theme.font(9.5, HEAVY, "mono"), _op(Theme.function, 0.9), tracking=1.3)
            t.paint(p, ar.left() + 14, ar.top() + 9)
            D.Text("dualsense_midi.py" + (" · service dualsense-midi · out port DualSense_Controller"
                                          if pi["mode"] == "scripts" else " → port DualSense_Controller"),
                   Theme.font(8.5, SEMIBOLD, "mono"), Theme.dim, max_w=ar.width() - t.w - 40, min_scale=0.7).paint(
                p, ar.left() + 22 + t.w, ar.top() + 10)
            self.hits.add(QRectF(ar.left(), ar.top(), ar.width(), 30), lambda: self.ctx.select("pi_sel", "env_pi"),
                          "The DualSense script")
        ir, sr = S["g"]["i2c"], S["g"]["ssp"]
        lfo_used = sum(1 for j in S["used"] if j[0] == "lfo")
        self._env(p, ir, "I2C2MIDI", "Teletype expander · gets MIDI over USB (XIAO-TT)" if not self.folded("i2c") else
                  f"LFO · {lfo_used} input{'s' if lfo_used != 1 else ''} patched", module_color("lfo"), "mod_lfo",
                  self.sel == "mod_lfo" and self.folded("i2c"))
        st = self.ctx.ssp
        sub = "gets MIDI over USB (XIAO-SSP)"
        if st.loaded and not st.stale and not st.devices():
            sub = "⚠ no MIDI device plugged in"
        if self.folded("ssp"):
            n = len(S["mods"]["ssp"][4])
            sub = (S["mods"]["ssp"][2] + f" · {n} module{'s' if n != 1 else ''} listen") if n else S["mods"]["ssp"][2]
        self._env(p, sr, "PERCUSSA SSP", sub, module_color("ssp"), "mod_ssp", self.sel == "mod_ssp" and self.folded("ssp"))

    # ── left: the controllers ──
    def _sources(self, p, S):
        rh, head = S["rh"], S["head"]
        src = S["g"]["src"]
        D.Text("CONTROLLERS", Theme.font(10, HEAVY, "mono"), Theme.dim, tracking=1.4).paint(p, src.left(), src.top() + 2)
        for dev, title, r, fold, items in S["cards"]:
            key = "src_" + dev
            sel = self.sel == key
            path = QPainterPath()
            path.addRoundedRect(r, 10, 10)
            accent = Theme.function if items else Theme.c("body", 0.5)
            if sel:
                D.glow(p, path, _op(Theme.function, 0.6), 10)
            p.fillPath(path, Theme.bg)
            D.draw_path(p, path, _op(Theme.function, 0.06) if items else Theme.c("body", 0.035),
                        Theme.ink if sel else (_op(accent, 0.6) if items else Theme.c("body", 0.4)), 2 if sel else 1.2)
            t = D.Text(title, Theme.font(11, HEAVY, "mono"), Theme.ink if items or sel else Theme.dim, tracking=1.2)
            t.paint(p, r.left() + 14, r.top() + 9)
            sub = self._sub(dev)
            if fold:
                sub = f"{len(items)} patched · " + sub if items else "nothing patched"
            D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=SRC_W - 70, min_scale=0.8).paint(
                p, r.left() + 14, r.top() + 11 + t.h)
            self.hits.add(path, lambda key=key: self.ctx.select("pi_sel", key), title.title())
            if fold:
                if items:
                    cols = {module_color(pt.jack[0]).name() for pt in items}
                    col = module_color(items[0].jack[0]) if len(cols) == 1 else Theme.function
                    jc = QPointF(r.right(), r.center().y())
                    D.circle(p, jc, 11, col, _op(col, 0.9), 1.2)
                    D.circle(p, jc, 4, Theme.bg)
                continue
            ry = r.top() + head
            if not items:
                D.Text("— nothing patched", Theme.font(10, REGULAR, "mono"), Theme.dim).paint(p, r.left() + 14, ry + 3)
            for pt in items:
                col = module_color(pt.jack[0])
                tok = D.Pill(D.Text(token(pt.func), Theme.font(10, BOLD, "mono"), col), 6, 2, _op(col, 0.14))
                lab = D.Text(pt.label, Theme.font(10.5, SEMIBOLD), _op(Theme.ink, 0.9), max_w=SRC_W - 54 - tok.w, min_scale=0.8)
                cy = ry + rh / 2
                lab.paint(p, r.left() + 14, cy - lab.h / 2)
                tok.paint(p, r.right() - 22 - tok.w, cy - tok.h / 2)
                jc = QPointF(r.right(), cy)
                D.circle(p, jc, 9, col, _op(col, 0.9), 1.2)
                D.circle(p, jc, 3.5, Theme.bg)
                ry += rh

    def _sub(self, dev: str) -> str:
        m = self.ctx.mapping
        if dev == "dualsense":
            return f"ch {m.channel} · assigned in the DualSense tab"
        n = sum(1 for x in m.midi_inputs if x.device == dev)
        return {"16n": "16n faderbank", "pedal": "MPC-20 via DOREMiDi", "ec4": "Faderfox EC4"}[dev] + \
            ("" if n or dev not in ("16n", "ec4") else " · not listened to")

    # ── a plain card (router, script parts, folded boxes) ──
    def _card(self, p, r: QRectF, title: str, sub: str, col: QColor, on: bool = True, sel: bool = False,
              sub2: str = "", dash=None, title_w: Optional[float] = None) -> QPainterPath:
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        if sel:
            D.glow(p, path, _op(col, 0.7), 10)
        _opaque(p, path, col if on else Theme.c("body"), 0.07 if on else 0.035)
        p.strokePath(path, D.pen(Theme.ink if sel else (_op(col, 0.7) if on else Theme.c("body", 0.4)), 2 if sel else 1.3, dash))
        t = D.Text(title, Theme.font(11, HEAVY, "mono"), col if on or sel else Theme.dim, tracking=1.2,
                   max_w=title_w or (r.width() - 32), min_scale=0.7)
        t.paint(p, r.left() + 14, r.top() + 9)
        y = r.top() + 11 + t.h
        if sub:
            s = D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=r.width() - 28, min_scale=0.7)
            s.paint(p, r.left() + 14, y)
            y += s.h
        if sub2:
            D.Text(sub2, Theme.font(8.5, SEMIBOLD, "mono"), _op(Theme.dim, 0.75), max_w=r.width() - 28, min_scale=0.7).paint(
                p, r.left() + 14, y + 1)
        return path

    def _jack_label(self, p, c: QPointF, col: QColor, on: bool, label: str, right: bool = False,
                    max_w: Optional[float] = None, d: float = 11):
        _jack(p, c, col, on, d)
        lt = D.Text(label, Theme.font(10, HEAVY, "mono"), col if on else Theme.dim, max_w=max_w, min_scale=0.7)
        lt.paint(p, (c.x() - 16 - lt.w) if right else (c.x() + 16), c.y() - lt.h / 2)

    # ── modules ──
    def _module(self, p, mod, r: QRectF, used, S):
        key, title, sub, colname, ports = mod
        sel = self.sel == "mod_" + key
        col = Theme.c(colname)
        on_any = any((key, pk) in used for pk, _ in ports)
        rj = S["rj"]
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        if sel:
            D.glow(p, path, _op(col, 0.7), 10)
        _opaque(p, path, col if on_any else Theme.c("body"), 0.08 if on_any else 0.035)
        p.strokePath(path, D.pen(Theme.ink if sel else (_op(col, 0.75) if on_any else Theme.c("body", 0.4)), 2 if sel else 1.3))
        if key == "ssp":
            title, sub = sub.upper() if sub.startswith("patch") else "NO PATCH", "MIDI inputs of its modules"
            title = title.replace("PATCH ", "")
        elif S["pi"]["mode"] == "scripts" and key in SCRIPT_SUB:
            sub = SCRIPT_SUB[key]
        t = D.Text(title, Theme.font(11, HEAVY, "mono"), col if on_any or sel else Theme.dim, tracking=1.2,
                   max_w=r.width() - 32, min_scale=0.7)
        t.paint(p, r.left() + 16, r.top() + 9)
        D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=r.width() - 32, min_scale=0.7).paint(
            p, r.left() + 16, r.top() + 11 + t.h)
        for pk, plabel in ports:
            on = (key, pk) in used
            jc = S["jacks"][(key, pk)]
            _jack(p, jc, col, on)
            lt = D.Text(plabel, Theme.font(10, HEAVY, "mono"), col if on else Theme.dim, max_w=r.width() - 40, min_scale=0.7)
            lt.paint(p, r.left() + 18, jc.y() - lt.h / 2)
        out = S["outs"].get(key)
        if out:
            jc, on, oc = out
            _jack(p, jc, oc, on)
            text = module_output(self.ctx, key)[0]
            lt = D.Text(text.upper() if key == "extras" else "OUT", Theme.font(9 if key == "extras" else 10, HEAVY, "mono"),
                        Theme.dim, max_w=r.width() / 2, min_scale=0.7)
            lt.paint(p, r.right() - 22 - lt.w, jc.y() - lt.h / 2)
        self.hits.add(path, lambda key=key: self.ctx.select("pi_sel", "mod_" + key), MOD_NAME[key])

    # ── the Raspberry Pi ──
    def _pi(self, p, S):
        pi, used = S["pi"], S["used"]
        col = Theme.function
        if pi["mode"] == "fold":
            r = pi["r"]
            path = self._card(p, r, "RASPBERRY PI", "", col, True, self.sel == "env_pi", title_w=r.width() - 70)
            self.hits.add(path, lambda: self.ctx.select("pi_sel", "env_pi"), "The Raspberry Pi")
            via = {pt.device for pt in S["pts"]}
            for d, t in SOURCES:
                self._jack_label(p, pi["in"][d], col, d in via, t, max_w=110)
            for k, name, to in HUB_OUT:
                oc = module_color("lfo" if k == "tt" else "ssp")
                self._jack_label(p, pi["out"][k], oc, True, f"{name} {to}", right=True, max_w=150)
            inuse = {pt.jack[0] for pt in S["pts"]}
            fn = [MOD_NAME[k].split("-")[0] if k != "extras" else "Extras" for k in PI_MODS if k in inuse]
            text = ("functions in use: " + " · ".join(fn)) if fn else "no Pi function patched"
            D.Text(text, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=r.width() - 28, lines=1, min_scale=0.7).paint(
                p, r.left() + 14, r.bottom() - S["rh"] - 8)
            return
        for k in PI_MODS:
            self._module(p, S["mods"][k], pi["mods"][k], used, S)
        if pi["mode"] == "scripts":
            self._router(p, S)
        self._hub(p, S)
        # where the script's MIDI leaves it
        a = pi["A_out"]
        lt = D.Text("OUT", Theme.font(9, HEAVY, "mono"), Theme.dim, tracking=1.2)
        lt.paint(p, a.x() - 10 - lt.w, a.y() - 14 - lt.h)

    def _router(self, p, S):
        pi, rj = S["pi"], S["rj"]
        col = Theme.function
        rt = pi["rt"]
        pts = S["pts"]
        ds_on = any(pt.device == "dualsense" for pt in pts)
        dev_on = any(pt.device != "dualsense" and ENV_OF.get(pt.jack[0]) == "pi" for pt in pts)
        fn_on = any(ENV_OF.get(pt.jack[0]) == "pi" for pt in pts)
        cc_on = any(pt.device == "dualsense" and ENV_OF.get(pt.jack[0]) != "pi" for pt in pts)
        path = self._card(p, rt, "ROUTERS", "button router · axis router", col, True, False)
        self._jack_label(p, pi["rt_ds"], col, ds_on, "DUALSENSE", max_w=RT_W - 40)
        self._jack_label(p, pi["rt_dev"], col, dev_on, "16N · EC4 · PEDALS", max_w=RT_W - 40)
        self._jack_label(p, pi["rt_fn"], col, fn_on, "FUNCTIONS", right=True, max_w=RT_W - 40)
        self._jack_label(p, pi["rt_cc"], col, cc_on, "PLAIN CC · NRPN", right=True, max_w=RT_W - 40)
        chip = D.Pill(D.Text("config/controls.py · re-read live", Theme.font(8.5, BOLD, "mono"), _op(col, 0.9),
                             max_w=RT_W - 40, min_scale=0.7), 7, 3, _op(col, 0.1), _op(col, 0.35), 1, dash=[3, 2])
        chip.paint(p, rt.left() + 14, rt.bottom() - chip.h - 9)
        self.hits.add(path, lambda: self.ctx.select("pi_sel", "env_pi"),
                      "Buttons, holds and chords → actions; sticks, triggers, touchpad → functions or plain MIDI")
        for key, title, sub, sub2 in SCRIPT_CARDS:
            r = pi["cards"][key]
            path = self._card(p, r, title, sub, col, True, False, sub2)
            self.hits.add(path, lambda: self.ctx.select("pi_sel", "env_pi"), title.title())

    def _hub(self, p, S):
        pi, rj = S["pi"], S["rj"]
        r = pi["hub"]
        col = Theme.function
        scripts = pi["mode"] == "scripts"
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        _opaque(p, path, col, 0.06)
        p.strokePath(path, D.pen(_op(col, 0.7), 1.3))
        t = D.Text("MIDI HUB", Theme.font(11, HEAVY, "mono"), col, tracking=1.2)
        t.paint(p, r.left() + 16, r.top() + 9)
        D.Text("midi_passthrough.py" + (" · service midi-passthrough" if scripts else " on the Pi"),
               Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=r.width() - 30, min_scale=0.7).paint(p, r.left() + 16, r.top() + 11 + t.h)
        pts = S["pts"]
        via = {DEV_HUB[pt.device] for pt in pts if ENV_OF.get(pt.jack[0]) != "pi"}
        script_on = any(o[1] for k, o in S["outs"].items() if k in PI_MODS) or "script" in via
        for k, label in (HUB_IN_X if scripts else HUB_IN):
            on = k in via or (k == "script" and script_on)
            self._jack_label(p, pi["hub_in"][k], col, on, label + (" · if plugged in" if k == "nerdseq" else ""),
                             max_w=r.width() - 40)
        if scripts:
            D.Text("ON THE WAY", Theme.font(9, HEAVY, "mono"), Theme.dim, tracking=1.2).paint(p, r.left() + 16, pi["steps_y0"])
            for k, text, sub in HUB_STEPS:
                sr = pi["steps"][k]
                D.circle(p, QPointF(sr.left() + 20, sr.top() + 9), 4, _op(col, 0.7))
                D.Text(text, Theme.font(9.5, BOLD, "mono"), _op(Theme.ink, 0.85), max_w=sr.width() - 44, min_scale=0.7).paint(
                    p, sr.left() + 30, sr.top() + 1)
                D.Text(sub, Theme.font(8.5, SEMIBOLD, "mono"), Theme.dim, max_w=sr.width() - 44, min_scale=0.7).paint(
                    p, sr.left() + 30, sr.top() + 1 + Theme.font(9.5, BOLD, "mono").pixelSize() * 1.3)
        D.Text("OUT", Theme.font(9, HEAVY, "mono"), Theme.dim, tracking=1.2).paint(p, r.left() + 16, pi["hub_out_y0"])
        for k, name, to in (HUB_OUT_X if scripts else HUB_OUT):
            oc = module_color("lfo" if k == "tt" else "ssp") if k != "nerdseq" else Theme.c("body", 0.6)
            self._jack_label(p, pi["hub_out"][k], oc, k != "nerdseq", f"{name} {to}", right=True, max_w=r.width() - 40)
        self.hits.add(path, lambda: self.ctx.select("pi_sel", "env_pi"), "The MIDI hub")

    # ── right: i2c2midi, ER-301, Teletype, SSP ──
    def _right(self, p, S):
        used = S["used"]
        if "lfo_r" in S:
            self._module(p, S["mods"]["lfo"], S["lfo_r"], used, S)
        if "ssp_r" in S:
            self._module(p, S["mods"]["ssp"], S["ssp_r"], used, S)
        rh = S["rh"]
        er, tt = S["g"]["er"], S["g"]["tt"]
        path = QPainterPath()
        path.addRoundedRect(er, 10, 10)
        _opaque(p, path, Theme.cv, 0.05)
        p.strokePath(path, D.pen(_op(Theme.cv, 0.55), 1.3))
        t = D.Text("ER-301", Theme.font(11, HEAVY, "mono"), Theme.ink, tracking=1.2)
        t.paint(p, er.left() + 14, er.top() + 9)
        rows = S["er_rows"]
        if self.folded("er"):
            n = len(self.ctx.mapping.port_map())
            D.Text(f"{n} port{'s' if n != 1 else ''} from the i2c2midi", Theme.font(9, SEMIBOLD, "mono"), Theme.dim,
                   max_w=er.width() - t.w - 64, min_scale=0.7).paint(p, er.left() + 24 + t.w, er.top() + 12)
        else:
            y = er.top() + 40
            for port, what, c, _from_lfo in rows:
                pt_t = D.Text(port, Theme.font(10.5, BOLD, "mono"), c)
                pt_t.paint(p, er.left() + 14, y + (rh - pt_t.h) / 2)
                D.Text(what, Theme.font(9.5, SEMIBOLD, "mono"), Theme.dim, max_w=er.width() - 100, min_scale=0.7).paint(
                    p, er.left() + 90, y + (rh - Theme.font(9.5, SEMIBOLD, "mono").pixelSize() * 1.2) / 2)
                y += rh
        self.hits.add(QRectF(er.left(), er.top(), er.width() - 36, er.height()), lambda: self.ctx.set_mode("er301"), "ER-301 tab")
        path = QPainterPath()
        path.addRoundedRect(tt, 10, 10)
        _opaque(p, path, Theme.c("body"), 0.04)
        p.strokePath(path, D.pen(Theme.c("body", 0.45), 1.2, [4, 3]))
        t = D.Text("TELETYPE", Theme.font(11, HEAVY, "mono"), Theme.ink, tracking=1.2, max_w=tt.width() - 50, min_scale=0.7)
        t.paint(p, tt.left() + 14, tt.top() + 9)
        if not self.folded("tt"):
            D.Text("I2C leader", Theme.font(9, SEMIBOLD, "mono"), Theme.dim).paint(p, tt.left() + 14, tt.top() + 13 + t.h)
            D.Text("shown only", Theme.font(9, SEMIBOLD, "mono"), _op(Theme.dim, 0.7)).paint(p, tt.left() + 14, tt.top() + 27 + t.h)
        self.hits.add(QRectF(tt), None, "The Teletype leads the I2C bus (shown only, nothing to set here)")

    # ── cables ──
    def _cable_ends(self, S):
        """(start, end, colour, hot, in front?) of every controller cable."""
        pi = S["pi"]
        mode = pi["mode"]
        out = []
        for a, pt in S["src_jacks"]:
            mod = pt.jack[0]
            env = ENV_OF.get(mod)
            if mode == "fold":
                b, front = pi["in"][pt.device], True
            elif mode == "scripts" and (env == "pi" or pt.device == "dualsense"):
                b, front = (pi["rt_ds"] if pt.device == "dualsense" else pi["rt_dev"]), True
            elif env == "pi":
                b, front = S["jacks"].get(pt.jack), True
            else:
                b, front = pi["hub_in"][DEV_HUB[pt.device]], False
            if b is not None:
                hot = self.sel in ("src_" + pt.device, "mod_" + mod)
                out.append((a, b, module_color(mod), hot, front))
        return out

    def _back_cables(self, p, S):
        for a, b, col, hot, front in self._cable_ends(S):
            if not front:
                p.strokePath(_curve(a, b), D.pen(_op(col, 0.85 if hot else 0.28), 2.2 if hot else 1.3))
        pi = S["pi"]
        outs = pi["out"] if pi["mode"] == "fold" else pi["hub_out"]
        for k, target in (("tt", "i2c_in"), ("ssp", "ssp_in")):
            a, b = outs[k], S[target]
            col = module_color("lfo" if k == "tt" else "ssp")
            hot = self.sel == ("mod_lfo" if k == "tt" else "mod_ssp")
            p.strokePath(_curve(a, b, 0.5), D.pen(_op(col, 0.95 if hot else 0.6), 4.0 if hot else 3.2))
        # I2C: i2c2midi → ER-301 and Teletype
        col = module_color("lfo")
        bus = S["i2c_bus"]
        for end in (S["er_i2c"], S["tt_i2c"]):
            p.strokePath(_vcurve(bus, end), D.pen(_op(col, 0.6), 2.4))

    def _front_cables(self, p, S):
        pi, used = S["pi"], S["used"]
        for a, b, col, hot, front in self._cable_ends(S):
            if front:
                p.strokePath(_curve(a, b), D.pen(_op(col, 0.9 if hot else 0.35), 2.4 if hot else 1.5))
        if pi["mode"] != "fold":
            ao = pi["A_out"]
            if pi["mode"] == "scripts":
                # routers → the jacks of the functions; plain CC straight to the script's MIDI out
                fn = pi["rt_fn"]
                for (mk, pk), end in S["jacks"].items():
                    if ENV_OF.get(mk) == "pi" and (mk, pk) in used:
                        col = module_color(mk)
                        hot = self.sel == "mod_" + mk
                        p.strokePath(_short(fn, end, 0.6), D.pen(_op(col, 0.9 if hot else 0.5), 2.0 if hot else 1.4))
                cc = pi["rt_cc"]
                cc_on = any(pt.device == "dualsense" and ENV_OF.get(pt.jack[0]) != "pi" for pt in S["pts"])
                lane = QPainterPath()
                lane.moveTo(cc)
                yb = pi["A"].bottom() - 34
                lane.cubicTo(QPointF(cc.x() + 30, cc.y()), QPointF(cc.x() + 30, yb), QPointF(cc.x() + 60, yb))
                lane.lineTo(QPointF(ao.x() - 60, yb))
                lane.cubicTo(QPointF(ao.x() - 24, yb), QPointF(ao.x() - 24, ao.y()), ao)
                p.strokePath(lane, D.pen(_op(Theme.function, 0.75 if cc_on else 0.25), 1.8 if cc_on else 1.2,
                                         None if cc_on else [3, 3]))
                # the channel file: DualSense script → hub (pedal channel)
                ch = pi["cards"]["chan"]
                st = pi["steps"]["pedal"]
                ly = pi["lane_y"]
                path = QPainterPath()
                gx = pi["rt"].right() + 14
                path.moveTo(ch.right(), ch.center().y())
                path.lineTo(gx, ch.center().y())
                path.lineTo(gx, ly)
                path.lineTo(st.left() - 22, ly)
                path.lineTo(st.left() - 22, st.top() + 9)
                path.lineTo(st.left(), st.top() + 9)
                dim = _op(Theme.function, 0.55)
                p.strokePath(path, D.pen(dim, 1.2, [2, 3]))
                _arrow(p, QPointF(st.left() + 2, st.top() + 9), False, dim)
                lab = D.Text("/tmp/dualsense_channel", Theme.font(8, SEMIBOLD, "mono"), _op(Theme.dim, 0.9))
                lab.paint(p, gx + 10, ly - lab.h - 1)
                # LED / rumble back to the DualSense
                hid = pi["cards"]["hid"]
                ds = next(r for d, _t, r, _f, _i in S["cards"] if d == "dualsense")
                a = QPointF(hid.left(), hid.top() + 18)
                b = QPointF(ds.right() + 4, ds.top() + 18)
                path = QPainterPath()
                path.moveTo(a)
                dx = max(40.0, abs(a.x() - b.x()) * 0.4)
                path.cubicTo(QPointF(a.x() - dx, a.y()), QPointF(b.x() + dx, b.y()), b)
                p.strokePath(path, D.pen(dim, 1.2, [2, 3]))
                _arrow(p, b, True, dim)
            # the functions' MIDI → the script's out port → the hub
            for k in PI_MODS:
                o = S["outs"].get(k)
                if not o:
                    continue
                jc, on, oc = o
                hot = self.sel == "mod_" + k
                p.strokePath(_short(jc, ao, 0.6), D.pen(_op(oc, 0.9 if hot else (0.55 if on else 0.2)), 2.0 if hot else 1.5,
                                                       None if on else [3, 3]))
            end = pi["hub_in"]["script"]
            p.strokePath(_short(ao, end, 0.5), D.pen(_op(Theme.function, 0.75), 2.8))
            _jack(p, ao, Theme.function, True, 13)
        # inside the i2c2midi and the SSP: from where the USB MIDI arrives to the inputs that listen
        for env, mod, fold in (("i2c_in", "lfo", "i2c"), ("ssp_in", "ssp", "ssp")):
            a = S[env]
            col = module_color(mod)
            hot = self.sel == "mod_" + mod
            if not self.folded(fold):
                for (mk, pk), end in S["jacks"].items():
                    if mk == mod and (mk, pk) in used:
                        path = QPainterPath()
                        path.moveTo(a)
                        path.cubicTo(QPointF(a.x() + 6, a.y()), QPointF(end.x() - 14, end.y()), end)
                        p.strokePath(path, D.pen(_op(col, 0.9 if hot else 0.55), 1.8 if hot else 1.4))
            _jack(p, a, col, True, 13)
        col = module_color("lfo")
        for c in (S["i2c_bus"], S["er_i2c"], S["tt_i2c"]):
            _jack(p, c, col, True, 11)
        bus = S["i2c_bus"]
        D.Text("I2C", Theme.font(9, HEAVY, "mono"), _op(col, 0.9), tracking=1.4).paint(p, bus.x() + 10, bus.y() + 4)

    # ── fold buttons, the scripts switch ──
    def _chrome(self, p, S):
        g = S["g"]

        def chev(c: QPointF, key: str, what: str):
            open_ = not self.folded(key)
            D.circle(p, c, 20, Theme.bg, Theme.c("body", 0.35), 1)
            _chevron(p, c, open_, Theme.ink if open_ else Theme.dim)
            self.hits.add(QRectF(c.x() - 13, c.y() - 13, 26, 26), lambda: self.toggle_fold(key),
                          ("Fold " if open_ else "Unfold ") + what)

        for dev, title, r, fold, _items in S["cards"]:
            chev(QPointF(r.right() - 24, r.top() + 18), "src_" + dev, title.title())
        pr = g["pi"]
        chev(QPointF(pr.right() - 22, pr.top() + 17), "pi", "the Raspberry Pi")
        if S["pi"]["mode"] != "fold":
            on = self.view["scripts"]
            col = Theme.function
            chip = D.Pill(D.Text("SCRIPTS", Theme.font(9, HEAVY, "mono"), Theme.bg if on else col, tracking=1.2), 9, 3,
                          col if on else _op(col, 0.08), col, 1.2)
            x = pr.right() - 44 - chip.w
            y = pr.top() + 17 - chip.h / 2
            chip.paint(p, x, y)
            self.hits.add(QRectF(x, y, chip.w, chip.h), lambda: self.set_scripts(not self.view["scripts"]),
                          "Hide how the scripts work inside" if on else "Show how the two scripts on the Pi work together")
        for key in ("i2c", "ssp"):
            r = g[key]
            chev(QPointF(r.right() - 22, r.top() + 17), key, GROUP_NAME[key])
        for key in ("er", "tt"):
            r = g[key]
            chev(QPointF(r.right() - 20, r.top() + 18), key, GROUP_NAME[key])


SCRIPT_SUB = {"slice": "grain slice → NRPN 3 · Create/Options pick ch",
              "loop": "records the DualSense + pedals, plays back",
              "extras": "motion NRPN · strum CC · freezes the sticks"}


# ── inspector ──────────────────────────────────────────────────────────────────────────────────

MODULE_HINT = {
    "slice": "Runs in the DualSense script on the Pi. The grain slice 0–1023 = block × 128 + section × 16 + slice, "
             "sent as one 14-bit value (NRPN 3 on the DualSense channel).",
    "loop": "Runs in the DualSense script on the Pi. Records what the DualSense and the pedals send on the current MIDI "
            "channel and plays it back into the MIDI hub; quantize follows the NerdSEQ clock (or 120 BPM).",
    "lfo": "A trigger LFO inside the i2c2midi: it fires pulses at a rate set by one control (0 = stopped). "
           "Rate and reset have to arrive on the same MIDI channel.",
    "extras": "Switches of the DualSense script: motion control (tilt / twist) on / off, harmonic strum, freezing the sticks.",
}
DEVICE_HINT = {
    "dualsense": "The DualSense's controls are assigned in the DualSense tab: click a control there and pick a function.",
    "16n": "The DualSense script on the Pi listens to the 16n's USB MIDI next to the MIDI hub, so the faders keep "
           "going to the i2c2midi as well. Needs the 16n on the SSH host.",
    "pedal": "The DualSense script on the Pi already reads the DOREMiDi adapter (for loop recording); patched messages "
             "also run a function. Channel as the pedal sends it (before the hub moves channel 1).",
    "ec4": "The DualSense script on the Pi listens to the EC4 next to the MIDI hub. Needs the EC4 on the SSH host. "
           "The list shows the setup and group picked in the EC4 tab.",
}


def build_panel(ctx, tab: str, col: QVBoxLayout):
    sel = ctx.pi_sel
    if sel.startswith("src_"):
        device_panel(ctx, sel[4:], col)
    elif sel == "mod_ssp":
        ssp_panel(ctx, col)
    elif sel == "env_pi":
        pi_panel(ctx, col)
    else:
        module_panel(ctx, sel[4:] if sel.startswith("mod_") else "slice", col)
    if not ctx.mapping.controls_on_pi:
        col.addWidget(W.hint("⚠︎ The SSH host doesn't have the button functions installed yet: only the ER-301 routing is uploaded."))


def _patch_row(ctx, pt: Patch, show_func: bool) -> QWidget:
    m = ctx.mapping
    dev = "DualSense" if pt.device == "dualsense" else M.PI_DEVICE_TITLE[pt.device]
    text = f"{dev} · {pt.label}"
    if pt.ref[0] == "midi" and not pt.label.startswith("ch "):
        text += f"  ({raw_label(pt.ref[2], pt.ref[3], pt.ref[4])})"
    lb = W.label(text, 12, REGULAR, "ui", Theme.ink, wrap=True)
    parts = [lb]
    if show_func:
        parts.append(W.label(token(pt.func), 10, BOLD, "mono", module_color(pt.jack[0])))
    parts.append("stretch")
    if pt.ref[0] == "ssp":
        def go_ssp():
            ctx.select("pi_sel", "mod_ssp")
        parts.append(W.icon_button("→", go_ssp, "Show the SSP patch"))
    elif pt.ref[0] == "midi":
        _, d, c, k, n = pt.ref

        def remove(d=d, c=c, k=k, n=n):
            m.drop_pi(d, c, k, n)
            ctx.changed(True)

        parts.append(W.icon_button("✕", remove, "Unpatch"))
    else:
        def go():
            ctx.set_mode("dualsense")
            if pt.ref[0] == "input":
                ctx.select_ds(M.group_of(M.INPUT[pt.ref[1]]).id)
            else:
                ctx.tabs["dualsense"][2].set_tab("Combos")
        parts.append(W.icon_button("→", go, "Edit in the DualSense tab"))
    return W.hbox(*parts)


def module_panel(ctx, key: str, col: QVBoxLayout):
    m = ctx.mapping
    _k, title, sub, colname, ports = next(x for x in MODULES if x[0] == key)
    mc = Theme.c(colname)
    col.addWidget(W.title(MOD_NAME[key]))
    col.addWidget(W.hint(MODULE_HINT[key]))
    pts = patches(ctx)
    card = W.Card("Inputs", mc)
    for pk, plabel in ports:
        mine = [pt for pt in pts if pt.jack == (key, pk)]
        card.add(W.label(plabel, 11, HEAVY, "mono", mc if mine else Theme.dim))
        if not mine:
            card.add(W.label("—", 12, REGULAR, "ui", Theme.dim))
        for pt in mine:
            card.add(_patch_row(ctx, pt, len(FUNCS_OF[(key, pk)]) > 1))
    col.addWidget(card)
    col.addWidget(add_card(ctx, [f for (mk, _pk), fs in FUNCS_OF.items() if mk == key for f in fs], mc))
    lp = m.lfo_problem()
    if key == "lfo" and lp:
        col.addWidget(W.label("⚠︎ " + lp, 11, REGULAR, "ui", Theme.trig, wrap=True))
    if key == "slice":
        from .dualsense import function_picker
        from ..store import NotesStore
        i = M.GROUP["slice"].inputs[0]
        out = W.Card("Output", Theme.cv)
        out.add(W.row("Send to", function_picker(ctx, i), SEMIBOLD))
        if m.effective_role(i) == M.CV:
            def set_port(v):
                m.ports[i.id] = v
                ctx.changed()
            out.add(W.row("SC.CV port", W.NumberField(m.port(i), 1, 99, set_port)))
        out.add(W.hint("0–16383 → 0–10 V in full resolution."))
        col.addWidget(out)
        key_n = NotesStore.ds(i.id)
        col.addWidget(W.NoteField(ctx.notes.raw(key_n), lambda t: ctx.set_note(key_n, t)))
    elif key == "lfo":
        from .er301 import lfo_card
        col.addWidget(lfo_card(ctx))


def add_card(ctx, funcs: List[str], accent: QColor, device: Optional[str] = None) -> W.Card:
    """Patch a control of the 16n / pedals / EC4 to one of `funcs`."""
    m = ctx.mapping
    card = W.Card("Patch a controller", accent)
    state = ctx.pi_add
    if device is not None:
        state["device"] = device
    dev = state.get("device", "16n")
    if device is None:
        card.add(W.Segmented(M.PI_DEVICES, dev, lambda v: (state.__setitem__("device", v), state.pop("control", None),
                                                          ctx.inspector.rebuild())))
    ctrls = controls_of(ctx, dev)
    keys = [f"{c}:{k}:{n}" for _l, c, k, n in ctrls]
    cur = state.get("control") if state.get("control") in keys + ["manual"] else (keys[0] if keys else "manual")
    state["control"] = cur
    man = state.setdefault("manual", {"ch": 1, "kind": "cc", "num": 0})
    if cur == "manual":
        ch, kind, num = man["ch"], man["kind"], man["num"]
    else:
        ch, kind, num = int(cur.split(":")[0]), cur.split(":")[1], int(cur.split(":")[2])

    def pick_ctl(v):
        state["control"] = v
        ctx.inspector.rebuild()

    opts = []
    for (label, c, k, n), key in zip(ctrls, keys):
        f = m.pi_fn(dev, c, k, n)
        opts.append((key, f"{label} · {raw_label(c, k, n)}" + ("" if f == "none" else f"  → {token(f)}")))
    opts.append(("manual", "Other: channel + number …"))
    card.add(W.row("Control", W.Picker(opts, cur, pick_ctl, 230), SEMIBOLD))
    loaded = {"16n": ctx.bank.loaded, "pedal": ctx.pedals.loaded, "ec4": ctx.ec4.loaded}[dev]
    if not loaded:
        card.add(W.hint({"16n": "The 16n isn't loaded (16n tab → Load): the list shows its default CCs.",
                         "pedal": "The pedals aren't loaded (Pedals tab → Load), so their messages aren't listed.",
                         "ec4": "No EC4 setups received yet (EC4 tab → Receive)."}[dev]))
    if cur == "manual":
        def setm(k, v, rebuild=False):
            man[k] = v
            if rebuild:
                ctx.inspector.rebuild()
        card.add(W.row("Channel", W.NumberField(man["ch"], 1, 16, lambda v: setm("ch", v))))
        card.add(W.row("Message", W.Segmented([("cc", "CC"), ("note", "Note")], man["kind"], lambda v: setm("kind", v, True))))
        card.add(W.row("Number", W.NumberField(man["num"], 0, 127, lambda v: setm("num", v))))
    allowed = [f for f in funcs if f in m.pi_choices(kind)]
    if not allowed:
        card.add(W.hint("A note can only fire an action."))
        return card
    fcur = state.get("func") if state.get("func") in allowed else allowed[0]
    state["func"] = fcur
    card.add(W.row("Function", W.Picker([(f, func_title(f)) for f in allowed], fcur,
                                        lambda v: state.__setitem__("func", v), 230), SEMIBOLD))

    def patch():
        c, k, n = (man["ch"], man["kind"], man["num"]) if state["control"] == "manual" else (ch, kind, num)
        m.set_pi_fn(dev, c, k, n, state["func"])
        ctx.changed(True)

    b = QPushButton("Patch")
    b.setObjectName("small")
    b.setCursor(Qt.PointingHandCursor)
    b.clicked.connect(patch)
    was = "none" if cur == "manual" else m.pi_fn(dev, ch, kind, num)
    card.add(W.hbox(b, W.label("" if was == "none" else f"replaces {token(was)}", 11, REGULAR, "ui", Theme.dim), stretch_end=True))
    if kind == "cc" and fcur in M.INPUT_ACTIONS:
        card.add(W.hint("Fires when the value goes to 64 or above (a switch, a push button or a fader passing the middle)."))
    return card


def device_panel(ctx, dev: str, col: QVBoxLayout):
    m = ctx.mapping
    col.addWidget(W.title("DualSense" if dev == "dualsense" else M.PI_DEVICE_TITLE[dev]))
    col.addWidget(W.hint(DEVICE_HINT[dev]))
    mine = [pt for pt in patches(ctx) if pt.device == dev]
    card = W.Card("Patched", Theme.function)
    if not mine:
        card.add(W.label("—", 12, REGULAR, "ui", Theme.dim))
    for pt in mine:
        if pt.ref[0] == "ssp":
            continue
        if pt.ref[0] != "midi":
            card.add(_patch_row(ctx, pt, True))
            continue
        _, d, c, k, n = pt.ref

        def setf(v, d=d, c=c, k=k, n=n):
            m.set_pi_fn(d, c, k, n, v)
            ctx.changed(True)

        card.add(W.label(pt.label if pt.label.startswith("ch ") else f"{pt.label}  ({raw_label(c, k, n)})",
                         12, SEMIBOLD, "ui", Theme.ink, wrap=True))
        card.add(W.hbox(W.Picker([(f, "Unpatch" if f == "none" else func_title(f)) for f in m.pi_choices(k)], pt.func, setf, 230),
                        stretch_end=True))
    col.addWidget(card)
    to_ssp = [pt for pt in mine if pt.ref[0] == "ssp"]
    if to_ssp:
        card = W.Card("Reaches the SSP patch", module_color("ssp"))
        for pt in to_ssp:
            card.add(_patch_row(ctx, pt, True))
        col.addWidget(card)
    if dev == "dualsense":
        b = W.small_button("Open the DualSense tab", lambda: ctx.set_mode("dualsense"))
        col.addWidget(W.hbox(b, stretch_end=True))
    else:
        col.addWidget(add_card(ctx, [f for f in m.pi_choices("cc") if f != "none"], Theme.function, dev))


# ── the Raspberry Pi ───────────────────────────────────────────────────────────────────────────

PI_SCRIPT_TEXT = [
    "Reads the DualSense (buttons, sticks, triggers, motion, touchpad) and, next to the MIDI hub, the 16n, the EC4 "
    "and the pedals.",
    "Its routers look up in config/controls.py what each control does: a Pi function (Slice-Selekteur, "
    "Loop-Rekordeur, the DualSense extras) or plain MIDI.",
    "Everything it sends leaves through its own MIDI port, DualSense_Controller, which the hub picks up: the controls "
    "set to MIDI, the grain slice (NRPN 3), loop playback, motion and strum. That is the hub's DUALSENSE SCRIPT input.",
    "Channels 1–3 (Create / Options) each keep their own freeze and loop; the current one is written to "
    "/tmp/dualsense_channel. Loop quantize follows the NerdSEQ clock, else 120 BPM. LED colour, player dots and "
    "rumble go back to the controller.",
]
PI_HUB_TEXT = [
    "Connects the MIDI devices on the Pi: what comes from DualSense_Controller, the 16n, the pedals, the EC4 and "
    "the NerdSEQ goes to the XIAO-TT (→ i2c2midi), the XIAO-SSP (→ SSP) and the NerdSEQ, never back to where it "
    "came from. The SSP's own MIDI out isn't routed.",
    "On the way: pedals on channel 1 move to the DualSense's channel; 16n values for the SSP are rescaled; CC "
    "floods to the XIAO-TT are thinned out; the ER-301 routes (config/er301_routes.py) are sent to the i2c2midi "
    "when they change; an OSC tap passes mapped MIDI to OpenMW on the Mac.",
]


def pi_panel(ctx, col: QVBoxLayout):
    col.addWidget(W.title("Raspberry Pi"))
    col.addWidget(W.hint("Two services run on the Pi. Kontrolleur writes their settings over SSH; both re-read "
                         "them while running, so nothing needs a restart."))
    tab = getattr(ctx, "tabs", {}).get("pi")
    if tab is None:                     # still starting up: the drawing isn't there yet
        from PySide6.QtCore import QTimer
        QTimer.singleShot(0, lambda: ctx.tabs.get("pi") and ctx.tabs["pi"][2].rebuild())
    else:
        _view_card(tab[3], col)
    for title_text, lines in (("DualSense script · dualsense_midi.py", PI_SCRIPT_TEXT),
                              ("MIDI hub · midi_passthrough.py", PI_HUB_TEXT)):
        card = W.Card(title_text, Theme.function)
        for line in lines:
            card.add(W.label(line, 12, REGULAR, "ui", Theme.ink, wrap=True))
        col.addWidget(card)


def _view_card(d, col: QVBoxLayout):
    card = W.Card("View", Theme.function)
    mode = "fold" if d.folded("pi") else ("scripts" if d.view["scripts"] else "normal")

    def pick(v):
        if v == "fold":
            if not d.folded("pi"):
                d.toggle_fold("pi")
        else:
            if d.folded("pi"):
                d.toggle_fold("pi")
            d.set_scripts(v == "scripts")

    card.add(W.Segmented([("fold", "Folded"), ("normal", "Functions"), ("scripts", "Scripts")], mode, pick))
    card.add(W.hint("Drag a box to move it, ▾ folds it. Fold all and Reset layout are in the corner of the drawing."))
    col.addWidget(card)


# ── the SSP ────────────────────────────────────────────────────────────────────────────────────

def ssp_panel(ctx, col: QVBoxLayout):
    st = ctx.ssp
    sc = module_color("ssp")
    col.addWidget(W.title("Percussa SSP"))
    col.addWidget(W.hint("Everything the MIDI hub gets from the DualSense, 16n, pedals and EC4 also goes to the SSP. "
                         "This shows which of it the SSP's patch listens to: MIDI learn on its modules, MIDI-to-CV "
                         "modules and the built-in MIDI module. Read over ssh from the patch files on the SSP's card."))
    card = W.Card("Patch", sc)
    if st.error:
        card.add(W.label("⚠︎ " + st.error, 11, REGULAR, "ui", Theme.trig, wrap=True))
    if st.loaded:
        cur = st.current
        loaded = next((p for p in st.presets if p["file"] == cur.get("file")), None)
        follow = "Follow the SSP" + (f" (now {S.label(loaded)})" if loaded else " (loaded patch unknown)")
        opts = [("", follow)] + [(p["file"], S.label(p)) for p in st.presets]

        def pick(v):
            st.pick = v
            ctx.s.setValue("sspPreset", v)
            ctx.refresh_all()

        card.add(W.row("Show", W.Picker(opts, st.pick if st.pick in {p["file"] for p in st.presets} else "", pick, 240), SEMIBOLD))
        if not cur.get("file") and not st.pick:
            card.add(W.hint("Which patch is loaded is read from the SSP's log, written from SYNTHOR's next start on. "
                            "Restart SYNTHOR once, or pick a patch here." if cur.get("logging") else
                            "The SSP doesn't log which patch it loads (its startup script sends SYNTHOR's output nowhere), "
                            "so pick a patch here."))
        where = f"via {st.host}" if st.host and not st.stale else "from the last connection"
        card.add(W.hint(f"{len(st.presets)} patches {where}."))
    elif not st.loading:
        card.add(W.hint("Not loaded yet."))
    card.add(W.hbox(W.small_button("Connecting …" if st.loading else "Reload from the SSP", lambda: ctx.load_ssp(), not st.loading),
                    stretch_end=True))
    col.addWidget(card)

    if st.loaded and not st.stale and not st.devices():
        col.addWidget(W.label("⚠︎ No MIDI device is plugged into the SSP right now, so nothing from the MIDI hub "
                              "reaches it (the XIAO bridge from the Pi isn't connected).", 11, REGULAR, "ui", Theme.trig, wrap=True))
    p = st.active
    mods = S.midi_modules(p)
    if p and not mods:
        col.addWidget(W.hint(f"{S.label(p)} doesn't listen to MIDI: no MIDI learn, no MIDI modules."))
    snd = S.senders(ctx) if mods else {}
    for m in mods:
        card = W.Card(S.module_title(m), sc)
        if m["code"] != "MIDI":
            text, ok = st.device_status(m.get("midi_in"), p)
            card.add(W.label(("listens to " if ok else "⚠︎ listens to ") + text, 11, REGULAR, "ui",
                             Theme.dim if ok else Theme.trig, wrap=True))
        else:
            chs = m.get("channels") or []
            card.add(W.label("channels " + (", ".join(map(str, chs)) if len(chs) < 16 else "all"), 11, REGULAR, "ui", Theme.dim))
        for i in m["inputs"]:
            who = S.matches(snd, i)
            src = ", ".join(f"{'DualSense' if d == 'dualsense' else M.PI_DEVICE_TITLE[d]} {lab}" for d, lab in who[:3])
            if len(who) > 3:
                src += f" +{len(who) - 3}"
            card.add(W.hbox(W.label(S.input_text(i), 11, BOLD, "mono", sc if who else Theme.dim),
                            W.label("→ " + S.target_text(m, i), 11, REGULAR, "ui", Theme.ink), "stretch"))
            card.add(W.label("← " + src if who else "← nothing sends this", 11, REGULAR, "ui",
                             Theme.ink if who else Theme.dim, wrap=True))
        col.addWidget(card)
    if mods:
        col.addWidget(W.hint("Parameter numbers as the SSP counts them inside each module. The DualSense sends on "
                             "channels 1–3 (Create / Options switch them)."))
    card = W.Card("Connection")

    def seth(k, v):
        ctx.s.setValue(k, v)

    for title_text, key, default, ph in (("SSP host(s)", "sspHosts", S.DEFAULT_HOSTS, "ssp-eth ssp …"),
                                         ("SSH key", "sspIdentity", "", "~/.ssh/… (empty: ~/.ssh/config)")):
        e = QLineEdit(str(ctx.get(key, default)))
        e.setPlaceholderText(ph)
        e.textEdited.connect(lambda t, k=key: seth(k, t))
        card.add(W.row(title_text, e))
    card.add(W.hint("Tried in order, as root. Names from ~/.ssh/config work; ssp-wifi = ssp.local, which the SSP answers on any Wi-Fi."))
    col.addWidget(card)
