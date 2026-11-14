"""Routing tab: the functions that run on the Pi (and the i2c2midi's LFO) as modules, patched from any
controller: DualSense, 16n, pedals (MPC-20) or EC4 on the left, modules in the middle, where their
results go on the right."""
from __future__ import annotations

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
    return "", False, Theme.dim


# ── drawing ────────────────────────────────────────────────────────────────────────────────────
# Environments, left to right: the controllers · the Raspberry Pi (Pi functions + MIDI hub) · the
# i2c2midi (Teletype expander, with its LFO) on the I2C bus with the ER-301 and the Teletype · the SSP.
# MIDI for the i2c2midi and the SSP leaves the Pi through the hub (XIAO-TT / XIAO-SSP).

PI_X, PI_W = 404.0, 616.0
HUB_X, HUB_W = 790.0, 210.0
R_X, R_W = 1090.0, 420.0
WIDTH = 1530.0
ENV_HEAD = 34.0
PI_MODS = ("slice", "loop", "extras")
ENV_OF = {"slice": "pi", "loop": "pi", "extras": "pi", "lfo": "i2c", "ssp": "ssp"}
HUB_IN = [("dualsense", "DUALSENSE"), ("16n", "16N"), ("pedal", "PEDALS"), ("ec4", "EC4"), ("script", "PI SCRIPT")]
HUB_OUT = [("tt", "XIAO-TT", "→ i2c2midi"), ("ssp", "XIAO-SSP", "→ SSP")]


def _opaque(p, path, tint: QColor, alpha: float):
    p.fillPath(path, Theme.bg)
    p.fillPath(path, _op(tint, alpha))


def _curve(a: QPointF, b: QPointF, bend: float = 0.45) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(a)
    dx = max(50.0, abs(b.x() - a.x()) * bend)
    path.cubicTo(QPointF(a.x() + dx, a.y()), QPointF(b.x() - dx, b.y()), b)
    return path


def _jack(p, c: QPointF, col: QColor, on: bool, d: float = 13):
    D.circle(p, c, d, col if on else Theme.bg, col if on else Theme.c("body", 0.5), 1.4)
    D.circle(p, c, d * 0.38, Theme.bg if on else None, None if on else Theme.c("body", 0.4), 1)


class PiDrawing(Drawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self._src_jacks = []           # (point, patch)
        self._jacks = {}               # (module, jack) -> point

    @property
    def sel(self) -> str:
        return self.ctx.pi_sel

    def _row_h(self) -> float:
        return max(22.0, Theme.font(10.5, SEMIBOLD).pixelSize() * 1.25 + 8)

    def _src_heights(self, by_dev):
        rh = self._row_h()
        return [44 * Theme.grow ** 0.5 + rh * max(1, len(by_dev.get(d, []))) + 8 for d, _ in SOURCES]

    def _mod_h(self, ports) -> float:
        return 44 + (self._row_h() + 2) * max(1, len(ports)) + 8

    def _layout(self, h: Optional[float] = None) -> dict:
        """Heights of the columns; positions when h is given."""
        ctx = self.ctx
        mods = {m[0]: m for m in modules(ctx)}
        rh = self._row_h()
        L = {"mods": mods}
        pi_mods = sum(self._mod_h(mods[k][4]) for k in PI_MODS) + 22 * (len(PI_MODS) - 1)
        hub_h = 44 + (rh + 2) * len(HUB_IN) + 30 + (rh + 2) * len(HUB_OUT) + 10
        L["pi_h"] = ENV_HEAD + max(pi_mods, hub_h) + 18
        L["hub_h"] = hub_h
        L["i2c_h"] = ENV_HEAD + self._mod_h(mods["lfo"][4]) + 18
        L["er_rows"] = self._er_rows()
        L["er_h"] = 40 + rh * len(L["er_rows"]) + 10
        L["bus_h"] = 24 + max(L["er_h"], 78)
        L["ssp_h"] = ENV_HEAD + self._mod_h(mods["ssp"][4]) + 18
        L["right_h"] = L["i2c_h"] + L["bus_h"] + 30 + L["ssp_h"]
        by_dev = {}
        for pt in patches(ctx):
            by_dev.setdefault(pt.device, []).append(pt)
        L["by_dev"] = by_dev
        L["src_h"] = sum(self._src_heights(by_dev)) + 14 * (len(SOURCES) - 1)
        if h is not None:
            L["pi_y"] = max(30.0, (h - L["pi_h"]) / 2)
            L["right_y"] = max(30.0, (h - L["right_h"]) / 2)
        return L

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
        L = self._layout()
        return (WIDTH, max(640.0, L["src_h"] + 60, L["pi_h"] + 60, L["right_h"] + 60))

    # ── painting ──
    def paint(self, p: QPainter, w, h):
        self._src_jacks, self._jacks = [], {}
        L = self._layout(h)
        pts = patches(self.ctx)
        used = {pt.jack for pt in pts}
        D.Text("CONTROLLERS", Theme.font(10, HEAVY, "mono"), Theme.dim, tracking=1.4).paint(p, SRC_X, 6)
        envs = self._environments(p, L)
        self._sources(p, L["by_dev"], h)
        hub = self._hub_geometry(L)
        self._via_hub_cables(p, pts, hub, envs, used)
        # boxes (opaque: the cables to the hub run behind them)
        y = L["pi_y"] + ENV_HEAD + max(0.0, (L["pi_h"] - ENV_HEAD - 18 - (sum(self._mod_h(L["mods"][k][4]) for k in PI_MODS) + 44)) / 2)
        outs = {}
        for k in PI_MODS:
            r, out = self._module(p, L["mods"][k], PI_X + 26, y, used)
            outs[k] = out
            y += r.height() + 22
        lfo_r, lfo_out = self._module(p, L["mods"]["lfo"], R_X + 18, L["right_y"] + ENV_HEAD, used)
        ssp_top = L["right_y"] + L["i2c_h"] + L["bus_h"] + 30
        self._module(p, L["mods"]["ssp"], R_X + 18, ssp_top + ENV_HEAD, used, width=R_W - 36)
        self._hub(p, hub, pts, outs)
        self._script_cables(p, outs, hub)
        self._fan_out(p, envs, used)
        self._i2c_bus(p, L, lfo_out)
        self._pi_cables(p)

    # ── left: the controllers ──
    def _sources(self, p, by_dev, h):
        heights = self._src_heights(by_dev)
        total = sum(heights) + 14 * (len(SOURCES) - 1)
        y = max(30.0, (h - total) / 2)
        rh = self._row_h()
        for (dev, title), ch in zip(SOURCES, heights):
            key = "src_" + dev
            sel = self.sel == key
            items = by_dev.get(dev, [])
            r = QRectF(SRC_X, y, SRC_W, ch)
            path = QPainterPath()
            path.addRoundedRect(r, 10, 10)
            accent = Theme.function if items else Theme.c("body", 0.5)
            if sel:
                D.glow(p, path, _op(Theme.function, 0.6), 10)
            D.draw_path(p, path, _op(Theme.function, 0.06) if items else Theme.c("body", 0.035),
                        Theme.ink if sel else (_op(accent, 0.6) if items else Theme.c("body", 0.4)), 2 if sel else 1.2)
            t = D.Text(title, Theme.font(11, HEAVY, "mono"), Theme.ink if items or sel else Theme.dim, tracking=1.2)
            t.paint(p, r.left() + 14, r.top() + 9)
            sub = self._sub(dev)
            D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=SRC_W - 28).paint(p, r.left() + 14, r.top() + 11 + t.h)
            ry = r.top() + 44 * Theme.grow ** 0.5
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
                self._src_jacks.append((jc, pt))
                ry += rh
            self.hits.add(path, lambda key=key: self.ctx.select("pi_sel", key), title.title())
            y += ch + 14

    def _sub(self, dev: str) -> str:
        m = self.ctx.mapping
        if dev == "dualsense":
            return f"ch {m.channel} · assigned in the DualSense tab"
        n = sum(1 for x in m.midi_inputs if x.device == dev)
        return {"16n": "16n faderbank", "pedal": "MPC-20 via DOREMiDi", "ec4": "Faderfox EC4"}[dev] + \
            ("" if n or dev not in ("16n", "ec4") else " · not listened to")

    # ── environments ──
    def _env(self, p, r: QRectF, title: str, sub: str, col: QColor, key: Optional[str] = None):
        path = QPainterPath()
        path.addRoundedRect(r, 14, 14)
        p.fillPath(path, _op(col, 0.035))
        p.strokePath(path, D.pen(_op(col, 0.45), 1.3, [5, 4]))
        t = D.Text(title, Theme.font(10, HEAVY, "mono"), _op(col, 0.95), tracking=1.4)
        t.paint(p, r.left() + 14, r.top() + 8)
        D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=r.width() - t.w - 40, min_scale=0.7).paint(
            p, r.left() + 24 + t.w, r.top() + 8 + (t.h - Theme.font(9, SEMIBOLD, "mono").pixelSize() * 1.2) / 2)
        if key:
            self.hits.add(QRectF(r.left(), r.top(), r.width(), ENV_HEAD), lambda: self.ctx.select("pi_sel", key))

    def _environments(self, p, L) -> dict:
        pi = QRectF(PI_X, L["pi_y"], PI_W, L["pi_h"])
        self._env(p, pi, "RASPBERRY PI", "DualSense script · MIDI hub", Theme.function)
        y = L["right_y"]
        i2c = QRectF(R_X, y, R_W, L["i2c_h"])
        self._env(p, i2c, "I2C2MIDI", "Teletype expander · gets MIDI over USB (XIAO-TT)", module_color("lfo"), "mod_lfo")
        ssp_r = QRectF(R_X, y + L["i2c_h"] + L["bus_h"] + 30, R_W, L["ssp_h"])
        st = self.ctx.ssp
        sub = "gets MIDI over USB (XIAO-SSP)"
        if st.loaded and not st.stale and not st.devices():
            sub = "⚠ no MIDI device plugged in"
        self._env(p, ssp_r, "PERCUSSA SSP", sub, module_color("ssp"), "mod_ssp")
        return {"pi": pi, "i2c": i2c, "ssp": ssp_r,
                "i2c_in": QPointF(R_X, y + ENV_HEAD + 6), "ssp_in": QPointF(R_X, ssp_r.top() + ENV_HEAD + 6)}

    # ── modules ──
    def _module(self, p, mod, x, y, used, width=MOD_W):
        key, title, sub, colname, ports = mod
        sel = self.sel == "mod_" + key
        col = Theme.c(colname)
        on_any = any((key, pk) in used for pk, _ in ports)
        rh = self._row_h() + 2
        r = QRectF(x, y, width, self._mod_h(ports))
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        if sel:
            D.glow(p, path, _op(col, 0.7), 10)
        _opaque(p, path, col if on_any else Theme.c("body"), 0.08 if on_any else 0.035)
        p.strokePath(path, D.pen(Theme.ink if sel else (_op(col, 0.75) if on_any else Theme.c("body", 0.4)), 2 if sel else 1.3))
        if key == "ssp":
            title, sub = sub.upper() if sub.startswith("patch") else "NO PATCH", "MIDI inputs of its modules"
            title = title.replace("PATCH ", "")
        t = D.Text(title, Theme.font(11, HEAVY, "mono"), col if on_any or sel else Theme.dim, tracking=1.2, max_w=width - 32, min_scale=0.7)
        t.paint(p, r.left() + 16, r.top() + 9)
        D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=width - 32, min_scale=0.7).paint(p, r.left() + 16, r.top() + 11 + t.h)
        ry = r.top() + 44
        for pk, plabel in ports:
            on = (key, pk) in used
            cy = ry + rh / 2
            jc = QPointF(r.left(), cy)
            _jack(p, jc, col, on)
            self._jacks[(key, pk)] = jc
            lt = D.Text(plabel, Theme.font(10, HEAVY, "mono"), col if on else Theme.dim, max_w=width - 40, min_scale=0.7)
            lt.paint(p, r.left() + 18, cy - lt.h / 2)
            ry += rh
        out = None
        text, on, oc = module_output(self.ctx, key)
        if text:
            oy = r.top() + 44 + rh / 2
            jc = QPointF(r.right(), oy)
            _jack(p, jc, oc, on)
            lt = D.Text("OUT", Theme.font(10, HEAVY, "mono"), Theme.dim)
            lt.paint(p, r.right() - 22 - lt.w, oy - lt.h / 2)
            out = (jc, on, oc)
        self.hits.add(path, lambda key=key: self.ctx.select("pi_sel", "mod_" + key), MOD_NAME[key])
        return r, out

    # ── the MIDI hub ──
    def _hub_geometry(self, L) -> dict:
        rh = self._row_h() + 2
        top = L["pi_y"] + ENV_HEAD + max(0.0, (L["pi_h"] - ENV_HEAD - 18 - L["hub_h"]) / 2)
        r = QRectF(HUB_X, top, HUB_W, L["hub_h"])
        ins, outs = {}, {}
        y = top + 44
        for k, _t in HUB_IN:
            ins[k] = QPointF(r.left(), y + rh / 2)
            y += rh
        y += 30
        for k, *_ in HUB_OUT:
            outs[k] = QPointF(r.right(), y + rh / 2)
            y += rh
        return {"r": r, "in": ins, "out": outs, "out_y0": top + 44 + rh * len(HUB_IN) + 8}

    def _hub(self, p, hub, pts, pi_outs):
        r = hub["r"]
        col = Theme.function
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        _opaque(p, path, col, 0.06)
        p.strokePath(path, D.pen(_op(col, 0.7), 1.3))
        t = D.Text("MIDI HUB", Theme.font(11, HEAVY, "mono"), col, tracking=1.2)
        t.paint(p, r.left() + 16, r.top() + 9)
        D.Text("midi_passthrough on the Pi", Theme.font(9, SEMIBOLD, "mono"), Theme.dim).paint(p, r.left() + 16, r.top() + 11 + t.h)
        via = {pt.device for pt in pts if ENV_OF.get(pt.jack[0]) != "pi"}
        script_on = any(o and o[1] for o in pi_outs.values())
        for k, label in HUB_IN:
            on = k in via or (k == "script" and script_on)
            c = hub["in"][k]
            _jack(p, c, col, on, 11)
            lt = D.Text(label, Theme.font(10, HEAVY, "mono"), col if on else Theme.dim)
            lt.paint(p, c.x() + 16, c.y() - lt.h / 2)
        D.Text("OUT", Theme.font(9, HEAVY, "mono"), Theme.dim, tracking=1.2).paint(p, r.left() + 16, hub["out_y0"])
        for k, name, to in HUB_OUT:
            c = hub["out"][k]
            oc = module_color("lfo" if k == "tt" else "ssp")
            _jack(p, c, oc, True, 11)
            lt = D.Text(f"{name} {to}", Theme.font(10, HEAVY, "mono"), oc, max_w=r.width() - 40, min_scale=0.7)
            lt.paint(p, c.x() - 16 - lt.w, c.y() - lt.h / 2)

    # ── cables ──
    def _via_hub_cables(self, p, pts, hub, envs, used):
        """Controllers → hub (for the i2c2midi and the SSP), hub → the other environments."""
        for a, pt in self._src_jacks:
            env = ENV_OF.get(pt.jack[0])
            if env == "pi":
                continue
            end = hub["in"].get(pt.device)
            if end is None:
                continue
            hot = self.sel in ("src_" + pt.device, "mod_" + pt.jack[0])
            col = module_color(pt.jack[0])
            p.strokePath(_curve(a, end), D.pen(_op(col, 0.85 if hot else 0.28), 2.2 if hot else 1.3))
        for k, target in (("tt", "i2c_in"), ("ssp", "ssp_in")):
            a, b = hub["out"][k], envs[target]
            col = module_color("lfo" if k == "tt" else "ssp")
            hot = self.sel == ("mod_lfo" if k == "tt" else "mod_ssp")
            p.strokePath(_curve(a, b, 0.5), D.pen(_op(col, 0.95 if hot else 0.6), 4.0 if hot else 3.2))
            _jack(p, b, col, True, 13)

    def _fan_out(self, p, envs, used):
        """Inside the i2c2midi and the SSP: from where the USB MIDI arrives to the inputs that listen."""
        for env, mod in (("i2c_in", "lfo"), ("ssp_in", "ssp")):
            a = envs[env]
            col = module_color(mod)
            hot = self.sel == "mod_" + mod
            for (mk, pk), end in self._jacks.items():
                if mk == mod and (mk, pk) in used:
                    path = QPainterPath()
                    path.moveTo(a)
                    path.cubicTo(QPointF(a.x() + 6, a.y()), QPointF(end.x() - 14, end.y()), end)
                    p.strokePath(path, D.pen(_op(col, 0.9 if hot else 0.55), 1.8 if hot else 1.4))
            _jack(p, a, col, True, 13)

    def _script_cables(self, p, outs, hub):
        """The Pi functions' MIDI (grain slice NRPN, loop playback) goes out through the hub."""
        end = hub["in"]["script"]
        for k, out in outs.items():
            if out and out[1]:
                jc, _on, oc = out
                hot = self.sel == "mod_" + k
                path = QPainterPath()
                path.moveTo(jc)
                path.cubicTo(QPointF(jc.x() + 40, jc.y()), QPointF(end.x() - 40, end.y()), end)
                p.strokePath(path, D.pen(_op(oc, 0.9 if hot else 0.5), 2.2 if hot else 1.6))

    def _pi_cables(self, p):
        for a, pt in self._src_jacks:
            if ENV_OF.get(pt.jack[0]) != "pi":
                continue
            end = self._jacks.get(pt.jack)
            if end is None:
                continue
            col = module_color(pt.jack[0])
            hot = self.sel in ("src_" + pt.device, "mod_" + pt.jack[0])
            p.strokePath(_curve(a, end), D.pen(_op(col, 0.9 if hot else 0.35), 2.4 if hot else 1.5))

    # ── I2C: i2c2midi → ER-301, with the Teletype on the same bus ──
    def _i2c_bus(self, p, L, lfo_out):
        rh = self._row_h()
        top = L["right_y"] + L["i2c_h"]
        bus_y = top + 14
        col = module_color("lfo")
        er = QRectF(R_X, bus_y + 12, 250, L["er_h"])
        tt = QRectF(R_X + 268, bus_y + 12, R_W - 268, 78)
        p.strokePath(_line(R_X + R_W / 2, top, R_X + R_W / 2, bus_y), D.pen(_op(col, 0.6), 2.2))
        p.strokePath(_line(er.center().x(), bus_y, tt.center().x(), bus_y), D.pen(_op(col, 0.6), 2.2))
        for x in (er.center().x(), tt.center().x()):
            p.strokePath(_line(x, bus_y, x, bus_y + 12), D.pen(_op(col, 0.6), 2.2))
        D.Text("I2C", Theme.font(9, HEAVY, "mono"), _op(col, 0.9), tracking=1.4).paint(p, R_X + R_W / 2 + 8, top + 1)
        # ER-301
        path = QPainterPath()
        path.addRoundedRect(er, 10, 10)
        _opaque(p, path, Theme.cv, 0.05)
        p.strokePath(path, D.pen(_op(Theme.cv, 0.55), 1.3))
        t = D.Text("ER-301", Theme.font(11, HEAVY, "mono"), Theme.ink, tracking=1.2)
        t.paint(p, er.left() + 14, er.top() + 9)
        y = er.top() + 40
        for port, what, c, from_lfo in L["er_rows"]:
            pt_t = D.Text(port, Theme.font(10.5, BOLD, "mono"), c)
            pt_t.paint(p, er.left() + 14, y + (rh - pt_t.h) / 2)
            D.Text(what, Theme.font(9.5, SEMIBOLD, "mono"), Theme.dim, max_w=er.width() - 100, min_scale=0.7).paint(
                p, er.left() + 90, y + (rh - Theme.font(9.5, SEMIBOLD, "mono").pixelSize() * 1.2) / 2)
            y += rh
        self.hits.add(QRectF(er), lambda: self.ctx.set_mode("er301"), "ER-301 tab")
        # Teletype (shown only)
        path = QPainterPath()
        path.addRoundedRect(tt, 10, 10)
        _opaque(p, path, Theme.c("body"), 0.04)
        p.strokePath(path, D.pen(Theme.c("body", 0.45), 1.2, [4, 3]))
        t = D.Text("TELETYPE", Theme.font(11, HEAVY, "mono"), Theme.ink, tracking=1.2)
        t.paint(p, tt.left() + 14, tt.top() + 9)
        D.Text("I2C leader", Theme.font(9, SEMIBOLD, "mono"), Theme.dim).paint(p, tt.left() + 14, tt.top() + 13 + t.h)
        D.Text("shown only", Theme.font(9, SEMIBOLD, "mono"), _op(Theme.dim, 0.7)).paint(p, tt.left() + 14, tt.top() + 27 + t.h)


def _line(x1, y1, x2, y2) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(x1, y1)
    path.lineTo(x2, y2)
    return path


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
