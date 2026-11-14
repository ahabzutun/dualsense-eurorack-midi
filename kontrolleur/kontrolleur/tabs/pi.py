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
MOD_X, MOD_W = 560.0, 270.0
OUT_X, OUT_W = 1000.0, 230.0
WIDTH = 1260.0


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

    def _mod_heights(self):
        rh = self._row_h() + 2
        return [44 + rh * max(1, len(ports)) + 8 for *_x, ports in modules(self.ctx)]

    @property
    def base(self):
        by_dev = {}
        for pt in patches(self.ctx):
            by_dev.setdefault(pt.device, []).append(pt)
        src = sum(self._src_heights(by_dev)) + 14 * (len(SOURCES) - 1)
        mod = sum(self._mod_heights()) + 22 * (len(modules(self.ctx)) - 1)
        return (WIDTH, max(640.0, src + 60, mod + 60))

    def paint(self, p: QPainter, w, h):
        self._src_jacks, self._jacks, self._outs = [], {}, {}
        pts = patches(self.ctx)
        by_dev = {}
        for pt in pts:
            by_dev.setdefault(pt.device, []).append(pt)
        self._heading(p)
        self._sources(p, by_dev, h)
        self._modules(p, pts, h)
        self._outputs(p)
        self._cables(p)

    def _heading(self, p):
        f = Theme.font(10, HEAVY, "mono")
        for x, t in ((SRC_X, "CONTROLLERS"), (MOD_X, "PI FUNCTIONS · SSP"), (OUT_X, "OUT")):
            D.Text(t, f, Theme.dim, tracking=1.4).paint(p, x, 6)

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

    # ── middle: the modules ──
    def _modules(self, p, pts, h):
        heights = self._mod_heights()
        mods = modules(self.ctx)
        total = sum(heights) + 22 * (len(mods) - 1)
        y = max(30.0, (h - total) / 2)
        rh = self._row_h() + 2
        used_jacks = {pt.jack for pt in pts}
        for (key, title, sub, colname, ports), mh in zip(mods, heights):
            sel = self.sel == "mod_" + key
            col = Theme.c(colname)
            used = any((key, pk) in used_jacks for pk, _ in ports)
            r = QRectF(MOD_X, y, MOD_W, mh)
            path = QPainterPath()
            path.addRoundedRect(r, 10, 10)
            if sel:
                D.glow(p, path, _op(col, 0.7), 10)
            D.draw_path(p, path, _op(col, 0.07) if used else Theme.c("body", 0.035),
                        Theme.ink if sel else (_op(col, 0.75) if used else Theme.c("body", 0.4)), 2 if sel else 1.3)
            t = D.Text(title, Theme.font(11, HEAVY, "mono"), col if used or sel else Theme.dim, tracking=1.2)
            t.paint(p, r.left() + 16, r.top() + 9)
            D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=MOD_W - 32, min_scale=0.7).paint(p, r.left() + 16, r.top() + 11 + t.h)
            ry = r.top() + 44
            for pk, plabel in ports:
                on = (key, pk) in used_jacks
                cy = ry + rh / 2
                jc = QPointF(r.left(), cy)
                D.circle(p, jc, 13, col if on else Theme.bg, col if on else Theme.c("body", 0.5), 1.4)
                D.circle(p, jc, 5, Theme.bg if on else None, None if on else Theme.c("body", 0.4), 1)
                self._jacks[(key, pk)] = jc
                lt = D.Text(plabel, Theme.font(10, HEAVY, "mono"), col if on else Theme.dim)
                lt.paint(p, r.left() + 18, cy - lt.h / 2)
                ry += rh
            text, on, oc = module_output(self.ctx, key)
            if text:
                oy = r.top() + 44 + rh / 2
                jc = QPointF(r.right(), oy)
                D.circle(p, jc, 13, oc if on else Theme.bg, oc if on else Theme.c("body", 0.5), 1.4)
                D.circle(p, jc, 5, Theme.bg if on else None, None if on else Theme.c("body", 0.4), 1)
                lt = D.Text("OUT", Theme.font(10, HEAVY, "mono"), Theme.dim)
                lt.paint(p, r.right() - 22 - lt.w, oy - lt.h / 2)
                self._outs[key] = (jc, text, on, oc)
            self.hits.add(path, lambda key=key: self.ctx.select("pi_sel", "mod_" + key), MOD_NAME[key])
            y += mh + 22

    # ── right: where the results go ──
    def _outputs(self, p):
        for key, (jc, text, on, oc) in self._outs.items():
            head = "MIDI HUB" if key == "loop" else "ER-301"
            sub = "loop playback" if key == "loop" else ("grain slice" if key == "slice" else "LFO pulses")
            t1 = D.Text(head, Theme.font(10, HEAVY, "mono"), Theme.ink if on else Theme.dim, tracking=1.2)
            t2 = D.Text(text, Theme.font(11, BOLD, "mono"), oc if on else Theme.dim)
            t3 = D.Text(sub, Theme.font(9, SEMIBOLD, "mono"), Theme.dim)
            bh = t1.h + t2.h + t3.h + 18
            r = QRectF(OUT_X, jc.y() - bh / 2, OUT_W, bh)
            D.rounded(p, r, 9, _op(oc, 0.08) if on else Theme.c("body", 0.035), _op(oc, 0.6) if on else Theme.c("body", 0.4), 1.2)
            t1.paint(p, r.left() + 22, r.top() + 7)
            t2.paint(p, r.left() + 22, r.top() + 9 + t1.h)
            t3.paint(p, r.left() + 22, r.top() + 11 + t1.h + t2.h)
            end = QPointF(r.left(), jc.y())
            D.circle(p, end, 9, oc if on else Theme.bg, oc if on else Theme.c("body", 0.5), 1.2)
            if on:
                path = QPainterPath()
                path.moveTo(jc)
                path.cubicTo(QPointF(jc.x() + 70, jc.y()), QPointF(end.x() - 70, end.y()), end)
                hot = self.sel == "mod_" + key
                p.strokePath(path, D.pen(_op(oc, 0.9 if hot else 0.5), 2.4 if hot else 1.8))
            self.hits.add(QRectF(r), lambda key=key: self.ctx.select("pi_sel", "mod_" + key))

    def _cables(self, p):
        for a, pt in self._src_jacks:
            end = self._jacks.get(pt.jack)
            if end is None:
                continue
            col = module_color(pt.jack[0])
            hot = self.sel in ("src_" + pt.device, "mod_" + pt.jack[0])
            path = QPainterPath()
            path.moveTo(a)
            dx = max(60.0, (end.x() - a.x()) * 0.45)
            path.cubicTo(QPointF(a.x() + dx, a.y()), QPointF(end.x() - dx, end.y()), end)
            p.strokePath(path, D.pen(_op(col, 0.9 if hot else 0.35), 2.4 if hot else 1.5))


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

    p = st.active
    mods = S.midi_modules(p)
    if p and not mods:
        col.addWidget(W.hint(f"{S.label(p)} doesn't listen to MIDI: no MIDI learn, no MIDI modules."))
    snd = S.senders(ctx) if mods else {}
    for m in mods:
        card = W.Card(S.module_title(m), sc)
        if m["code"] != "MIDI":
            text, ok = st.device_status(m.get("midi_in"))
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
