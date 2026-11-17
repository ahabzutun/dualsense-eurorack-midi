"""Macros tab: the monome grid's left half as the Pi runs it — 8 banks × 7 slots of snapshots — with what each
slot holds, in this rig's words (which control a value came from, where it lands). Names, slew, copy / move,
taking things out of a slot, the settings and backups; ▶ loads a slot on the Pi as if pressed on the grid."""
from __future__ import annotations

import datetime as dt
import json
from pathlib import Path

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath
from PySide6.QtWidgets import QCheckBox, QFileDialog, QGridLayout, QLineEdit, QVBoxLayout, QWidget

from .. import draw as D
from .. import macros as MC
from .. import widgets as W
from ..stage import Drawing
from ..theme import BOLD, HEAVY, REGULAR, SEMIBOLD, Theme

KEY, GAP, SPLIT = 46.0, 8.0, 28.0      # grid key, gap between keys, gap between the halves
LED = QColor(255, 190, 110)            # the grid's warm white
TT_LED = QColor(140, 190, 255)


def _op(c: QColor, a: float) -> QColor:
    q = QColor(c)
    q.setAlphaF(max(0.0, min(1.0, c.alphaF() * a)))
    return q


def _mix(a: QColor, b: QColor, t: float) -> QColor:
    t = max(0.0, min(1.0, t))
    return QColor.fromRgbF(a.redF() + (b.redF() - a.redF()) * t, a.greenF() + (b.greenF() - a.greenF()) * t,
                           a.blueF() + (b.blueF() - a.blueF()) * t, 1.0)


def levels(st: MC.MacrosState, bank: int):
    """64 LED levels of the left half for a bank, as the grid shows them (macros/engine.py, without the
    moment-to-moment parts: holding save, the pulse while gliding)."""
    lv = [0] * 64
    for y in range(MC.BANKS):
        lv[y * 8] = 15 if y == bank else (5 if st.bank_used(y) else 1)
    active = st.active(bank)
    for s in range(MC.SLOTS):
        x = s + 1
        saved = st.slot(bank, s) is not None
        lv[x] = 6 if saved else 2
        step = st.slew(bank, s)
        for y in range(1, 7):
            k = 6 - y
            if k == step:
                lv[y * 8 + x] = 10 if saved else 5
            elif k < step:
                lv[y * 8 + x] = 4 if saved else 2
        lv[7 * 8 + x] = 15 if (saved and active == s) else (6 if saved else 0)
    return lv


class MacrosDrawing(Drawing):
    base = (1180.0, 760.0)

    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx

    def paint(self, p: QPainter, w, h):
        ctx = self.ctx
        st = ctx.macros
        b, sel = ctx.mac_bank, ctx.mac_slot
        cx = w / 2
        D.Text("MONOME GRID · LEFT HALF: MACROS ON THE PI · RIGHT HALF: TELETYPE", Theme.font(11, HEAVY, "mono"),
               Theme.dim, tracking=1.2).paint_center(p, cx, 34)
        D.Text(service_line(st), Theme.font(10, SEMIBOLD, "mono"),
               Theme.special if (st.service or {}).get("grid") else Theme.dim).paint_center(p, cx, 56)

        gw = 16 * KEY + 14 * GAP + SPLIT
        gh = 8 * KEY + 7 * GAP
        box = QRectF(cx - gw / 2 - 26, 84, gw + 52, gh + 52)
        path = QPainterPath()
        path.addRoundedRect(box, 18, 18)
        p.fillPath(path, D.vgradient(box, Theme.c("body", 0.12), Theme.c("body", 0.05)))
        p.strokePath(path, D.pen(Theme.c("body", 0.55), 1.4))
        x0, y0 = box.left() + 26, box.top() + 26
        lv = levels(st, b)
        tt = (st.service or {}).get("teletype")
        for y in range(8):
            for x in range(16):
                right = x >= 8
                kx = x0 + x * (KEY + GAP) + (SPLIT - GAP if right else 0)
                r = QRectF(kx, y0 + y * (KEY + GAP), KEY, KEY)
                base = _mix(Theme.bg, Theme.c("body"), 0.13)       # an unlit key, opaque
                if right:
                    D.rounded(p, r, 7, _mix(base, TT_LED, 0.05), _op(TT_LED, 0.18), 1)
                    continue
                v = lv[y * 8 + x]
                fill = _mix(base, LED, 0.08 + 0.92 * v / 15) if v else base
                selected = (x == 0 and y == b) or (x - 1 == sel and x > 0)
                D.rounded(p, r, 7, fill, Theme.ink if (selected and x > 0 and y in (0, 7)) else Theme.c("body", 0.35),
                          2 if (selected and x > 0 and y in (0, 7)) else 1)
                if x == 0:
                    self.hits.add(r, lambda y=y: ctx.select("mac_bank", y, "Bank"), f"Bank {y + 1}")
                else:
                    self.hits.add(r, lambda s=x - 1: ctx.select("mac_slot", s, "Slot"),
                                  f"Slot {b + 1}.{x}" + (f" “{st.name(b, x - 1)}”" if st.name(b, x - 1) else ""))
        # the right half belongs to Teletype
        rx = x0 + 8 * (KEY + GAP) + SPLIT - GAP
        rr = QRectF(rx, y0, 8 * KEY + 7 * GAP, gh)
        t1 = D.Text("TELETYPE", Theme.font(18, HEAVY, "mono"), _op(TT_LED, 0.85), tracking=3)
        t2 = D.Text("Grid Control mode / grid ops · passed through by the Pi", Theme.font(10, SEMIBOLD, "mono"),
                    Theme.dim, max_w=rr.width() - 30, min_scale=0.7)
        t3 = D.Text(f"gadget: {tt}" if tt else "", Theme.font(9, SEMIBOLD, "mono"), _op(Theme.dim, 0.8))
        D.Col([t1, t2, t3], 6).paint_center(p, rr.center().x(), rr.center().y())
        self.hits.add(rr, lambda: ctx.set_mode("teletype"), "Teletype's half: its scene is in the Teletype tab")
        # labels: rows on the left, columns below
        f9 = Theme.font(9, HEAVY, "mono")
        for text, yy in (("SAVE", y0 + KEY / 2), ("SLEW", y0 + 3.5 * (KEY + GAP) + KEY / 2 - (KEY + GAP) / 2),
                         ("LOAD", y0 + 7 * (KEY + GAP) + KEY / 2)):
            t = D.Text(text, f9, Theme.dim, tracking=1)
            t.paint(p, box.left() - 12 - t.w, yy - t.h / 2)
        sl = QPainterPath()
        top, bot = y0 + (KEY + GAP), y0 + 6 * (KEY + GAP) + KEY
        sl.moveTo(box.left() - 6, top)
        sl.lineTo(box.left() - 10, top)
        sl.lineTo(box.left() - 10, bot)
        sl.lineTo(box.left() - 6, bot)
        p.strokePath(sl, D.pen(Theme.c("body", 0.5), 1.2))
        for x in range(8):
            lab = "BANK" if x == 0 else str(x)
            t = D.Text(lab, f9, Theme.ink if (x - 1 == sel and x > 0) else Theme.dim, tracking=1)
            t.paint_center(p, x0 + x * (KEY + GAP) + KEY / 2, box.bottom() + 12)

        # the selected bank's slots
        ty = box.bottom() + 36
        bn = st.bank_name(b)
        bt = D.Text(f"BANK {b + 1}" + (f" · {bn.upper()}" if bn else ""), Theme.font(11, HEAVY, "mono"), Theme.ink,
                    tracking=1.2, max_w=box.width() * 0.45, min_scale=0.7)
        bt.paint(p, box.left(), ty)
        scope = st.providers_for(b)
        if set(scope) != set(MC.PROVIDER_NAMES):
            D.Text("saves / loads only: " + ", ".join(dict(MC.PROVIDERS)[x] for x in scope if x in dict(MC.PROVIDERS)),
                   Theme.font(9.5, SEMIBOLD, "mono"), Theme.trig, max_w=box.width() - bt.w - 30, min_scale=0.7).paint(
                p, box.left() + bt.w + 18, ty + 2)
        tw = (box.width() - 6 * 10) / 7
        for s in range(MC.SLOTS):
            r = QRectF(box.left() + s * (tw + 10), ty + 24, tw, 128)
            self._slot_tile(p, r, b, s, s == sel)

    def _slot_tile(self, p: QPainter, r: QRectF, b: int, s: int, selected: bool):
        ctx, st = self.ctx, self.ctx.macros
        snap = st.slot(b, s)
        active = st.active(b) == s and snap is not None
        col = Theme.function if snap is not None else Theme.c("body", 0.5)
        path = QPainterPath()
        path.addRoundedRect(r, 10, 10)
        if selected:
            D.glow(p, path, _op(col, 0.5), 10)
        D.draw_path(p, path, _op(col, 0.10 if snap is not None else 0.03),
                    Theme.ink if selected else _op(col, 0.6 if snap is not None else 0.35), 2 if selected else 1.2,
                    None if snap is not None else [4, 4])
        f = Theme.font(10, HEAVY, "mono")
        head = D.Text(f"{b + 1}.{s + 1}" + ("  ▶" if active else ""), f, Theme.ink if snap is not None else Theme.dim)
        head.paint(p, r.left() + 10, r.top() + 8)
        name = st.name(b, s) or ("—" if snap is not None else "empty")
        D.Text(name, Theme.font(12, BOLD), Theme.ink if st.name(b, s) else Theme.dim, max_w=r.width() - 20,
               min_scale=0.7).paint(p, r.left() + 10, r.top() + 26)
        D.Text(MC.fmt_secs(st.glide(b, s)), Theme.font(9.5, SEMIBOLD, "mono"), Theme.cv).paint(p, r.left() + 10, r.top() + 50)
        y = r.top() + 70
        for line in MC.summary(snap)[:3]:
            t = D.Text(line, Theme.font(9, SEMIBOLD, "mono"), Theme.dim, max_w=r.width() - 20, min_scale=0.7)
            t.paint(p, r.left() + 10, y)
            y += t.h + 2
        self.hits.add(path, lambda: ctx.select("mac_slot", s, "Slot"), "Show this slot in the panel")


def service_line(st: MC.MacrosState) -> str:
    if st.loading and not st.loaded:
        return "loading from the Pi …"
    if st.error:
        return st.error
    if not st.loaded:
        return "not loaded — ↓ Load"
    if not st.exists:
        return "no grid_macros.json on the Pi yet: install the grid macros (install-pi.sh --grid-macros)"
    sv = st.service
    if not sv:
        return f"{st.used_slots()} slots saved · the grid-macros service isn't running"
    parts = [f"{st.used_slots()} slots saved", "service running",
             "grid connected" if sv.get("grid") else "no grid", "Teletype connected" if sv.get("teletype") else "no Teletype"]
    return " · ".join(parts)


# ── inspector ───────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    st = ctx.macros
    if not st.loaded:
        col.addWidget(W.title("Macros"))
        col.addWidget(W.hint(st.error or "Not loaded yet. Press ↓ Load: Kontrolleur reads grid_macros.json from the Pi."))
        return
    if tab == "Slot":
        slot_panel(ctx, col)
    elif tab == "Bank":
        bank_panel(ctx, col)
    else:
        settings_panel(ctx, col)


def _changed(ctx, rebuild=True):
    ctx.changed(rebuild)


def _line_edit(text: str, placeholder: str, on_change, max_len=40) -> QLineEdit:
    e = QLineEdit(text)
    e.setPlaceholderText(placeholder)
    e.setMaxLength(max_len)
    e.setFont(Theme.font(13, SEMIBOLD))
    e.textEdited.connect(on_change)
    return e


def _targets(st: MC.MacrosState, here):
    out = []
    for b in range(MC.BANKS):
        for s in range(MC.SLOTS):
            if (b, s) == here:
                continue
            n = st.name(b, s) or ("empty" if st.slot(b, s) is None else "—")
            out.append(((b, s), f"{b + 1}.{s + 1}  {n}"))
    return out


def slot_panel(ctx, col: QVBoxLayout):
    st = ctx.macros
    b, s = ctx.mac_bank, ctx.mac_slot
    snap = st.slot(b, s)
    col.addWidget(W.title(f"Slot {b + 1}.{s + 1}"))
    card = W.Card("Slot", Theme.function)
    card.add(W.row("Name", _line_edit(st.name(b, s), "e.g. drop, breakdown", lambda t: (st.set_name(b, s, t), ctx.changed()))))
    times = st.glide_times()
    card.add(W.row("Slew", W.Picker([(k, f"{k + 1} · {MC.fmt_secs(times[k])}") for k in range(MC.SLEW_STEPS)],
                                    st.slew(b, s), lambda v: (st.set_slew(b, s, v), _changed(ctx)), 150)))
    card.add(W.hint("The slew fader on the grid: step 1 is the key just above load. The times are in Settings."))
    if snap is not None:
        when = dt.datetime.fromtimestamp(snap.get("saved_at") or 0).strftime("%d %b %Y, %H:%M")
        card.add(W.row("Saved", W.label(when, 12, REGULAR, "mono", Theme.dim)))
    col.addWidget(card)

    if snap is None:
        col.addWidget(W.hint("Empty. Save into it on the grid: hold its top key for ½ s. Or copy another slot here."))
    else:
        act = W.Card("On the Pi", Theme.special)
        tip = "Upload your changes first: ▶ loads what's on the Pi" if ctx.macros.dirty else ""
        bt = W.small_button("▶  Load this slot now", lambda: ctx.macros_audition(b, s), enabled=not ctx.macros.dirty)
        bt.setToolTip(tip or "The Pi loads it as if you pressed its load key on the grid (with its slew)")
        act.add(W.hbox(bt, stretch_end=True))
        if ctx.macros.dirty:
            act.add(W.hint("You have changes that aren't on the Pi yet: ↑ Upload first."))
        col.addWidget(act)
        parts(ctx, col, b, s, snap)

    ops = W.Card("Copy · move · clear")
    tgt = {"to": next((t for t, _ in _targets(st, (b, s)) if st.slot(*t) is None), (b, (s + 1) % MC.SLOTS))}
    ops.add(W.row("To", W.Picker(_targets(st, (b, s)), tgt["to"], lambda v: tgt.__setitem__("to", v), 190)))

    def do(kind):
        t = tgt["to"]
        if kind == "copy":
            st.copy_slot((b, s), t)
        else:
            st.move_slot((b, s), t)
        ctx.mac_bank, ctx.mac_slot = t
        _changed(ctx)

    ops.add(W.hbox(W.small_button("Copy", lambda: do("copy"), snap is not None),
                   W.small_button("Move", lambda: do("move")), stretch_end=True))
    ops.add(W.hint("Move swaps with what's at the target, so nothing is lost."))
    ops.add(W.hbox(W.small_button("Clear this slot", lambda: (st.clear(b, s), _changed(ctx)), snap is not None), stretch_end=True))
    col.addWidget(ops)


def parts(ctx, col: QVBoxLayout, b: int, s: int, snap: dict):
    st = ctx.macros
    state = snap.get("state") or {}
    scope = st.providers_for(b)
    ds = state.get("dualsense")
    if ds:
        card = W.Card("DualSense script", Theme.octave)
        card.add(W.hbox(W.label(MC.ds_summary(ds), 12, SEMIBOLD, "mono"), "stretch",
                        W.icon_button("✕", lambda: (st.remove_part(b, s, "dualsense"), _changed(ctx)),
                                      "Take the DualSense part out: loading leaves the slice and freeze alone")))
        rows = [("Sample block", ds.get("block")), ("Section", ds.get("section")), ("Slice in section", ds.get("fine"))]
        fr = ds.get("freeze") or {}
        rows += [("Freeze L2 / R2 / left stick", "on" if fr.get("left") else "off"),
                 ("Freeze right stick", "on" if fr.get("right") else "off"),
                 ("Motion control", "on" if ds.get("motion") else "off")]
        for k, v in rows:
            card.add(W.row(k, W.label(str(v), 12, SEMIBOLD, "mono", Theme.dim)))
        jump = st.settings.get("jump_at", "end")
        card.add(W.hint(f"Doesn't glide: set at the {jump} of a load, whichever control moves it later "
                        "(stick, d-pad, pedals …)."))
        if "dualsense" not in scope:
            card.add(W.hint("Bank setting: this bank doesn't load the DualSense part."))
        col.addWidget(card)

    vals = MC.midi_values(snap)
    if vals:
        from .. import ssp as S
        senders = S.senders(ctx)
        card = W.Card(f"MIDI values · {len(vals)}", Theme.cv)
        grid = QWidget()
        g = QGridLayout(grid)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(8)
        g.setVerticalSpacing(4)
        pk = set(st.settings.get("pickup_sources") or [])
        fb = set(st.settings.get("feedback_sources") or [])
        for i, (kind, ch, num, value, src) in enumerate(vals):
            name = MC.value_name(ctx, kind, ch, num, src, senders)
            dest = MC.destinations(ctx, kind, ch, num)
            nm = W.label(name, 11.5, SEMIBOLD)
            nm.setWordWrap(True)
            tags = []
            if src in pk:
                tags.append("pickup")
            if src in fb:
                tags.append("sent back")
            sub = " · ".join(dest) if dest else "no ER-301 route / SSP input known"
            sl = W.label(sub + (f"  ({', '.join(tags)})" if tags else ""), 10, REGULAR, "mono",
                         Theme.cv if dest else Theme.dim, wrap=True)
            v = W.label(str(value), 12, BOLD, "mono")
            x = W.icon_button("✕", lambda i=i: (st.remove_value(b, s, i), _changed(ctx)),
                              "Take this value out: loading leaves it where it is")
            g.addWidget(nm, i * 2, 0)
            g.addWidget(v, i * 2, 1, Qt.AlignRight)
            g.addWidget(x, i * 2, 2)
            g.addWidget(sl, i * 2 + 1, 0, 1, 3)
        g.setColumnStretch(0, 1)
        card.add(grid)
        card.add(W.hint("Glide from where they are to these values. Names and destinations come from the 16n / EC4 / "
                        "routing loaded in Kontrolleur and the SSP's patch in use."))
        if "midi" not in scope:
            card.add(W.hint("Bank setting: this bank doesn't load MIDI values."))
        col.addWidget(card)

    tt = state.get("teletype")
    if tt:
        card = W.Card("Teletype", Theme.c("body", 0.9))
        lit = sum(1 for v in tt.get("leds") or [] if v)
        card.add(W.hbox(W.label(f"its half of the grid · {lit} keys lit", 12, SEMIBOLD, "mono"), "stretch",
                        W.icon_button("✕", lambda: (st.remove_part(b, s, "teletype"), _changed(ctx)), "Take it out")))
        card.add(W.hint("Kept with the snapshot so you can see what Teletype showed. Not restored yet."))
        col.addWidget(card)


def bank_panel(ctx, col: QVBoxLayout):
    st = ctx.macros
    b = ctx.mac_bank
    col.addWidget(W.title(f"Bank {b + 1}"))
    card = W.Card("Bank", Theme.function)
    card.add(W.row("Name", _line_edit(st.bank_name(b), "e.g. set A, ambient", lambda t: (st.set_bank_name(b, t), ctx.changed()))))
    for s in range(MC.SLOTS):
        n = st.name(b, s) or ("empty" if st.slot(b, s) is None else "—")
        btn = W.small_button(f"{b + 1}.{s + 1}  {n}", lambda s=s: ctx.select("mac_slot", s, "Slot"))
        card.add(W.hbox(btn, W.label(MC.fmt_secs(st.glide(b, s)), 11, REGULAR, "mono", Theme.dim), stretch_end=True))
    col.addWidget(card)

    scope = W.Card("What this bank saves and loads", Theme.trig)
    per = (st.settings.get("bank_providers") or {}).get(str(b))
    cur = set(st.providers_for(b))

    def toggle(name, on):
        names = set(st.providers_for(b))
        names.add(name) if on else names.discard(name)
        allp = set(st.settings.get("providers") or MC.PROVIDER_NAMES)
        st.set_bank_providers(b, None if names == allp else [p for p in MC.PROVIDER_NAMES if p in names])
        _changed(ctx)

    for name, label in MC.PROVIDERS:
        scope.add(W.switch_row(label, name in cur, lambda on, n=name: toggle(n, on)))
    scope.add(W.hint("e.g. a bank of mixer scenes that only touches MIDI values and leaves the slice alone."
                     + ("" if per is not None else " Now: like every bank (Settings).")))
    col.addWidget(scope)

    cp = W.Card("Copy the bank")
    tgt = {"to": (b + 1) % MC.BANKS}
    cp.add(W.row("To bank", W.Picker([(k, f"{k + 1}" + (f"  {st.bank_name(k)}" if st.bank_name(k) else "")
                                          + ("" if st.bank_used(k) else "  (empty)")) for k in range(MC.BANKS) if k != b],
                                     tgt["to"], lambda v: tgt.__setitem__("to", v), 160)))
    cp.add(W.hbox(W.small_button("Copy all 7 slots", lambda: (st.copy_bank(b, tgt["to"]), ctx.select("mac_bank", tgt["to"]),
                                                                    ctx.changed())),
                  stretch_end=True))
    cp.add(W.hint("Replaces the slots of the target bank."))
    col.addWidget(cp)


def settings_panel(ctx, col: QVBoxLayout):
    st = ctx.macros
    se = st.settings
    col.addWidget(W.title("Settings"))
    card = W.Card("Slew times", Theme.cv)
    times = st.glide_times()

    def set_time(k, v):
        t = st.glide_times()
        t[k] = round(float(v), 2)
        se["glide_times"] = t
        ctx.changed()

    for k in range(MC.SLEW_STEPS):
        card.add(W.row(f"Step {k + 1}" + (" (just above load)" if k == 0 else " (just below save)" if k == 5 else ""),
                       W.DecimalField(times[k], 0.0, 600.0, lambda v, k=k: set_time(k, v))))
    card.add(W.row("Curve", W.Picker(MC.CURVES, se.get("curve", "linear"), lambda v: (se.__setitem__("curve", v), ctx.changed()), 200)))
    card.add(W.row("Slice / freeze change", W.Segmented([("start", "At the start"), ("end", "At the end")], se.get("jump_at", "end"),
                                                       lambda v: (se.__setitem__("jump_at", v), _changed(ctx)))))
    col.addWidget(card)

    card = W.Card("Grid", Theme.function)
    card.add(W.row("Hold save for (s)", W.DecimalField(float(se.get("hold_to_save", 0.5)), 0.1, 3.0,
                                                       lambda v: (se.__setitem__("hold_to_save", round(float(v), 2)), ctx.changed()))))
    col.addWidget(card)

    card = W.Card("Remembered controllers", Theme.special)
    card.add(W.hint("Which controllers' values go into a snapshot (their last CC / NRPN values), and how they behave after a load."))
    head = QWidget()
    g = QGridLayout(head)
    g.setContentsMargins(0, 0, 0, 0)
    for c, t in enumerate(["", "Remember", "Pickup", "Send back"]):
        g.addWidget(W.label(t, 10, HEAVY, "mono", Theme.dim), 0, c, Qt.AlignCenter if c else Qt.AlignLeft)
    for r, (key, label) in enumerate(MC.SOURCES, start=1):
        g.addWidget(W.label(label, 12, SEMIBOLD), r, 0)
        for c, setting in enumerate(("midi_sources", "pickup_sources", "feedback_sources"), start=1):
            cb = QCheckBox()
            cb.setChecked(key in (se.get(setting) or []))

            def flip(on, key=key, setting=setting):
                lst = [x for x in (se.get(setting) or []) if x != key]
                if on:
                    lst.append(key)
                se[setting] = lst
                ctx.changed()

            cb.toggled.connect(flip)
            g.addWidget(cb, r, c, Qt.AlignCenter)
    g.setColumnStretch(0, 1)
    card.add(head)
    card.add(W.hint("Pickup: after a load the fader does nothing until it passes the value (16n). Send back: the "
                    "controller gets the new values (EC4 encoders). Pedals usually send presses, not settings: "
                    "they change the slice through the DualSense script, which is saved anyway."))
    card.add(W.row("Pickup window", W.NumberField(int(se.get("pickup_window", 2)), 0, 20,
                                                 lambda v: (se.__setitem__("pickup_window", v), ctx.changed()))))
    col.addWidget(card)

    card = W.Card("Saved in every slot")
    allp = set(se.get("providers") or MC.PROVIDER_NAMES)

    def flipp(name, on):
        names = set(se.get("providers") or MC.PROVIDER_NAMES)
        names.add(name) if on else names.discard(name)
        se["providers"] = [p for p in MC.PROVIDER_NAMES if p in names]
        _changed(ctx)

    for name, label in MC.PROVIDERS:
        card.add(W.switch_row(label, name in allp, lambda on, n=name: flipp(n, on)))
    card.add(W.hint("Banks can narrow this down (Bank tab)."))
    col.addWidget(card)

    card = W.Card("Backup")
    card.add(W.hint("All banks, names and settings as one file on this computer — e.g. one per live set. "
                    "Restoring replaces what Kontrolleur shows; ↑ Upload puts it on the Pi."))
    card.add(W.hbox(W.small_button("Save as file…", lambda: export_file(ctx)),
                    W.small_button("Restore from file…", lambda: import_file(ctx)), stretch_end=True))
    if st.path:
        card.add(W.hint(f"On the Pi: {st.path} (the Pi keeps the last 10 versions in backups/ next to it)."))
    col.addWidget(card)


def _folder() -> Path:
    from .. import store
    try:
        return store.config_folder()
    except Exception:
        return Path.home()


def export_file(ctx):
    name = dt.datetime.now().strftime("grid-macros-%Y%m%d-%H%M.json")
    path, _ = QFileDialog.getSaveFileName(ctx, "Save the grid macros", str(_folder() / name), "Grid macros (*.json)")
    if not path:
        return
    Path(path).write_text(json.dumps(ctx.macros.export_set(), indent=1))
    ctx.set_status("ok", f"Saved the grid macros to {path}.")


def import_file(ctx):
    path, _ = QFileDialog.getOpenFileName(ctx, "Restore grid macros", str(_folder()), "Grid macros (*.json)")
    if not path:
        return
    try:
        ctx.macros.import_set(json.loads(Path(path).read_text()))
    except (OSError, ValueError) as e:
        ctx.set_status("error", f"Can't restore {path}: {e}")
        return
    ctx.set_status("ok", f"Restored {Path(path).name}: ↑ Upload to put it on the Pi.")
    _changed(ctx)
