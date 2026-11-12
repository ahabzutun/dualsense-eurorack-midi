"""DualSense tab: the controller drawing and its inspector (Control / Combos / Overview)."""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath, QTransform
from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget

from .. import draw as D
from .. import model as M
from .. import widgets as W
from ..stage import Drawing
from ..store import NotesStore
from ..theme import BOLD, HEAVY, MEDIUM, REGULAR, SEMIBOLD, Theme

# The controller is drawn in its own 900 × 600 space, placed into 1100 × 640
S, CX, CY = 0.80, 550.0, 330.0


def gp(x, y) -> QPointF:
    return QPointF(CX + (x - 450) * S, CY + (y - 290) * S)


def gd(v) -> float:
    return v * S


def body_path() -> QPainterPath:
    p = QPainterPath()
    p.moveTo(300, 112)
    p.quadTo(450, 96, 600, 112)
    p.cubicTo(682, 104, 737, 120, 762, 160)
    p.cubicTo(803, 222, 845, 342, 838, 432)
    p.cubicTo(834, 500, 792, 532, 746, 522)
    p.cubicTo(702, 512, 682, 432, 640, 410)
    p.cubicTo(560, 386, 340, 386, 260, 410)
    p.cubicTo(218, 432, 198, 512, 154, 522)
    p.cubicTo(108, 532, 66, 500, 62, 432)
    p.cubicTo(55, 342, 97, 222, 138, 160)
    p.cubicTo(163, 120, 218, 104, 300, 112)
    p.closeSubpath()
    t = QTransform()
    t.translate(CX - 450 * S, CY - 290 * S)
    t.scale(S, S)
    return t.map(p)


CALLOUTS = [  # group, anchor, side, y
    ("l2", (147, 62), "left", 66), ("l1", (153, 100), "left", 124), ("create", (294, 142), "left", 182),
    ("dpad", (158, 214), "left", 258), ("leftStick", (284, 318), "left", 380),
    ("r2", (753, 62), "right", 66), ("r1", (747, 100), "right", 124), ("options", (606, 142), "right", 182),
    ("triangle", (710, 168), "right", 236), ("circle", (756, 214), "right", 280), ("cross", (710, 260), "right", 324),
    ("square", (642, 236), "right", 368), ("rightStick", (616, 318), "right", 436), ("touchpad", (450, 119), "top", 44),
]


def token_row(mapping: M.Mapping, i: M.InputDef, compact=False, show_label=True) -> D.Item:
    items = []
    if show_label:
        items.append(D.Text(i.short, Theme.font(8 if compact else 10, SEMIBOLD, "mono"), Theme.dim))
    for text, kind in mapping.tokens(i):
        col = Theme.token_color(kind)
        off = kind[0] == "off"
        t = D.Text(text, Theme.font(9 if compact else 10.5, BOLD, "mono"), col)
        fill = QColor(col)
        fill.setAlphaF(col.alphaF() * 0.14)
        items.append(D.Pill(t, 0 if off else 6, 2, None if off else fill))
    return D.Row(items, 5)


class ControllerDrawing(Drawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self.selected = "cross"
        self._body = body_path()

    @property
    def m(self) -> M.Mapping:
        return self.ctx.mapping

    def select(self, g: str):
        self.selected = g
        self.ctx.select_ds(g)

    def accent(self, g: str) -> QColor:
        for i in M.GROUP[g].inputs:
            for _, k in self.m.tokens(i):
                if k[0] != "off":
                    return Theme.token_color(k)
        return Theme.body

    def mapped(self, g: str) -> bool:
        return any(k[0] != "off" for i in M.GROUP[g].inputs for _, k in self.m.tokens(i))

    def paint(self, p: QPainter, w, h):
        body = self._body
        p.fillPath(body, D.vgradient(QRectF(0, 0, w, h), Theme.c("body", 0.10), Theme.c("body", 0.03)))
        p.strokePath(body, D.pen(Theme.c("body", 0.05), 12))
        p.strokePath(body, D.pen(Theme.c("body", 0.07), 7))
        p.strokePath(body, D.pen(Theme.c("body", 0.55), 1.6))
        for t, x in (("CREATE", 302), ("OPTIONS", 598)):
            D.Text(t, Theme.font(8, SEMIBOLD, "mono"), Theme.dim).paint_center(p, *_xy(gp(x, 178)))
        self._callouts(p)
        self._controls(p)

    # ── controls ──
    def _hot(self, p, g, shape, size, at, label, radius=0.0):
        sel = self.selected == g
        col = self.accent(g)
        mp = self.mapped(g)
        c = gp(*at)
        w, h = gd(size[0]), gd(size[1])
        r = QRectF(c.x() - w / 2, c.y() - h / 2, w, h)
        path = QPainterPath()
        if shape == "circle":
            path.addEllipse(r)
        elif shape == "capsule":
            path.addRoundedRect(r, h / 2, h / 2)
        else:
            path.addRoundedRect(r, gd(radius), gd(radius))
        if sel:
            D.glow(p, path, _op(col, 0.8), 10)
        D.draw_path(p, path, _op(col, 0.16 if mp else 0.04),
                    Theme.ink if sel else _op(col, 0.9 if mp else 0.5), 2.2 if sel else 1.4)
        if label:
            D.Text(label, Theme.font(11 if len(label) > 2 else 10, BOLD, "mono"),
                   Theme.ink if sel else _op(col, 0.85)).paint_center(p, c.x(), c.y())
        self.hits.add(path, lambda: self.select(g))

    def _controls(self, p):
        self._hot(p, "l2", "rect", (130, 36), (212, 62), "L2", 15)
        self._hot(p, "r2", "rect", (130, 36), (688, 62), "R2", 15)
        self._hot(p, "l1", "rect", (118, 24), (212, 100), "L1", 11)
        self._hot(p, "r1", "rect", (118, 24), (688, 100), "R1", 11)
        self._hot(p, "touchpad", "rect", (236, 134), (450, 186), "TOUCHPAD", 26)
        self._hot(p, "create", "rect", (18, 36), (302, 142), "", 9)
        self._hot(p, "options", "rect", (18, 36), (598, 142), "", 9)
        self._hot(p, "ps", "circle", (34, 34), (450, 296), "PS")
        self._hot(p, "mic", "capsule", (36, 14), (450, 332), "")
        self._dpad(p)
        self._face(p, "triangle", "triangle", (688, 168))
        self._face(p, "circle", "circle", (734, 214))
        self._face(p, "cross", "xmark", (688, 260))
        self._face(p, "square", "square", (642, 214))
        self._stick(p, "leftStick", (334, 318))
        self._stick(p, "rightStick", (566, 318))
        self._center_tokens(p)
        self._chip(p, "motion", "gyroscope", "MOTION", (352, 470))
        self._chip(p, "slice", "grid", "GRAIN SLICE", (548, 470))

    def _dpad(self, p):
        g = "dpad"
        sel = self.selected == g
        ins = M.GROUP[g].inputs

        def col(i):
            k = self.m.tokens(i)[0][1]
            return Theme.body if k[0] == "off" else Theme.token_color(k)

        def on(i):
            return self.m.tokens(i)[0][1][0] != "off"

        vcol, hcol = col(ins[0]), col(ins[1])
        arm, ln = gd(34), gd(108)
        c = gp(212, 214)
        hr = QRectF(c.x() - ln / 2, c.y() - arm / 2, ln, arm)
        vr = QRectF(c.x() - arm / 2, c.y() - ln / 2, arm, ln)
        if sel:
            gpth = QPainterPath()
            gpth.addRoundedRect(hr, gd(7), gd(7))
            gpth.addRoundedRect(vr, gd(7), gd(7))
            D.glow(p, gpth.simplified(), _op(vcol, 0.8), 10)
        D.rounded(p, hr, gd(7), _op(hcol, 0.16 if on(ins[1]) else 0.04), Theme.ink if sel else _op(hcol, 0.75), 2 if sel else 1.3)
        D.rounded(p, vr, gd(7), _op(vcol, 0.16 if on(ins[0]) else 0.04), Theme.ink if sel else _op(vcol, 0.75), 2 if sel else 1.3)
        D.symbol(p, "arrows4", c, 13, Theme.ink if sel else Theme.dim, 1.3)
        self.hits.add(QRectF(c.x() - ln / 2, c.y() - ln / 2, ln, ln), lambda: self.select(g))

    def _face(self, p, g, sym, at):
        sel = self.selected == g
        col = self.accent(g)
        mp = self.mapped(g)
        c = gp(*at)
        d = gd(44)
        path = QPainterPath()
        path.addEllipse(c, d / 2, d / 2)
        if sel:
            D.glow(p, path, _op(col, 0.8), 10)
        D.draw_path(p, path, _op(col, 0.18 if mp else 0.04), Theme.ink if sel else _op(col, 0.95 if mp else 0.5), 2.2 if sel else 1.4)
        D.symbol(p, sym, c, 13, Theme.ink if sel else _op(col, 0.9), 1.8)
        self.hits.add(path, lambda: self.select(g))

    def _stick(self, p, g, at):
        sel = self.selected == g
        col = self.accent(g)
        mp = self.mapped(g)
        c = gp(*at)
        outer = QPainterPath()
        outer.addEllipse(c, gd(50), gd(50))
        if sel:
            D.glow(p, outer, _op(col, 0.8), 12)
        D.draw_path(p, outer, Theme.c("bg", 0.6), _op(col, 0.85 if mp else 0.45), 1.3)
        D.circle(p, c, gd(66), _op(col, 0.18 if mp else 0.05), Theme.ink if sel else _op(col, 0.8), 2.2 if sel else 1.3)
        D.circle(p, c, gd(40), None, _op(col, 0.35), 1)
        cross = QPainterPath()
        k = gd(8)
        cross.moveTo(c.x() - k, c.y()); cross.lineTo(c.x() + k, c.y())
        cross.moveTo(c.x(), c.y() - k); cross.lineTo(c.x(), c.y() + k)
        p.strokePath(cross, D.pen(_op(col, 0.6), 1))
        self.hits.add(outer, lambda: self.select(g))

    def _chip(self, p, g, icon, text, at):
        sel = self.selected == g
        col = self.accent(g)
        mp = self.mapped(g)
        group = M.GROUP[g]
        fg = Theme.ink if sel else _op(col, 0.9)
        icon_item = D.Custom(13, 13, lambda pp, r: D.symbol(pp, icon, r.center(), 12, fg, 1.3))
        head = D.Row([icon_item, D.Text(text, Theme.font(10, BOLD, "mono"), fg)], 6)
        chip_item = D.Pill(head, 12, 7, _op(col, 0.16 if mp else 0.05), Theme.ink if sel else _op(col, 0.7),
                           2 if sel else 1.2, dash=[4, 3])
        rows = D.Col([token_row(self.m, i, True, len(group.inputs) > 1) for i in group.inputs], 2)
        whole = D.Col([chip_item, rows], 5)
        c = gp(at[0], at[1] + 26)
        w, h = whole.size()
        x, y = c.x() - w / 2, c.y() - h / 2
        cw, chh = chip_item.size()
        if sel:
            pth = QPainterPath()
            pth.addRoundedRect(QRectF(x + (w - cw) / 2, y, cw, chh), chh / 2, chh / 2)
            D.glow(p, pth, _op(col, 0.8), 10)
        whole.paint(p, x, y)
        self.hits.add(QRectF(x, y, w, h), lambda: self.select(g))

    def _center_tokens(self, p):
        rows = []
        for g in ("ps", "mic"):
            lab = D.Text("PS" if g == "ps" else "MIC", Theme.font(8, HEAVY, "mono"), Theme.ink if self.selected == g else Theme.dim)
            rows.append(D.Row([lab, token_row(self.m, M.GROUP[g].inputs[0], True, False)], 5))
        col = D.Col(rows, 3)
        c = gp(450, 372)
        w, h = col.size()
        x, y = c.x() - w / 2, c.y() - h / 2
        col.paint(p, x, y)
        cy = y
        for g, r in zip(("ps", "mic"), rows):
            rw, rh = r.size()
            self.hits.add(QRectF(x + (w - rw) / 2, cy, rw, rh), lambda g=g: self.select(g))
            cy += rh + 3

    # ── callouts ──
    def _callouts(self, p):
        for g, anchor, side, y in CALLOUTS:
            a = gp(*anchor)
            col = self.accent(g)
            sel = self.selected == g
            mp = self.mapped(g)
            line = QPainterPath()
            if side == "left":
                line.moveTo(214, y); line.lineTo(244, y); line.lineTo(a)
            elif side == "right":
                line.moveTo(886, y); line.lineTo(856, y); line.lineTo(a)
            else:
                line.moveTo(a.x(), y + 26); line.lineTo(a)
            p.strokePath(line, D.pen(_op(Theme.ink, 0.8) if sel else _op(col, 0.55 if mp else 0.18),
                                     1.4 if sel else 1, None if mp else [3, 4]))
            D.circle(p, a, 5, Theme.ink if sel else _op(col, 0.9 if mp else 0.35))

            item = self._label(g, sel, side)
            w, h = item.size()
            if side == "left":
                x = 210 - w
            elif side == "right":
                x = 890
            else:
                x = a.x() - w / 2
            item.paint(p, x, y - h / 2)
            self.hits.add(QRectF(x, y - h / 2, w, h), lambda g=g: self.select(g))

    def _label(self, g, sel, side) -> D.Item:
        group = M.GROUP[g]
        align = "trailing" if side == "left" else ("leading" if side == "right" else "center")
        items = [D.Text(group.title.upper(), Theme.font(10, HEAVY, "mono"), Theme.ink if sel else _op(Theme.ink, 0.85), tracking=1.2)]
        if len(group.inputs) == 1:
            items.append(token_row(self.m, group.inputs[0], False, False))
        elif g == "touchpad":
            items.append(D.Row([token_row(self.m, i) for i in group.inputs], 14))
        else:
            items += [token_row(self.m, i) for i in group.inputs]
        ns = [n for n in (self.ctx.notes.note(NotesStore.ds(i.id)) for i in group.inputs) if n]
        if ns:
            items.append(D.note_badge(" / ".join(ns), 9.5, 480 if g == "touchpad" else 190))
        return D.Col(items, 3, align)


def _xy(pt: QPointF):
    return pt.x(), pt.y()


def _op(c: QColor, o: float) -> QColor:
    q = QColor(c)
    q.setAlphaF(max(0.0, min(1.0, c.alphaF() * o)))
    return q


# ── inspector ───────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    if tab == "Control":
        control_panel(ctx, M.GROUP[ctx.ds_selected], col)
    elif tab == "Combos":
        combos_panel(ctx, col)
    else:
        overview_panel(ctx, col)


def control_panel(ctx, group: M.GroupDef, col: QVBoxLayout):
    head = QWidget()
    hv = QVBoxLayout(head)
    hv.setContentsMargins(0, 0, 0, 4)
    hv.setSpacing(6)
    hv.addWidget(W.title(group.title))
    hv.addWidget(W.hint(group.hint))
    col.addWidget(head)
    for i in group.inputs:
        col.addWidget(input_editor(ctx, i))
        key = NotesStore.ds(i.id)
        col.addWidget(W.NoteField(ctx.notes.raw(key), lambda t, k=key: ctx.set_note(k, t),
                                  f"Note · {i.label}" if len(group.inputs) > 1 else "Note"))


def _accent(m: M.Mapping, i: M.InputDef) -> QColor:
    for _, k in m.tokens(i):
        if k[0] != "off":
            return Theme.token_color(k)
    return Theme.dim


def input_editor(ctx, i: M.InputDef) -> QWidget:
    m = ctx.mapping
    role = m.role(i)
    sends = m.sends_midi(i)
    has_function = i.button is not None or i.id in ("dpad_horizontal", "left_stick_x")
    card = W.Card(i.label, _accent(m, i))
    if i.button:
        b = i.button
        f = m.fn(b)

        def set_tap(a, b=b):
            m.set_tap(a, b)
            ctx.changed(rebuild=True)

        def set_hold(a, b=b):
            m.set_hold(a, b)
            ctx.changed(rebuild=True)

        card.add(W.row("Tap", W.Picker([(a, M.ACTION_TITLE[a]) for a in M.ACTIONS], f.tap, set_tap, 230), SEMIBOLD))
        card.add(W.row("Hold", W.Picker([(a, M.ACTION_TITLE[a]) for a in M.HOLD_CHOICES], f.hold, set_hold, 230), SEMIBOLD))
        card.add(W.hint(_timing(m, f)))
        combos = [c for c in m.chords if c.a == b or c.b == b]
        if combos:
            card.add(W.label("\n".join(f"Combo {M.button_label(c.a)} + {M.button_label(c.b)} → {M.ACTION_TITLE[c.action]}"
                                       for c in combos), 11, REGULAR, "ui", _op(Theme.function, 0.85), wrap=True))
    if i.id == "dpad_horizontal":
        card.add(W.Segmented([("slice_section", "Slice section ±16"), ("midi", "MIDI CC steps")], m.dpad_horizontal,
                             lambda v: (setattr(m, "dpad_horizontal", v), ctx.changed(rebuild=True))))
    if i.id == "left_stick_x":
        card.add(W.Segmented([("slice_fine", "Slice in section"), ("midi", "MIDI CC")], m.left_stick_x,
                             lambda v: (setattr(m, "left_stick_x", v), ctx.changed(rebuild=True))))
    if sends:
        if has_function:
            card.add(W.divider())
            card.add(W.label("SEND TO ER-301", 9, HEAVY, "mono", Theme.dim, tracking=1))
        card.add(W.Segmented([(r, M.ROLE_TITLE[r]) for r in i.roles], role,
                             lambda r: (m.set_role(r, i), ctx.changed(rebuild=True))))
        if M.role_uses_port(role):
            def set_port(v):
                m.ports[i.id] = v
                ctx.changed()
            card.add(W.row("SC.CV port" if role == M.CV else f"{m.trigger_port_name} port",
                           W.NumberField(m.port(i), 1, 99, set_port)))
        card.add(W.hint(_explanation(m, i, role)))
    n = m.number(i)
    name = f" · {i.name}" if i.name else ""
    src = {"cc": f"MIDI ch {m.channel} · CC {n}{name}", "nrpn": f"MIDI ch {m.channel} · NRPN {n} (14-bit){name}"}.get(
        i.kind, f"MIDI ch {m.channel} · note {n}")
    card.add(W.label(src, 10, REGULAR, "mono", _op(Theme.dim, 0.8)))
    return card


def _timing(m: M.Mapping, f: M.ButtonFunc) -> str:
    hold = "%g" % m.long_press
    if f.hold != "none":
        return f"Shorter than {hold} s = tap, longer = hold; decided when you let go."
    if f.tap == "midi":
        return "Sends its CC: 127 while pressed, 0 on release."
    if f.tap == "none":
        return "Does nothing."
    return "Fires when you let go." if M.fires_on_release(f.tap) else "Fires when you press."


def _explanation(m: M.Mapping, i: M.InputDef, r: str) -> str:
    if r == M.OFF:
        return "Not sent to the ER-301."
    if r == M.CV:
        return "0–16383 → 0–10 V in full resolution." if i.kind == "nrpn" else "0–127 → 0–10 V."
    if r == M.TRIGGER:
        return "10 V while pressed, 0 V when released (on an SC.CV port)." if m.triggers_as_cv else "SC.TR high while pressed."
    if r == M.NOTE_TRIGGER:
        return "10 V while the note is held (on an SC.CV port)." if m.triggers_as_cv else "SC.TR high while the note is held."
    if r == M.LFO_RATE:
        return "Sets the trigger LFO speed. 0 = stopped. Port and range: ER-301 tab → Pitch & LFO. Only one control can do this."
    if r == M.LFO_RESET:
        return "Pressing restarts the LFO phase (a trigger right away). Only one control can do this."
    return f"Shifts the pitch CV by {m.semis_per_step} semitones per step. Only one control can do this."


def combos_panel(ctx, col: QVBoxLayout):
    m = ctx.mapping
    col.addWidget(W.title("Combos"))
    col.addWidget(W.hint("Two buttons pressed together. Their own release actions are skipped then, so L1 + R1 doesn't also delete or play the loop."))
    if not m.controls_on_pi:
        col.addWidget(W.hint("⚠︎ The SSH host doesn't have the button functions installed yet: only the ER-301 routing is uploaded."))
    buttons = [(n, lab) for n, lab in M.BUTTONS]
    for c in list(m.chords):
        card = W.Card(f"{M.button_label(c.a)} + {M.button_label(c.b)}", Theme.function)

        def set_a(v, c=c):
            c.a = v
            ctx.changed(rebuild=True)

        def set_b(v, c=c):
            c.b = v
            ctx.changed(rebuild=True)

        def remove(c=c):
            m.chords = [x for x in m.chords if x.id != c.id]
            ctx.changed(rebuild=True)

        def set_action(v, c=c):
            c.action = v
            ctx.changed()

        plus = W.label("+", 12, REGULAR, "ui", Theme.dim)
        card.add(W.hbox(W.Picker(buttons, c.a, set_a, 110), plus, W.Picker(buttons, c.b, set_b, 110), "stretch",
                        W.icon_button("✕", remove, "Remove this combo")))
        card.add(W.Picker([(a, M.ACTION_TITLE[a]) for a in M.CHORD_CHOICES], c.action, set_action))
        col.addWidget(card)

    def add():
        m.chords.append(M.Chord("triangle", "square", "motion"))
        ctx.changed(rebuild=True)

    b = QPushButton("＋  Add combo")
    b.setCursor(Qt.PointingHandCursor)
    b.setMinimumHeight(32)
    b.clicked.connect(add)
    col.addWidget(b)


def overview_panel(ctx, col: QVBoxLayout):
    m = ctx.mapping
    col.addWidget(W.title("DualSense"))
    card = W.Card("MIDI")

    def set_ch(v):
        m.channel = v
        ctx.changed()

    card.add(W.row("MIDI channel", W.NumberField(m.channel, 1, 16, set_ch)))
    card.add(W.hint("The instrument channel the DualSense sends on (pedals on channel 1 follow it in the MIDI hub). The ER-301 routes use it as CHANNEL."))
    col.addWidget(card)
    card = W.Card("All buttons", Theme.function)
    for name, lab in M.BUTTONS:
        f = m.fn(name)
        w = QWidget()
        from PySide6.QtWidgets import QHBoxLayout
        h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(6)
        ll = W.label(lab, 12, BOLD, "mono", wrap=True)
        ll.setFixedWidth(int(92 * Theme.grow))
        ll.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        h.addWidget(ll, 0, Qt.AlignTop)
        v = QVBoxLayout()
        v.setSpacing(1)
        inp = M.input_for_button(name)
        tap = ("MIDI → " + (m.route_token(inp) if inp else "")) if f.tap == "midi" else M.ACTION_TITLE[f.tap]
        v.addWidget(W.label(tap, 12, REGULAR, "ui", Theme.dim if f.tap == "none" else Theme.ink, wrap=True))
        if f.hold != "none":
            v.addWidget(W.label("hold: " + M.ACTION_TITLE[f.hold], 11, REGULAR, "ui", Theme.function, wrap=True))
        h.addLayout(v, 1)
        card.add(w)
    card.add(W.hint(f"Hold = {'%g' % m.long_press} s or longer (LONG_PRESS_DURATION in mappings.py)."))
    col.addWidget(card)
