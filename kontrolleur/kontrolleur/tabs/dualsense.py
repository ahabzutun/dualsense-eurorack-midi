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
    ("l2", (147, 57), "left", 66), ("l1", (153, 94), "left", 124), ("create", (294, 142), "left", 182),
    ("dpad", (158, 214), "left", 258), ("leftStick", (284, 318), "left", 380),
    ("r2", (753, 57), "right", 66), ("r1", (747, 94), "right", 124), ("options", (606, 142), "right", 182),
    ("triangle", (710, 168), "right", 236), ("circle", (756, 214), "right", 280), ("cross", (710, 260), "right", 324),
    ("square", (642, 236), "right", 368), ("rightStick", (616, 318), "right", 436), ("touchpad", (450, 119), "top", 44),
]


def token_row(mapping: M.Mapping, i: M.InputDef, compact=False, show_label=True, rec=None) -> D.Item:
    """The tokens of one assigned input ('X  CV 4', 'HOLD REC' …). `rec` collects (rect, input id, function)
    of every pill as it is drawn."""
    items = []
    if show_label:
        items.append(D.Text(i.short, Theme.font(8 if compact else 10, SEMIBOLD, "mono"), Theme.dim))
    keys = pill_keys(mapping, i)
    for k, (text, kind) in enumerate(mapping.shown(i)):
        col = _kind_color(kind)
        t = D.Text(text, Theme.font(9 if compact else 10.5, BOLD, "mono"), col)
        pill = D.Pill(t, 6, 2, _op(col, 0.14))
        if rec is not None and k < len(keys):
            pill = D.Decor(pill, after=lambda p, r, key=keys[k]: rec.append((QRectF(r), i.id, key)))
        items.append(pill)
    return D.Row(items, 5)


def pill_keys(m: M.Mapping, i: M.InputDef):
    """The function behind each shown token of an input, in the same order (for the module jacks)."""
    if i.button:
        f, out = m.fn(i.button), []
        if f.tap == "midi":
            if m.effective_role(i) != M.OFF:
                out.append(M.ROLE_ER.get(m.effective_role(i), "midi"))
        elif f.tap != "none":
            out.append(f.tap)
        if f.hold != "none":
            out.append(f.hold)
        return out
    return [m.choice(i)] if m.assigned(i) else []


def short_name(i: M.InputDef) -> str:
    g = M.group_of(i)
    if len(g.inputs) == 1:
        return g.title
    return {"X axis": f"{g.title} X", "Y axis": f"{g.title} Y", "X position": "Touch X", "Y position": "Touch Y",
            "Up / down": "D-pad ↕", "Left / right": "D-pad ↔"}.get(i.label, f"{g.title} {i.label.lower()}")


def _kind_color(kind: tuple) -> QColor:
    if kind[0] == "midi":
        return _op(Theme.ink, 0.75)
    return Theme.token_color(kind)


class ControllerDrawing(Drawing):
    """Neutral drawing of the controller: a control gets its colour and a label only when
    something is assigned to it. What the Pi functions get from it: Routing tab."""
    base = (1100.0, 640.0)

    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self._body = body_path()
        self._pills = []
        self._jacks = {}

    @property
    def m(self) -> M.Mapping:
        return self.ctx.mapping

    @property
    def selected(self) -> str:
        return self.ctx.ds_selected

    def select(self, g: str):
        self.ctx.select_ds(g)

    def accent(self, g: str):
        for i in M.GROUP[g].inputs:
            for _, k in self.m.shown(i):
                return _kind_color(k)
        return None

    def mapped(self, g: str) -> bool:
        return any(self.m.assigned(i) for i in M.GROUP[g].inputs)

    def _colors(self, g):
        """(accent or neutral, assigned?)"""
        a = self.accent(g)
        return (a if a is not None else Theme.body), a is not None

    def paint(self, p: QPainter, w, h):
        body = self._body
        p.fillPath(body, D.vgradient(QRectF(0, 0, w, h), Theme.c("body", 0.10), Theme.c("body", 0.03)))
        p.strokePath(body, D.pen(Theme.c("body", 0.05), 12))
        p.strokePath(body, D.pen(Theme.c("body", 0.07), 7))
        p.strokePath(body, D.pen(Theme.c("body", 0.55), 1.6))
        self._pills, self._jacks = [], {}
        self._callouts(p)
        self._controls(p)

    # ── controls ──
    def _style(self, g, path, glow_r=10):
        sel = self.selected == g
        col, on = self._colors(g)
        if sel:
            D.glow(self._p, path, _op(col, 0.8), glow_r)
        fill = _op(col, 0.16) if on else Theme.c("body", 0.035)
        stroke = Theme.ink if sel else (_op(col, 0.9) if on else Theme.c("body", 0.45))
        D.draw_path(self._p, path, fill, stroke, 2.2 if sel else 1.4)
        return sel, col, on

    def _hot(self, p, g, shape, size, at, label, radius=0.0):
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
        sel, col, on = self._style(g, path)
        if label:
            D.Text(label, Theme.font(10, BOLD, "mono"), Theme.ink if sel else (_op(col, 0.85) if on else Theme.dim)
                   ).paint_center(p, c.x(), c.y())
        self.hits.add(path, lambda: self.select(g), M.GROUP[g].title)

    def _controls(self, p):
        self._p = p
        self._hot(p, "l2", "rect", (130, 36), (212, 57), "L2", 15)
        self._hot(p, "r2", "rect", (130, 36), (688, 57), "R2", 15)
        self._hot(p, "l1", "rect", (118, 24), (212, 94), "L1", 11)
        self._hot(p, "r1", "rect", (118, 24), (688, 94), "R1", 11)
        self._hot(p, "touchpad", "rect", (236, 134), (450, 186), "", 26)
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
        self._motion(p)

    def _dpad(self, p):
        g = "dpad"
        sel = self.selected == g
        ins = M.GROUP[g].inputs

        def col(i):
            t = self.m.shown(i)
            return (_kind_color(t[0][1]), True) if t else (Theme.body, False)

        (vcol, von), (hcol, hon) = col(ins[0]), col(ins[1])
        arm, ln = gd(34), gd(108)
        c = gp(212, 214)
        hr = QRectF(c.x() - ln / 2, c.y() - arm / 2, ln, arm)
        vr = QRectF(c.x() - arm / 2, c.y() - ln / 2, arm, ln)
        if sel:
            gpth = QPainterPath()
            gpth.addRoundedRect(hr, gd(7), gd(7))
            gpth.addRoundedRect(vr, gd(7), gd(7))
            D.glow(p, gpth.simplified(), _op(vcol if von else hcol, 0.8), 10)
        for r, cc, on in ((hr, hcol, hon), (vr, vcol, von)):
            D.rounded(p, r, gd(7), _op(cc, 0.16) if on else Theme.c("body", 0.035),
                      Theme.ink if sel else (_op(cc, 0.8) if on else Theme.c("body", 0.45)), 2 if sel else 1.3)
        D.symbol(p, "arrows4", c, 13, Theme.ink if sel else Theme.dim, 1.3)
        self.hits.add(QRectF(c.x() - ln / 2, c.y() - ln / 2, ln, ln), lambda: self.select(g), "D-pad")

    def _face(self, p, g, sym, at):
        c = gp(*at)
        d = gd(44)
        path = QPainterPath()
        path.addEllipse(c, d / 2, d / 2)
        sel, col, on = self._style(g, path)
        D.symbol(p, sym, c, 13, Theme.ink if sel else (_op(col, 0.9) if on else Theme.c("body", 0.6)), 1.8)
        self.hits.add(path, lambda: self.select(g), M.GROUP[g].title)

    def _stick(self, p, g, at):
        sel = self.selected == g
        col, on = self._colors(g)
        c = gp(*at)
        outer = QPainterPath()
        outer.addEllipse(c, gd(50), gd(50))
        if sel:
            D.glow(p, outer, _op(col, 0.8), 12)
        D.draw_path(p, outer, Theme.c("bg", 0.6), _op(col, 0.85) if on else Theme.c("body", 0.4), 1.3)
        D.circle(p, c, gd(66), _op(col, 0.18) if on else Theme.c("body", 0.035),
                 Theme.ink if sel else (_op(col, 0.8) if on else Theme.c("body", 0.45)), 2.2 if sel else 1.3)
        D.circle(p, c, gd(40), None, _op(col, 0.35) if on else Theme.c("body", 0.18), 1)
        cross = QPainterPath()
        k = gd(8)
        cross.moveTo(c.x() - k, c.y()); cross.lineTo(c.x() + k, c.y())
        cross.moveTo(c.x(), c.y() - k); cross.lineTo(c.x(), c.y() + k)
        p.strokePath(cross, D.pen(_op(col, 0.6) if on else Theme.c("body", 0.3), 1))
        self.hits.add(outer, lambda: self.select(g), M.GROUP[g].title)

    def _below(self, p, g, icon_item, at, rows):
        """A small control off the controller body (motion sensor, grain slice output), with tokens below."""
        sel = self.selected == g
        col, on = self._colors(g)
        head = D.Pill(icon_item, 10, 6, _op(col, 0.16) if on else Theme.c("body", 0.035),
                      Theme.ink if sel else (_op(col, 0.7) if on else Theme.c("body", 0.4)), 2 if sel else 1.2, dash=[4, 3])
        whole = D.Col([head] + ([D.Col(rows, 2)] if rows else []), 5)
        c = gp(*at)
        w, h = whole.size()
        x, y = c.x() - w / 2, c.y() - head.h / 2
        if sel:
            pth = QPainterPath()
            pth.addRoundedRect(QRectF(x + (w - head.w) / 2, y, head.w, head.h), head.h / 2, head.h / 2)
            D.glow(p, pth, _op(col, 0.8), 10)
        whole.paint(p, x, y)
        self.hits.add(QRectF(x, y, w, h), lambda: self.select(g), M.GROUP[g].title)

    def _motion(self, p):
        g = "motion"
        sel = self.selected == g
        col, on = self._colors(g)
        fg = Theme.ink if sel else (_op(col, 0.9) if on else Theme.c("body", 0.6))
        icon = D.Custom(16, 16, lambda pp, r: D.symbol(pp, "gyroscope", r.center(), 14, fg, 1.3))
        rows = [token_row(self.m, i, True, rec=self._pills) for i in M.GROUP[g].inputs if self.m.assigned(i)]
        self._below(p, g, icon, (352, 470), rows)

    def _center_tokens(self, p):
        rows = []
        for g in ("ps", "mic"):
            i = M.GROUP[g].inputs[0]
            if not self.m.assigned(i):
                continue
            lab = D.Text("PS" if g == "ps" else "MIC", Theme.font(8, HEAVY, "mono"), Theme.ink if self.selected == g else Theme.dim)
            rows.append((g, D.Row([lab, token_row(self.m, i, True, False, rec=self._pills)], 5)))
        if not rows:
            return
        col = D.Col([r for _, r in rows], 3)
        c = gp(450, 372)
        w, h = col.size()
        x, y = c.x() - w / 2, c.y() - 8
        col.paint(p, x, y)
        cy = y
        for g, r in rows:
            rw, rh = r.size()
            self.hits.add(QRectF(x + (w - rw) / 2, cy, rw, rh), lambda g=g: self.select(g))
            cy += rh + 3

    # ── callouts: only for controls with something assigned ──
    def _callouts(self, p):
        for g, anchor, side, y in CALLOUTS:
            if not self.mapped(g):
                continue
            a = gp(*anchor)
            col, _ = self._colors(g)
            sel = self.selected == g
            line = QPainterPath()
            if side == "left":
                line.moveTo(214, y); line.lineTo(244, y); line.lineTo(a)
            elif side == "right":
                line.moveTo(886, y); line.lineTo(856, y); line.lineTo(a)
            else:
                line.moveTo(a.x(), y + 26); line.lineTo(a)
            p.strokePath(line, D.pen(_op(Theme.ink, 0.8) if sel else _op(col, 0.55), 1.4 if sel else 1))
            D.circle(p, a, 5, Theme.ink if sel else _op(col, 0.9))
            item = self._label(g, sel, side)
            w, h = item.size()
            x = 210 - w if side == "left" else (890 if side == "right" else a.x() - w / 2)
            item.paint(p, x, y - h / 2)
            self.hits.add(QRectF(x, y - h / 2, w, h), lambda g=g: self.select(g))

    def _label(self, g, sel, side) -> D.Item:
        group = M.GROUP[g]
        align = "trailing" if side == "left" else ("leading" if side == "right" else "center")
        items = [D.Text(group.title.upper(), Theme.font(10, HEAVY, "mono"), Theme.ink if sel else _op(Theme.ink, 0.85), tracking=1.2)]
        ins = [i for i in group.inputs if self.m.assigned(i)]
        rec = self._pills
        if len(group.inputs) == 1:
            items.append(token_row(self.m, ins[0], False, False, rec=rec))
        elif g == "touchpad":
            items.append(D.Row([token_row(self.m, i, rec=rec) for i in ins], 14))
        else:
            items += [token_row(self.m, i, rec=rec) for i in ins]
        ns = [n for n in (self.ctx.notes.note(NotesStore.ds(i.id)) for i in ins) if n]
        if ns:
            items.append(D.note_badge(" / ".join(ns), 9.5, 480 if g == "touchpad" else 190))
        return D.Col(items, 3, align)


def _hline(x0, x1, y) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(x0, y)
    path.lineTo(x1, y)
    return path


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
        col.addWidget(input_editor(ctx, i, len(group.inputs) > 1))
        key = NotesStore.ds(i.id)
        col.addWidget(W.NoteField(ctx.notes.raw(key), lambda t, k=key: ctx.set_note(k, t),
                                  f"Note · {i.label}" if len(group.inputs) > 1 else "Note"))


def _accent(m: M.Mapping, i: M.InputDef) -> QColor:
    t = m.shown(i)
    return _kind_color(t[0][1]) if t else Theme.dim


def _group_label(key: str) -> str:
    if key == "none":
        return ""
    if key.startswith("er_"):
        return "ER-301"
    if key == "midi":
        return "MIDI"
    if key in ("slice_fine", "slice_section", "slice_block", "block_up", "block_down", "block_reset", "section_up", "section_down"):
        return "Grain slice"
    if key.startswith("loop_") or key in ("quantize", "quantize_grid"):
        return "Loop"
    return "Other"


def function_picker(ctx, i: M.InputDef) -> W.Picker:
    m = ctx.mapping
    opts = [(k, m.func_title(k)) for k in m.choices(i)]

    def pick(k):
        m.set_choice(i, k)
        ctx.changed(rebuild=True)

    return W.Picker(opts, m.choice(i), pick, 230)


def input_editor(ctx, i: M.InputDef, multi: bool) -> QWidget:
    m = ctx.mapping
    key = m.choice(i)
    title = i.label if multi else ("Button" if i.button else "Control")
    card = W.Card(title, _accent(m, i))
    card.add(W.row("Tap" if i.button else "Does", function_picker(ctx, i), SEMIBOLD))
    role = m.effective_role(i)
    if M.role_uses_port(role):
        def set_port(v):
            m.ports[i.id] = v
            ctx.changed()
        card.add(W.row("SC.CV port" if role == M.CV else f"{m.trigger_port_name} port", W.NumberField(m.port(i), 1, 99, set_port)))
    card.add(W.hint(_explain(m, i, key)))
    if i.button:
        b = i.button
        f = m.fn(b)

        def set_hold(a, b=b):
            m.set_hold(a, b)
            ctx.changed(rebuild=True)

        card.add(W.divider())
        card.add(W.row("Hold", W.Picker([(a, M.ACTION_TITLE[a]) for a in M.HOLD_CHOICES], f.hold, set_hold, 230), SEMIBOLD))
        if f.hold != "none":
            card.add(W.hint(f"Shorter than {'%g' % m.long_press} s = tap, longer = hold; decided when you let go."))
        combos = [c for c in m.chords if c.a == b or c.b == b]
        if combos:
            card.add(W.label("\n".join(f"Combo {M.button_label(c.a)} + {M.button_label(c.b)} → {M.ACTION_TITLE[c.action]}"
                                       for c in combos), 11, REGULAR, "ui", _op(Theme.function, 0.85), wrap=True))
    if key != "none" and (m.sends_midi(i) or i.kind == "nrpn"):
        n = m.number(i)
        name = f" · {i.name}" if i.name else ""
        src = {"cc": f"MIDI ch {m.channel} · CC {n}{name}", "nrpn": f"MIDI ch {m.channel} · NRPN {n} (14-bit){name}"}.get(
            i.kind, f"MIDI ch {m.channel} · note {n}")
        card.add(W.label(src, 10, REGULAR, "mono", _op(Theme.dim, 0.8)))
    return card


def _explain(m: M.Mapping, i: M.InputDef, key: str) -> str:
    step = M.AXIS_OF.get(i.id) in M.STEP_AXES
    if key == "none":
        return "Does nothing. Pick a function to give it one; it then shows up on the drawing."
    if key in M.ER_ROLE and key != "midi":
        return _explanation(m, i, M.ER_ROLE[key])
    if key == "midi":
        return "Sends its own MIDI (see below) without routing it to the ER-301 — for other MIDI gear or loop recording."
    if key == "slice_fine":
        return "Picks the slice (0–15) inside the current section of the grain slice."
    if key == "slice_section":
        return "Steps one section (of 16 slices) down / up." if step else "Picks the section of 16 slices (0–7) inside the block."
    if key == "slice_block":
        return "Steps one block (of 128 slices) down / up." if step else "Picks the block of 128 slices (0–7)."
    if key == "loop_scrub":
        return "Moves the playback position of the playing loop."
    if key == "quantize_grid":
        return "Picks the loop's quantize subdivision: 1/32 · 1/16 · 1/8 · 1/4 from low to high."
    return _timing(m, M.ButtonFunc(key, "none")) + " " + M.ACTION_TITLE.get(key, "") + "."


def _timing(m: M.Mapping, f: M.ButtonFunc) -> str:
    if f.tap == "midi":
        return "Sends its CC: 127 while pressed, 0 on release."
    if f.tap == "none":
        return "Does nothing."
    return "Fires when you let go:" if M.fires_on_release(f.tap) else "Fires when you press:"


def _explanation(m: M.Mapping, i: M.InputDef, r: str) -> str:
    if r == M.OFF:
        return "Not sent to the ER-301."
    if r == M.CV:
        if i.button:
            return "10 V while pressed, 0 V when released."
        return "0–16383 → 0–10 V in full resolution." if i.kind == "nrpn" else "0–127 → 0–10 V."
    if r == M.TRIGGER:
        gate = "while pressed" if i.button else "while the value is 64 or more"
        return f"Trigger high {gate} (on an SC.CV port as 10 V)." if m.triggers_as_cv else f"SC.TR high {gate}."
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
    card = W.Card("Assigned", Theme.function)
    any_ = False
    for g in M.GROUPS:
        for i in g.inputs:
            if not m.assigned(i):
                continue
            any_ = True
            w = QWidget()
            from PySide6.QtWidgets import QHBoxLayout
            h = QHBoxLayout(w)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(6)
            name = M.input_title(i) if len(g.inputs) > 1 else g.title
            ll = W.label(name, 12, BOLD, "ui", wrap=True)
            ll.setFixedWidth(int(130 * Theme.grow))
            h.addWidget(ll, 0, Qt.AlignTop)
            text = "  ·  ".join(t for t, _ in m.shown(i))
            h.addWidget(W.label(text, 11, SEMIBOLD, "mono", _accent(m, i), wrap=True), 1)
            w.setCursor(Qt.PointingHandCursor)
            w.mousePressEvent = lambda ev, g=g.id: ctx.select_ds(g)
            card.add(w)
    if not any_:
        card.add(W.hint("Nothing assigned yet: click a control on the drawing."))
    midi = [(M.input_title(i) if len(g.inputs) > 1 else g.title) + " " + m.tokens(i)[0][0]
            for g in M.GROUPS for i in g.inputs if m.midi_only(i)]
    if midi:
        card.add(W.divider())
        card.add(W.label("MIDI ONLY", 9, HEAVY, "mono", Theme.dim, tracking=1))
        card.add(W.hint(" · ".join(midi) + ". Sent to the MIDI hub, not to the ER-301 — shown only here."))
    card.add(W.hint(f"Hold = {'%g' % m.long_press} s or longer (LONG_PRESS_DURATION in mappings.py)."))
    col.addWidget(card)
