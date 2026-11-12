"""EC4 tab: the Faderfox EC4 drawing and its inspector (Encoder / Push / Names)."""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath
from PySide6.QtWidgets import QVBoxLayout

from .. import draw as D
from .. import widgets as W
from ..devices import EC4_MODES, EC4_PUSH_TYPES, EC4_SCALES, EC4_TYPES, ec4_highres, ec4_short
from ..stage import GrowingDrawing
from ..store import NotesStore
from ..theme import BOLD, HEAVY, MEDIUM, REGULAR, SEMIBOLD, Theme
from .dualsense import _op

LCD = QColor.fromRgbF(0.55, 0.95, 0.75)


class EC4Drawing(GrowingDrawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx

    def content(self):
        ec4 = self.ctx.ec4
        if not ec4.loaded:
            return D.Col([D.Text("Faderfox EC4", Theme.font(22, HEAVY, "title"), Theme.ink),
                          D.Text("No setups received yet.", Theme.font(13, REGULAR, "mono"), Theme.dim),
                          D.Text("Press “Receive”, then on the EC4: Func → Setup → Send → “Send all setups” (hold the knob).",
                                 Theme.font(13, REGULAR, "mono"), Theme.dim)], 10)
        ctx = self.ctx
        when = f" · received {ec4.received_at.strftime('%-d %b %Y at %H:%M')}" if ec4.received_at else ""
        head = D.Text(f"EC4 · firmware data {ec4.version:.1f}{when}", Theme.font(11, HEAVY, "mono"), Theme.dim, tracking=1.2)
        su = ec4.setups[ctx.ec4_setup]
        strips = D.Grid([
            self._strip("SETUP", [s["name"] for s in ec4.setups], ctx.ec4_setup, lambda k: self._pick_setup(k)),
            self._strip("GROUP", [g["name"] for g in su["groups"]], ctx.ec4_group, lambda k: ctx.select("ec4_group", k)),
        ], 3, 6, stretch=False)
        gr = su["groups"][ctx.ec4_group]
        rows = [[self._tile(gr["encoders"][r * 4 + c], r * 4 + c) for c in range(4)] for r in range(4)]
        grid = D.Grid(rows, 6, 6, stretch=True)
        body = D.Decor(D.Box(grid, w=grid.w + 52, h=grid.h + 44), before=_body)
        return D.Col([head, strips, body], 20)

    def _pick_setup(self, k):
        self.ctx.ec4_group = 0
        self.ctx.select("ec4_setup", k)

    def _strip(self, title, names, sel, pick):
        cells = [D.Box(D.Text(title, Theme.font(9, HEAVY, "mono"), Theme.dim), align="leading", pad=0)]
        for i in range(16):
            n = names[i].strip()
            t = D.Text(n or str(i + 1), Theme.font(10, BOLD, "mono"), Theme.bg if i == sel else (Theme.dim if not n else Theme.ink))
            box = D.Box(t, w=max(52.0, t.w + 10), h=max(22.0, t.h + 8))

            def bg(p, r, on=(i == sel)):
                D.rounded(p, r, 5, Theme.cv if on else Theme.c("body", 0.10))

            cells.append(D.Decor(box, before=bg, hits=self.hits, on_click=lambda i=i: pick(i)))
        return cells

    def _tile(self, enc: dict, idx: int) -> D.Item:
        ctx = self.ctx
        sel = ctx.ec4_enc == idx
        g = Theme.grow
        port = ctx.mapping.fader_port(enc["channel"], enc["number"]) if enc["type"] == "cc" else None
        t = enc["type"]
        col = Theme.function if t.startswith("cc_rel") else (Theme.trig if t in ("note", "pc") else Theme.cv)
        name = enc["name"].strip() or "—"
        lcd = D.Text(name, Theme.font(12, HEAVY, "mono"), LCD, max_w=96 * g - 4, min_scale=0.7)
        lcd = D.Decor(D.Box(lcd, w=96 * g, h=lcd.h + 4), before=lambda p, r: D.rounded(p, r, 3, QColor(0, 0, 0, 166)))
        knob = D.Custom(52, 52, lambda p, r: _knob(p, r, sel, col))
        f = Theme.font(10, SEMIBOLD, "mono")
        num = ec4_short(t) if t in ("pitchbend", "aftertouch") else f"{ec4_short(t)}{enc['number']}"
        line = D.Row([D.Text(f"ch{enc['channel']}", f, Theme.dim), D.Text(num, f, Theme.ink)], 4)
        items = [lcd, knob, line]
        note = ctx.notes.note(NotesStore.ec4(ctx.ec4_setup, ctx.ec4_group, idx))
        push = None if enc["push"]["type"] == "off" else f"push: {enc['push']['type']}"
        if port is not None or note or push:
            extra = []
            if port is not None or not note:
                extra.append(D.Text(f"CV {port}" if port is not None else (push or ""), Theme.font(9, BOLD, "mono"),
                                    Theme.dim if port is None else Theme.cv))
            if note:
                extra.append(D.note_badge(note, 9, (126 if port is None else 80) * g))
            items.append(D.Row(extra, 5))
        content = D.Box(D.Col(items, 6), w=140 * g, pad=0)
        content = D.Box(content, w=140 * g, h=content.h + 16)

        def bg(p, r):
            if sel:
                D.rounded(p, r, 10, _op(col, 0.12), _op(Theme.ink, 0.7), 1.2)

        # the grid stretches every tile in a row to the same height; content stays at the top
        return D.Decor(content, before=bg, hits=self.hits, valign="top",
                       on_click=lambda: ctx.select("ec4_enc", idx))


def _body(p, r):
    path = QPainterPath()
    path.addRoundedRect(r, 22, 22)
    p.fillPath(path, D.vgradient(r, Theme.c("body", 0.10), Theme.c("body", 0.03)))
    p.strokePath(path, D.pen(Theme.c("body", 0.55), 1.6))


def _knob(p: QPainter, r: QRectF, sel: bool, col: QColor):
    c = r.center()
    D.circle(p, c, 52, Theme.bg, Theme.ink if sel else _op(col, 0.8), 2)
    p.fillRect(QRectF(c.x() - 1.25, c.y() - 15 - 8, 2.5, 16), Theme.ink if sel else col)


# ── inspector ───────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    if not ctx.ec4.loaded:
        col.addWidget(W.hint("Receive the setups from the EC4 first."))
        return
    if tab == "Encoder":
        encoder_editor(ctx, col)
    elif tab == "Push":
        push_editor(ctx, col)
    else:
        names_editor(ctx, col)


def _encs(ctx):
    return ctx.ec4.setups[ctx.ec4_setup]["groups"][ctx.ec4_group]["encoders"]


def move_route(ctx, k: int, ch: int, num: int):
    old = _encs(ctx)[k]
    m = ctx.mapping
    if old["type"] != "cc" or (old["channel"], old["number"]) == (ch, num):
        return
    p = m.fader_port(old["channel"], old["number"])
    if p is None:
        return
    m.set_fader_port(old["channel"], old["number"], None)
    m.set_fader_port(ch, num, p)


def set_ch(ctx, k, ch):
    move_route(ctx, k, ch, _encs(ctx)[k]["number"])
    _encs(ctx)[k]["channel"] = ch


def set_num(ctx, k, n):
    move_route(ctx, k, _encs(ctx)[k]["channel"], n)
    _encs(ctx)[k]["number"] = n


def encoder_editor(ctx, col: QVBoxLayout):
    sel = ctx.ec4_enc
    e = _encs(ctx)[sel]
    su = ctx.ec4.setups[ctx.ec4_setup]
    m = ctx.mapping
    col.addWidget(W.title(f"Encoder {sel + 1}"))
    col.addWidget(W.hint(f"Setup {ctx.ec4_setup + 1} “{su['name']}” · group {ctx.ec4_group + 1} “{su['groups'][ctx.ec4_group]['name']}”"))
    key = NotesStore.ec4(ctx.ec4_setup, ctx.ec4_group, sel)
    col.addWidget(W.NoteField(ctx.notes.raw(key), lambda t: ctx.set_note(key, t)))

    def sete(k, v, rebuild=False):
        e[k] = v
        ctx.changed(rebuild)

    card = W.Card("Turn", Theme.cv)
    card.add(W.row("Name", W.NameField(e["name"], lambda t: sete("name", t))))
    card.add(W.row("Type", W.Picker(EC4_TYPES, e["type"], lambda v: sete("type", v, True), 170)))
    card.add(W.row("Channel", W.NumberField(e["channel"], 1, 16, lambda v: (set_ch(ctx, sel, v), ctx.changed()))))
    if e["type"] == "nrpn":
        card.add(W.row("NRPN MSB / LSB", W.hbox(W.NumberField(e["number_h"], 0, 127, lambda v: sete("number_h", v)),
                                                 W.NumberField(e["number"], 0, 127, lambda v: (set_num(ctx, sel, v), ctx.changed())), spacing=4)))
    elif e["type"] not in ("pitchbend", "aftertouch"):
        card.add(W.row("Note" if e["type"] == "note" else ("Program" if e["type"] == "pc" else "CC"),
                       W.NumberField(e["number"], 0, 31 if e["type"] == "cc14" else 127, lambda v: (set_num(ctx, sel, v), ctx.changed()))))
    mx = 4095 if ec4_highres(e) else 255
    card.add(W.row("Lower → upper", W.hbox(W.NumberField(e["lower"], 0, mx, lambda v: sete("lower", v), 52),
                                           W.label("→", 12, REGULAR, "ui", Theme.dim),
                                           W.NumberField(e["upper"], 0, mx, lambda v: sete("upper", v), 52), spacing=4)))
    card.add(W.row("Display", W.Picker(EC4_SCALES, e["scale"], lambda v: sete("scale", v, True), 170)))
    card.add(W.row("Acceleration", W.Picker(EC4_MODES, e["mode"], lambda v: sete("mode", v), 170)))
    card.add(W.switch_row("Link to the next encoder", e["link"], lambda v: sete("link", v)))
    col.addWidget(card)
    if e["type"] == "cc":
        card = W.Card("Send to ER-301", Theme.cv)
        port = m.fader_port(e["channel"], e["number"])

        def route(on):
            m.set_fader_port(e["channel"], e["number"], m.first_free_port() if on else None)
            ctx.changed(True)

        card.add(W.switch_row("Route through the i2c2midi", port is not None, route))
        if port is not None:
            card.add(W.row("SC.CV port", W.NumberField(port, 1, 99, lambda v: (m.set_fader_port(e["channel"], e["number"], v), ctx.changed()))))
        card.add(W.hint("Needs the EC4 on SSH (the MIDI hub on the SSH host passes its MIDI to the i2c2midi). The route is stored with the ER-301 routing, so it applies in every setup that sends this channel + CC."))
        col.addWidget(card)

    def ch_all():
        ch = _encs(ctx)[sel]["channel"]
        for k in range(16):
            set_ch(ctx, k, ch)
        ctx.changed(True)

    def num_up():
        base = _encs(ctx)[sel]["number"] - sel
        for k in range(16):
            set_num(ctx, k, min(127, max(0, base + k)))
        ctx.changed(True)

    base = e["number"] - sel
    col.addWidget(W.hbox(W.small_button(f"Channel {e['channel']} on all 16", ch_all),
                         W.small_button("Numbers ascending from here", num_up, base >= 0 and base + 15 <= 127), stretch_end=True))


def push_editor(ctx, col: QVBoxLayout):
    sel = ctx.ec4_enc
    p = _encs(ctx)[sel]["push"]
    col.addWidget(W.title(f"Push {sel + 1}"))
    key = NotesStore.ec4_push(ctx.ec4_setup, ctx.ec4_group, sel)
    col.addWidget(W.NoteField(ctx.notes.raw(key), lambda t: ctx.set_note(key, t), "Note · push button"))

    def setp(k, v, rebuild=False):
        p[k] = v
        ctx.changed(rebuild)

    card = W.Card("Push button", Theme.trig)
    card.add(W.row("Type", W.Picker(EC4_PUSH_TYPES, p["type"], lambda v: setp("type", v, True), 170)))
    if p["type"] in ("note", "cc", "pc", "pitchbend", "aftertouch"):
        card.add(W.row("Channel", W.NumberField(p["channel"], 1, 16, lambda v: setp("channel", v))))
        if p["type"] in ("note", "cc", "pc"):
            card.add(W.row("Note" if p["type"] == "note" else ("Program" if p["type"] == "pc" else "CC"),
                           W.NumberField(p["number"], 0, 127, lambda v: setp("number", v))))
        card.add(W.row("Mode", W.Picker([("key", "Key (momentary)"), ("toggle", "Toggle")], p["mode"], lambda v: setp("mode", v), 170)))
        card.add(W.row("Released → pressed", W.hbox(W.NumberField(p["lower"], 0, 127, lambda v: setp("lower", v)),
                                                    W.label("→", 12, REGULAR, "ui", Theme.dim),
                                                    W.NumberField(p["upper"], 0, 127, lambda v: setp("upper", v)), spacing=4)))
        card.add(W.switch_row("Show on the display", p["display"], lambda v: setp("display", v)))
        card.add(W.switch_row("Link to the next push button", p["link"], lambda v: setp("link", v)))
    elif p["type"] in ("group", "setup"):
        card.add(W.row("Group" if p["type"] == "group" else "Setup",
                       W.NumberField(p["number"] + 1, 1, 16, lambda v: setp("number", max(0, min(15, v - 1))))))
    col.addWidget(card)


def names_editor(ctx, col: QVBoxLayout):
    su = ctx.ec4.setups[ctx.ec4_setup]
    col.addWidget(W.title("Names"))
    card = W.Card(f"Setup {ctx.ec4_setup + 1}")
    card.add(W.row("Setup name", W.NameField(su["name"], lambda t: (su.__setitem__("name", t), ctx.changed()))))
    col.addWidget(card)
    card = W.Card("Groups of this setup")
    for g in range(16):
        card.add(W.row(f"Group {g + 1}", W.NameField(su["groups"][g]["name"],
                                                     lambda t, g=g: (su["groups"][g].__setitem__("name", t), ctx.changed()))))
    col.addWidget(card)
    col.addWidget(W.hint("Names are 4 characters, as on the EC4's display."))
