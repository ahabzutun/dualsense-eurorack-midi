"""Pedals tab: the MPC-20 drawing and its inspector (Pedal / Device)."""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath
from PySide6.QtWidgets import QPushButton, QVBoxLayout, QWidget

from .. import draw as D
from .. import widgets as W
from ..devices import (PEDAL_LABEL, PEDAL_SHORT, hub_channel, pedal_choices, pedal_has_number, pedal_has_range,
                       pedal_max, pedal_name)
from ..stage import Drawing
from ..store import NotesStore
from ..theme import BOLD, HEAVY, MEDIUM, REGULAR, SEMIBOLD, Theme
from .common import pi_card, pi_pill, route_card, route_pill
from .dualsense import _op

TYPE_NAME = {"expression": "Expression pedal", "switch": "Switch pedal", "hihat": "Hi-hat pedal"}


class PedalDrawing(Drawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx

    # tiles grow with the text size; the jacks spread out to make room
    @property
    def tile_w(self) -> float:
        return 190 * Theme.grow

    @property
    def spacing(self) -> float:
        return max(230.0, self.tile_w + 16)

    @property
    def base(self):
        return (max(1100.0, 3 * self.spacing + self.tile_w + 80), 640.0)

    def jack_x(self, i: int, w: float) -> float:
        return w / 2 + (i - 1.5) * self.spacing

    def paint(self, p: QPainter, w, h):
        ctx = self.ctx
        board = ctx.pedals
        sel = ctx.pedal_sel
        cx = w / 2
        head = f"DOREMiDi {board.device} · firmware {board.firmware}" if board.loaded else "DOREMiDi MPC-20 · not loaded"
        D.Text(head, Theme.font(11, HEAVY, "mono"), Theme.dim, tracking=1.2).paint_center(p, cx, 40)
        # the box
        bw = max(900.0, 3 * self.spacing + 210)
        box = QRectF(cx - bw / 2, 150 - 60, bw, 120)
        path = QPainterPath()
        path.addRoundedRect(box, 18, 18)
        p.fillPath(path, D.vgradient(box, Theme.c("body", 0.12), Theme.c("body", 0.04)))
        p.strokePath(path, D.pen(Theme.c("body", 0.6), 1.6))
        D.Row([D.Text("MPC-20", Theme.font(22, HEAVY, "title"), Theme.ink),
               D.Text("pedal → MIDI", Theme.font(11, SEMIBOLD, "mono"), Theme.dim)], 10).paint_center(p, cx, 150 - 28)
        for i in range(4):
            x = self.jack_x(i, w)
            jc = QPointF(x, 150 + 22 - 9)
            D.circle(p, jc, 26, Theme.bg, Theme.ink if sel == i else Theme.c("body", 0.8), 2)
            D.circle(p, jc, 8, Theme.c("body", 0.5))
            lab = D.Text(f"EXP{i + 1}", Theme.font(10, HEAVY, "mono"), Theme.ink if sel == i else Theme.dim)
            lab.paint_center(p, x, jc.y() + 13 + 5 + lab.h / 2)
            self.hits.add(QRectF(x - 24, jc.y() - 15, 48, 50), lambda i=i: ctx.select("pedal_sel", i, "Pedal"))
            # cable
            used = board.inputs[i]["type_id"] != 0
            cable = QPainterPath()
            cable.moveTo(x, 211)
            cable.cubicTo(x + 30, 245, x - 30, 280, x, 318)
            col = (_op(Theme.ink, 0.8) if sel == i else Theme.c("cv", 0.7)) if used else Theme.c("body", 0.18)
            p.strokePath(cable, D.pen(col, 3 if used else 1.4, None if used else [4, 5]))
            # tile
            tile = self._tile(i)
            tw, th = tile.size()
            tile.paint(p, x - tw / 2, 430 - th / 2)

    def _tile(self, i: int) -> D.Item:
        ctx = self.ctx
        inp = ctx.pedals.inputs[i]
        sel = ctx.pedal_sel == i
        active = [s for s in inp["slots"] if s["on"]]
        t = inp["type"]
        col = Theme.trig if t == "switch" else (Theme.function if t == "hihat" else Theme.cv)
        tw = self.tile_w
        icon = D.Custom(120, 92, lambda p, r: pedal_icon(p, r, t, col, sel))
        name = D.Text("nothing plugged in" if inp["type_id"] == 0 else TYPE_NAME.get(t, t), Theme.font(11, BOLD, "mono"),
                      Theme.dim if inp["type_id"] == 0 else Theme.ink, max_w=tw - 24, min_scale=0.75)
        rows = []
        for s in active[:5]:
            k = s["kind"]
            pill = route_pill(ctx.mapping, hub_channel(s["channel"], ctx.mapping), s["number"], k == "note", 10) \
                if k in ("cc", "note") else None
            f = Theme.font(10, SEMIBOLD, "mono")
            items = [D.Text(f"ch{s['channel']}", f, Theme.dim),
                     D.Text(f"{PEDAL_SHORT.get(k, k)}{s['number']}" if pedal_has_number(k) else PEDAL_SHORT.get(k, k), f, Theme.ink)]
            if pill is not None:
                items.append(pill)
            pp = pi_pill(ctx.mapping, "pedal", s["channel"], k, s["number"], 10) if k in ("cc", "note") else None
            if pp is not None:
                items.append(pp)
            n = ctx.notes.note(NotesStore.pedal(i, s["slot"]))
            if n:
                items.append(D.note_badge(n, 9, 80 * Theme.grow))
            rows.append(D.Row(items, 5))
        if len(active) > 5:
            rows.append(D.Text(f"+{len(active) - 5} more", Theme.font(10, REGULAR, "mono"), Theme.dim))
        if inp["type_id"] != 0 and not active:
            rows.append(D.Text("no messages", Theme.font(10, REGULAR, "mono"), Theme.dim))
        msgs = D.Box(D.Col(rows, 3), w=tw - 24, min_h=110, valign="top")
        content = D.Box(D.Col([icon, name, msgs], 10), w=tw, pad=12)

        def bg(p, r):
            if sel:
                path = QPainterPath()
                path.addRoundedRect(r, 12, 12)
                D.glow(p, path, _op(col, 0.5), 10)
                D.draw_path(p, path, _op(col, 0.10), _op(Theme.ink, 0.7), 1.2)

        return D.Decor(content, before=bg, hits=self.hits, on_click=lambda: ctx.select("pedal_sel", i, "Pedal"))


def pedal_icon(p: QPainter, r: QRectF, t: str, color: QColor, selected: bool):
    x0, y0, w, h = r.left(), r.top(), r.width(), r.height()
    stroke = Theme.ink if selected else color

    def R(a, b, c, d):
        return QRectF(x0 + a, y0 + b, c, d)

    if t in ("expression", "hihat"):
        D.rounded(p, R(w * 0.08, h * 0.72, w * 0.84, h * 0.2), 5, _op(color, 0.18), stroke, 1.5)
        tread = QPainterPath()
        tread.moveTo(x0 + w * 0.12, y0 + h * 0.70)
        tread.lineTo(x0 + w * 0.86, y0 + h * 0.30)
        tread.lineTo(x0 + w * 0.92, y0 + h * 0.40)
        tread.lineTo(x0 + w * 0.18, y0 + h * 0.78)
        tread.closeSubpath()
        D.draw_path(p, tread, _op(color, 0.30), stroke, 1.5)
        piv = QPainterPath()
        piv.addEllipse(R(w * 0.47, h * 0.52, 9, 9))
        p.fillPath(piv, stroke)
        if t == "hihat":
            cy = QPainterPath()
            cy.addEllipse(R(w * 0.62, h * 0.02, w * 0.3, h * 0.12))
            p.strokePath(cy, D.pen(stroke, 1.3))
    elif t == "switch":
        D.rounded(p, R(w * 0.2, h * 0.30, w * 0.6, h * 0.58), 8, _op(color, 0.18), stroke, 1.5)
        k = QPainterPath()
        k.addEllipse(R(w * 0.38, h * 0.42, w * 0.24, w * 0.24))
        D.draw_path(p, k, _op(color, 0.4), stroke, 1.5)
    else:
        D.rounded(p, R(w * 0.15, h * 0.3, w * 0.7, h * 0.55), 8, None, Theme.c("body", 0.35), 1.2, [4, 4])


# ── inspector ───────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    if tab == "Pedal":
        editor(ctx, col)
    else:
        device(ctx, col)


def device(ctx, col: QVBoxLayout):
    board = ctx.pedals
    col.addWidget(W.title("MPC-20"))
    col.addWidget(W.hint(f"DOREMiDi {board.device}, firmware {board.firmware}, connected via {board.port}." if board.loaded
                         else "Not loaded yet. Use “Load”."))
    card = W.Card("Jacks")
    for i in range(4):
        inp = board.inputs[i]
        nm = W.label(f"EXP{i + 1}", 12, BOLD, "mono")
        nm.setFixedWidth(int(52 * Theme.grow))
        card.add(W.hbox(nm, W.label("nothing plugged in" if inp["type_id"] == 0 else inp["type"], 12, REGULAR, "ui",
                                    Theme.dim if inp["type_id"] == 0 else Theme.ink), "stretch",
                        W.label(f"{sum(1 for s in inp['slots'] if s['on'])} messages", 11, REGULAR, "mono", Theme.dim)))
    card.add(W.hint("The box detects the pedal type by itself. Settings are stored per jack and pedal type, and kept when it's unplugged."))
    col.addWidget(card)
    card = W.Card("Backup")
    card.add(W.hint("Save Configuration (Ctrl+Shift+S, Configs menu) includes the pedal settings. The SSH host also has a raw backup: scripts/mpc20_backup_20261111.json (mpc20_tool.py restore)."))
    col.addWidget(card)


def editor(ctx, col: QVBoxLayout):
    board = ctx.pedals
    sel = ctx.pedal_sel
    inp = board.inputs[sel]
    col.addWidget(W.title(f"EXP{sel + 1}"))
    if not board.loaded:
        col.addWidget(W.hint("Not loaded from the MPC-20 yet. Use “Load” first."))
        return
    if inp["type_id"] == 0:
        col.addWidget(W.hint("Nothing is plugged into this jack. The MPC-20 recognises the pedal type by itself: plug the pedal in, then press “Load”."))
        return
    line = {"expression": "Expression pedal (detected by the box).", "switch": "Switch pedal (detected by the box).",
            "hihat": "Hi-hat pedal (detected by the box)."}.get(inp["type"], "")
    col.addWidget(W.hint(f"{line} All messages that are switched on are sent together."))
    first_off = next((k for k, s in enumerate(inp["slots"]) if not s["on"]), None)
    for s in range(8):
        if inp["slots"][s]["on"] or s == first_off:
            col.addWidget(slot_card(ctx, sel, s))
    col.addWidget(W.hint("Start = released, end = pressed. Curve, gain and trigger settings stay as they are on the box."
                         if inp["type"] == "switch" else
                         "Start = heel, end = toe (swap them to invert). Curve, gain and trigger settings stay as they are on the box."))


def slot_card(ctx, input_i: int, slot: int):
    board, m = ctx.pedals, ctx.mapping
    inp = board.inputs[input_i]
    s = inp["slots"][slot]
    t = inp["type"]
    if not s["on"]:
        def add():
            used = {x["number"] for i in board.inputs for x in i["slots"] if x["on"] and x["kind"] == "cc"}
            last = max([x["number"] for x in inp["slots"] if x["on"] and x["kind"] == "cc"] or [101])
            c = last + 1
            while c in used and c < 127:
                c += 1
            s.update(on=True, kind="cc", channel=next((x["channel"] for x in inp["slots"] if x["on"]), 1),
                     number=min(c, 127), start=0, end=127)
            ctx.changed(True)

        b = QPushButton("＋  Add a message")
        b.setObjectName("small")
        b.setCursor(Qt.PointingHandCursor)
        b.clicked.connect(add)
        return W.hbox(b, stretch_end=True)

    kind = s["kind"]
    card = W.Card(f"Message {slot + 1}", Theme.cv)

    def set_kind(k):
        if s["kind"] in ("cc", "note") and k != s["kind"]:
            m.set_ext_route(hub_channel(s["channel"], m), s["number"], None, note=s["kind"] == "note")
            m.drop_pi("pedal", s["channel"], s["kind"], s["number"])
        s["kind"] = k
        if k == "pitchbend":
            s["start"], s["end"] = 0, 16383
        else:
            s["start"], s["end"] = min(s["start"], 127), min(s["end"], 127)
            if s["start"] == s["end"]:
                s["end"] = 127
        ctx.changed(True)

    def remove():
        if s["kind"] in ("cc", "note"):
            m.set_ext_route(hub_channel(s["channel"], m), s["number"], None, note=s["kind"] == "note")
            m.drop_pi("pedal", s["channel"], s["kind"], s["number"])
        s["on"] = False
        ctx.changed(True)

    card.add(W.hbox(W.Picker([(k, PEDAL_LABEL[k]) for k in pedal_choices(t)], kind, set_kind, 140), "stretch",
                    W.icon_button("✕", remove, "Switch this message off")))
    key = NotesStore.pedal(input_i, slot)
    card.add(W.NoteField(ctx.notes.raw(key), lambda x: ctx.set_note(key, x)))

    def move(channel, number):
        """Changing channel / CC moves an existing ER-301 route along with it."""
        if s["kind"] in ("cc", "note"):
            m.move_ext_route((hub_channel(s["channel"], m), s["number"]), (hub_channel(channel, m), number), s["kind"] == "note")
            m.move_pi("pedal", s["kind"], (s["channel"], s["number"]), (channel, number))
        s["channel"], s["number"] = channel, number
        ctx.changed()

    card.add(W.row("Channel", W.NumberField(s["channel"], 1, 16, lambda v: move(v, s["number"]))))
    if pedal_has_number(kind):
        card.add(W.row("Note" if kind == "note" else ("Program" if kind == "pc" else "CC"),
                       W.NumberField(s["number"], 0, 127, lambda v: move(s["channel"], v))))
    if pedal_has_range(kind):
        wd = 60 if kind == "pitchbend" else 44
        a = W.NumberField(s["start"], 0, pedal_max(kind), lambda v: (s.__setitem__("start", v), ctx.changed()), wd)
        b = W.NumberField(s["end"], 0, pedal_max(kind), lambda v: (s.__setitem__("end", v), ctx.changed()), wd)
        card.add(W.row("Released → pressed" if t == "switch" else "Heel → toe",
                       W.hbox(a, W.label("→", 12, REGULAR, "ui", Theme.dim), b, spacing=4)))
    if t == "switch":
        opts = [(0, "Momentary"), (1, "Toggle")] + ([(s["mode"], f"Other ({s['mode']})")] if s["mode"] > 1 else [])
        card.add(W.row("Mode", W.Picker(opts, s["mode"], lambda v: (s.__setitem__("mode", v), ctx.changed()), 130)))
    if kind in ("cc", "note"):
        hint = (f"The MIDI hub moves pedal messages on channel 1 to the DualSense channel ({m.channel}); the route follows that."
                if s["channel"] == 1 else f"Routed on channel {s['channel']}.")
        if t == "switch" and kind == "cc":
            hint += " A switch makes a good trigger: set released → pressed to 0 → 127."
        wrap = QWidget()
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(8)
        v.addWidget(card)
        v.addWidget(route_card(ctx, lambda: (hub_channel(s["channel"], m), s["number"]),
                               hint, note=kind == "note", title=f"Message {slot + 1} → ER-301"))
        v.addWidget(pi_card(ctx, "pedal", lambda: (s["channel"], s["kind"], s["number"])))
        return wrap
    return card
