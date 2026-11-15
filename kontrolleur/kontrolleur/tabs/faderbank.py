"""16n tab: the faderbank drawing and its inspector (Fader / Device)."""
from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath
from PySide6.QtWidgets import QSlider, QVBoxLayout, QWidget

from .. import draw as D
from .. import widgets as W
from ..devices import CURVE_PRESETS, RANGE_PRESETS, curve_name, fader_out, full_range, range_text
from ..stage import GrowingDrawing
from ..store import NotesStore
from ..theme import BOLD, HEAVY, MEDIUM, REGULAR, SEMIBOLD, Theme
from .common import pi_card, pi_pill, route_card, route_pill
from .dualsense import _op


def shaped(f: dict) -> bool:
    return f["curve"] != 64 or not full_range(f)


def curve_preview(p: QPainter, r: QRectF, f: dict, color: QColor, frame_alpha: float = 0.2, labels: bool = False):
    """What the fader sends over its travel: its curve inside its range (dashed lines: min and max)."""
    fr = QPainterPath()
    fr.addRect(r)
    p.strokePath(fr, D.pen(Theme.c("body", frame_alpha), 0.6))
    if not full_range(f):
        for v in (f["range_min"], f["range_max"]):
            y = r.top() + (1 - v / 127) * r.height()
            p.strokePath(_hline(r.left(), r.right(), y), D.pen(_op(color, 0.45), 0.8, [3, 3]))
            if labels:
                t = D.Text(str(v), Theme.font(9, BOLD, "mono"), _op(color, 0.9))
                t.paint(p, r.left() + 4, min(max(r.top() + 1, y - t.h - 1), r.bottom() - t.h - 1))
    path = QPainterPath()
    for s in range(41):
        x = s / 40
        pt = QPointF(r.left() + x * r.width(), r.top() + (1 - fader_out(x, f)) * r.height())
        path.moveTo(pt) if s == 0 else path.lineTo(pt)
    p.strokePath(path, D.pen(color, 1.6))


def _hline(x1: float, x2: float, y: float) -> QPainterPath:
    path = QPainterPath()
    path.moveTo(x1, y)
    path.lineTo(x2, y)
    return path


class FaderbankDrawing(GrowingDrawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx

    def content(self):
        bank = self.ctx.bank
        head = D.Text(f"16n · {bank.device} · {bank.firmware_text}"
                      if bank.loaded else "16n · not loaded", Theme.font(11, HEAVY, "mono"), Theme.dim, tracking=1.2)
        cols = D.Row([self._column(i) for i in range(16)], 4)
        body = D.Box(cols, min_w=1000, min_h=520, pad=12)
        body = D.Decor(body, before=_body)
        return D.Col([head, body], 10)

    def _column(self, i: int) -> D.Item:
        ctx = self.ctx
        f = ctx.bank.faders[i]
        sel = ctx.fader_sel == i
        pill = route_pill(ctx.mapping, f["usb_channel"], f["usb_cc"], max_w=56 * Theme.grow)
        col = Theme.function if shaped(f) else Theme.cv
        g = Theme.grow
        w = 56 * g
        items = [
            D.Text(str(i + 1), Theme.font(13, HEAVY, "mono"), Theme.ink if sel else _op(Theme.ink, 0.8)),
            D.Custom(44, 34, lambda p, r, f=f, cc=col: curve_preview(p, r, f, cc)),
            D.Custom(40, 256, lambda p, r, s=sel, cc=col: _slider(p, r, s, cc)),
        ]
        small = [
            D.Text(f"ch{f['usb_channel']}", Theme.font(10, SEMIBOLD, "mono"), Theme.dim, max_w=w, min_scale=0.7),
            D.Text(f"CC{f['usb_cc']}{'ʰ' if f['usb_14bit'] else ''}", Theme.font(11, BOLD, "mono"), Theme.ink, max_w=w, min_scale=0.7),
        ]
        if not full_range(f):
            small.append(D.Text(range_text(f), Theme.font(10, BOLD, "mono"), Theme.function, max_w=w, min_scale=0.7))
        pp = pi_pill(ctx.mapping, "16n", f["usb_channel"], "cc", f["usb_cc"], 9.5, w)
        if pill is not None:
            small.append(pill)
        elif pp is None:
            small.append(D.Text("—", Theme.font(10, BOLD, "mono"), Theme.dim))
        if pp is not None:
            small.append(pp)
        n = ctx.notes.note(NotesStore.fader(i))
        if n:
            small.append(D.note_badge(n, 8.5, w, 2))
        items.append(D.Col(small, 3))
        inner = D.Box(D.Col(items, 8), w=w, pad=0)
        inner = D.Box(inner, w=w, h=inner.size()[1] + 24)

        def bg(p, r, sel=sel, col=col):
            if sel:
                path = QPainterPath()
                path.addRoundedRect(r, 10, 10)
                D.glow(p, path, _op(col, 0.6), 10)
                D.draw_path(p, path, _op(col, 0.10), _op(Theme.ink, 0.7), 1.2)

        return D.Decor(inner, before=bg, hits=self.hits, on_click=lambda: ctx.select("fader_sel", i, "Fader"))


def _body(p, r):
    path = QPainterPath()
    path.addRoundedRect(r, 22, 22)
    p.fillPath(path, D.vgradient(r, Theme.c("body", 0.10), Theme.c("body", 0.03)))
    p.strokePath(path, D.pen(Theme.c("body", 0.55), 1.6))


def _slider(p: QPainter, r: QRectF, sel: bool, col: QColor):
    c = r.center()
    track = QRectF(c.x() - 4, c.y() - 125, 8, 250)
    D.rounded(p, track, 4, Theme.c("bg", 0.8), Theme.c("body", 0.35), 1)
    knob = QRectF(c.x() - 18, c.y() + 40 - 11, 36, 22)
    D.rounded(p, knob, 4, _op(Theme.ink, 0.9) if sel else _op(col, 0.35), col, 1.4)
    p.fillRect(QRectF(knob.left() + 0.7, knob.center().y() - 1, knob.width() - 1.4, 2), Theme.bg)


# ── inspector ───────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    if tab == "Fader":
        fader_editor(ctx, col)
    else:
        device_editor(ctx, col)


def set_usb(ctx, k: int, ch: int, cc: int):
    """Changing the USB channel / CC moves an existing ER-301 route along with it."""
    bank, m = ctx.bank, ctx.mapping
    old = bank.faders[k]
    m.move_ext_route((old["usb_channel"], old["usb_cc"]), (ch, cc))
    m.move_pi("16n", "cc", (old["usb_channel"], old["usb_cc"]), (ch, cc))
    bank.faders[k]["usb_channel"] = ch
    bank.faders[k]["usb_cc"] = cc


def fader_editor(ctx, col: QVBoxLayout):
    bank, m = ctx.bank, ctx.mapping
    sel = ctx.fader_sel
    f = bank.faders[sel]
    col.addWidget(W.title(f"Fader {sel + 1}"))
    if not bank.loaded:
        col.addWidget(W.hint("Not loaded from the 16n yet: these are defaults. Use “Load” first."))
    key = NotesStore.fader(sel)
    col.addWidget(W.NoteField(ctx.notes.raw(key), lambda t: ctx.set_note(key, t)))

    def setf(k, v, rebuild=False):
        bank.faders[sel][k] = v
        ctx.changed(rebuild)

    card = W.Card("Curve", Theme.function)
    prev = QWidget()
    prev.setFixedHeight(120)
    prev.paintEvent = lambda ev, w=prev: _paint_curve(w, bank.faders[sel])
    card.add(prev)
    name = W.label(curve_name(f["curve"]), 12, SEMIBOLD, "mono")

    def set_curve(v):
        if bank.ranges_supported:
            v &= 0x7E                     # the fork with ranges keeps curves in steps of 2
        bank.faders[sel]["curve"] = v
        name.setText(curve_name(v))
        prev.update()
        if num.value() != v:
            num.blockSignals(True); num.setValue(v); num.blockSignals(False)
        if sl.value() != v:
            sl.blockSignals(True); sl.setValue(v); sl.blockSignals(False)
        ctx.changed()

    num = W.NumberField(f["curve"], 0, 127, set_curve)
    card.add(W.hbox(name, "stretch", num))
    sl = QSlider(Qt.Horizontal)
    sl.setRange(0, 127)
    sl.setValue(f["curve"])
    sl.valueChanged.connect(set_curve)
    card.add(sl)
    card.add(W.hbox(*[W.small_button(t, lambda v=v: set_curve(v)) for t, v in CURVE_PRESETS], stretch_end=True))

    def all_curve():
        for k in range(16):
            bank.faders[k]["curve"] = bank.faders[sel]["curve"]
        ctx.changed()

    card.add(W.hbox(W.small_button("Use this curve on all 16 faders", all_curve), stretch_end=True))
    card.add(W.hint("Applied inside the 16n to USB, TRS MIDI and I2C. The CV jacks stay linear (they are wired straight to the faders)."
                    if bank.curves_supported or not bank.loaded else "⚠︎ This 16n runs firmware without curves: curves are not uploaded."))
    col.addWidget(card)
    col.addWidget(range_card(ctx, sel, prev))

    card = W.Card("USB MIDI", Theme.cv)
    card.add(W.row("Channel", W.NumberField(f["usb_channel"], 1, 16, lambda v: (set_usb(ctx, sel, v, bank.faders[sel]["usb_cc"]), ctx.changed()))))
    card.add(W.row("CC", W.NumberField(f["usb_cc"], 0, 127, lambda v: (set_usb(ctx, sel, bank.faders[sel]["usb_channel"], v), ctx.changed()))))
    card.add(W.switch_row("14-bit (LSB on CC + 32)", f["usb_14bit"], lambda v: setf("usb_14bit", v), bank.hires_supported or not bank.loaded))

    def ch_all():
        ch = bank.faders[sel]["usb_channel"]
        for k in range(16):
            set_usb(ctx, k, ch, bank.faders[k]["usb_cc"])
        ctx.changed(True)

    def cc_up():
        base = bank.faders[sel]["usb_cc"] - sel
        for k in range(16):
            set_usb(ctx, k, bank.faders[k]["usb_channel"], min(127, base + k))
        ctx.changed(True)

    base = f["usb_cc"] - sel
    card.add(W.hbox(W.small_button(f"Channel {f['usb_channel']} on all", ch_all),
                    W.small_button("CCs ascending from here", cc_up, base >= 0 and base + 15 <= 127), stretch_end=True))
    col.addWidget(card)

    col.addWidget(route_card(ctx, lambda: (bank.faders[sel]["usb_channel"], bank.faders[sel]["usb_cc"]),
                             "Needs the 16n on SSH (its USB MIDI goes through the MIDI hub on the SSH host to the i2c2midi). "
                             "With 14-bit only the MSB is used for now."))
    col.addWidget(pi_card(ctx, "16n", lambda: (bank.faders[sel]["usb_channel"], "cc", bank.faders[sel]["usb_cc"]),
                          "Needs the 16n on SSH."))

    card = W.Card("TRS MIDI")
    card.add(W.row("Channel", W.NumberField(f["trs_channel"], 1, 16, lambda v: setf("trs_channel", v))))
    card.add(W.row("CC", W.NumberField(f["trs_cc"], 0, 127, lambda v: setf("trs_cc", v))))
    card.add(W.switch_row("14-bit (LSB on CC + 32)", f["trs_14bit"], lambda v: setf("trs_14bit", v), bank.hires_supported or not bank.loaded))
    col.addWidget(card)


def range_card(ctx, sel: int, prev: QWidget) -> W.Card:
    bank = ctx.bank
    f = bank.faders[sel]
    card = W.Card("Range", Theme.function)
    fields = {}

    def set_range(lo=None, hi=None):
        fd = bank.faders[sel]
        if lo is not None:
            fd["range_min"] = lo
        if hi is not None:
            fd["range_max"] = hi
        for key, fld in fields.items():
            if fld.value() != fd[key]:
                fld.blockSignals(True)
                fld.setValue(fd[key])
                fld.blockSignals(False)
        prev.update()
        ctx.changed()

    fields["range_min"] = W.NumberField(f["range_min"], 0, 127, lambda v: set_range(lo=v))
    fields["range_max"] = W.NumberField(f["range_max"], 0, 127, lambda v: set_range(hi=v))
    card.add(W.row("Min (fader down)", fields["range_min"]))
    card.add(W.row("Max (fader up)", fields["range_max"]))
    card.add(W.hbox(*[W.small_button(t, lambda lo=lo, hi=hi: set_range(lo, hi)) for t, lo, hi in RANGE_PRESETS],
                    stretch_end=True))

    def all_range():
        for k in range(16):
            bank.faders[k]["range_min"] = bank.faders[sel]["range_min"]
            bank.faders[k]["range_max"] = bank.faders[sel]["range_max"]
        ctx.changed()

    card.add(W.hbox(W.small_button("Use this range on all 16 faders", all_range), stretch_end=True))
    if bank.ranges_supported or not bank.loaded:
        card.add(W.hint("The fader's whole travel sends Min … Max (after the curve), on USB, TRS MIDI and I2C. "
                        "On the 7-bit scale; in 14-bit mode 127 = 16383 and each step is 129. Min above Max reverses the fader."))
    else:
        card.add(W.hint("⚠︎ This 16n's firmware has no fader ranges yet: flash the 16n fork with ranges "
                        "(firmware/16n), then load it again. Until then ranges are not uploaded."))
    return card


def _paint_curve(w: QWidget, f: dict):
    p = QPainter(w)
    p.setRenderHint(QPainter.Antialiasing)
    curve_preview(p, QRectF(0.5, 0.5, w.width() - 1, w.height() - 1), f, Theme.function if shaped(f) else Theme.cv,
                  labels=True)
    p.end()


def device_editor(ctx, col: QVBoxLayout):
    bank = ctx.bank
    o = bank.options
    col.addWidget(W.title("16n"))
    col.addWidget(W.hint(f"{bank.device}, {bank.firmware_text}, port “{bank.port}”."
                         if bank.loaded else "Not loaded yet."))

    def seto(k, v):
        o[k] = v
        ctx.changed()

    card = W.Card("Device")
    for k, t in (("led_on", "LED on"), ("led_flash", "LED blinks on MIDI"), ("rotate", "Rotate 180° (fader 1 on the right)"),
                 ("midi_thru", "Soft MIDI thru"), ("i2c_leader", "I2C leader (sends to TXo / ER-301 / Ansible)")):
        card.add(W.switch_row(t, o[k], lambda v, k=k: seto(k, v)))
    card.add(W.hint("I2C leader takes effect after the 16n restarts. As follower (default) Teletype reads the faders."))
    col.addWidget(card)
    card = W.Card("Calibration")
    card.add(W.row("Fader min (raw)", W.NumberField(o["fader_min"], 0, 16383, lambda v: seto("fader_min", v), 64)))
    card.add(W.row("Fader max (raw)", W.NumberField(o["fader_max"], 0, 16383, lambda v: seto("fader_max", v), 64)))
    card.add(W.hint("Raw ADC values that count as fully down / up. Only change if faders don't reach 0 or 127."))
    col.addWidget(card)
