"""Pieces the device tabs share: sending a control to the ER-301 as CV or trigger."""
from __future__ import annotations

from typing import Callable, Optional, Tuple

from PySide6.QtGui import QColor

from .. import draw as D
from .. import model as M
from .. import widgets as W
from ..theme import BOLD, Theme


def route_pill(m: M.Mapping, ch: int, num: int, note: bool = False, size: float = 10,
               max_w: Optional[float] = None) -> Optional[D.Item]:
    """'CV 4' / 'TR 7' on the drawing, or None."""
    e = m.ext_route(ch, num, note)
    if e is None:
        return None
    col = Theme.cv if e.kind == M.EXT_CV else Theme.trig
    fill = Theme.c("cv", 0.14) if e.kind == M.EXT_CV else Theme.c("trig", 0.14)
    t = D.Text(m.route_label(e), Theme.font(size, BOLD, "mono"), col, max_w=None if max_w is None else max_w - 10, min_scale=0.6)
    return D.Pill(t, 5, 1.5, fill)


def route_card(ctx, where: Callable[[], Tuple[int, int]], hint: str, note: bool = False, title: str = "Send to ER-301",
               trigger_only: bool = False) -> W.Card:
    """Off / CV / Trigger with its port. `where()` gives the (channel, number) the hub sees."""
    m = ctx.mapping
    ch, num = where()
    e = m.ext_route(ch, num, note)
    cur = "off" if e is None else ("cv" if e.kind == M.EXT_CV else "trigger")
    card = W.Card(title, Theme.trig if cur == "trigger" else Theme.cv)

    def pick(k):
        c, n = where()
        kind = None if k == "off" else (M.EXT_NOTE if note else (M.EXT_CV if k == "cv" else M.EXT_TRIGGER))
        m.set_ext_route(c, n, kind, note=note)
        ctx.changed(True)

    opts = [("off", "Off"), ("trigger", "Trigger")] if (note or trigger_only) else [("off", "Off"), ("cv", "CV"), ("trigger", "Trigger")]
    card.add(W.Segmented(opts, cur, pick))
    if e is not None:
        def set_port(v):
            c, n = where()
            m.set_ext_route(c, n, (M.EXT_NOTE if note else e.kind), v, note=note)
            ctx.changed()

        label = "SC.CV port" if e.kind == M.EXT_CV else f"{m.trigger_port_name} port"
        card.add(W.row(label, W.NumberField(e.port, 1, 99, set_port)))
        if e.kind == M.EXT_CV:
            card.add(W.hint("0–127 → 0–10 V."))
        elif note:
            card.add(W.hint("High while the note is held" + (" (10 V on an SC.CV port)." if m.triggers_as_cv else ".")))
        else:
            card.add(W.hint("High while the value is 64 or more" + (" (10 V on an SC.CV port)." if m.triggers_as_cv else ".")
                            + " Triggers go to " + m.trigger_port_name + " ports: ER-301 tab → Pitch & LFO → Output."))
    card.add(W.hint(hint))
    return card


# ── Pi functions (config/controls.py MIDI_INPUTS) ──────────────────────────────────────────────

def pi_pill(m: M.Mapping, device: str, ch: int, kind: str, num: int, size: float = 10,
            max_w: Optional[float] = None) -> Optional[D.Item]:
    """'SECTION' / 'REC' / 'LFO RATE' on the drawing when the control runs a Pi function, or None."""
    from .pi import PORT_OF, module_color, token
    f = m.pi_fn(device, ch, kind, num)
    if f == "none":
        return None
    col = module_color(PORT_OF[f][0]) if f in PORT_OF else Theme.function
    t = D.Text("π " + token(f), Theme.font(size, BOLD, "mono"), col, max_w=None if max_w is None else max_w - 10, min_scale=0.6)
    fill = QColor(col)
    fill.setAlphaF(0.14)
    return D.Pill(t, 5, 1.5, fill)


def pi_card(ctx, device: str, where: Callable[[], Tuple[int, str, int]], hint: str = "") -> W.Card:
    """Picker for the Pi function of one control. `where()` gives (channel as sent, 'cc' | 'note', number)."""
    from .pi import func_title
    m = ctx.mapping
    ch, kind, num = where()
    cur = m.pi_fn(device, ch, kind, num)
    card = W.Card("Pi function", Theme.function if cur != "none" else None)

    def pick(f):
        c, k, n = where()
        m.set_pi_fn(device, c, k, n, f)
        ctx.changed(True)

    card.add(W.Picker([(f, func_title(f)) for f in m.pi_choices(kind)], cur, pick))
    if cur in M.INPUT_ACTIONS and kind == "cc":
        card.add(W.hint("Fires when the value goes to 64 or above."))
    elif cur in ("er_lfo_rate", "er_lfo_reset"):
        card.add(W.hint("Goes to the i2c2midi's trigger LFO (ER-301 tab → Pitch & LFO)."))
    card.add(W.hint((hint + " " if hint else "") + "The DualSense script on the Pi runs it in addition to the MIDI going on to the "
                    "i2c2midi. All patches: Pi tab."))
    if not m.controls_on_pi:
        card.add(W.hint("⚠︎ The SSH host doesn't have the button functions installed yet."))
    return card
