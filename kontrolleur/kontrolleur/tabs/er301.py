"""ER-301 tab: all SC.CV ports with the device each one is fed from, and the routing inspector."""
from __future__ import annotations

from typing import List, Optional

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QLinearGradient, QPainter, QPainterPath
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget

from .. import draw as D
from .. import model as M
from .. import widgets as W
from ..devices import hub_channel, pedal_name
from ..stage import GrowingDrawing
from ..store import NotesStore
from ..theme import BOLD, HEAVY, MEDIUM, REGULAR, SEMIBOLD, Theme
from .dualsense import _op

DEVICES = ["DualSense", "16n", "MPC-20", "EC4", "i2c2midi", "MIDI"]


class Source:
    def __init__(self, device: str, detail: str, note: Optional[str] = None):
        self.device, self.detail, self.note = device, detail, note


def device_color(d: str) -> QColor:
    return {"DualSense": Theme.cv, "16n": Theme.special, "MPC-20": Theme.trig, "EC4": Theme.octave,
            "i2c2midi": Theme.function}.get(d, Theme.dim)


def describe(ctx, ref: tuple) -> List[Source]:
    """Which device feeds a port ('16n · fader 3' …)."""
    notes = ctx.notes
    if ref[0] == "dualsense":
        _, lab, iid, trig = ref
        return [Source("DualSense", lab + (" (trigger)" if trig else ""), notes.note(NotesStore.ds(iid)))]
    if ref[0] == "builtin":
        return [Source("i2c2midi", "trigger LFO" if ref[1] == "LFO" else "pitch from DualSense notes")]
    _, kind, ch, control = ref
    out: List[Source] = []
    try:
        cc = int(control)
    except ValueError:
        cc = None
    note = kind == "note"
    if kind in ("cc", "trigger", "note") and cc is not None:
        if not note:
            for i, f in enumerate(ctx.bank.faders):
                if f["usb_channel"] == ch and f["usb_cc"] == cc:
                    out.append(Source("16n", f"fader {i + 1}", notes.note(NotesStore.fader(i))))
        want = "note" if note else "cc"
        for inp in ctx.pedals.inputs:
            for s in inp["slots"]:
                if s["on"] and s["kind"] == want and s["number"] == cc and hub_channel(s["channel"], ctx.mapping) == ch:
                    out.append(Source("MPC-20", f"{pedal_name(inp['input'])} · message {s['slot'] + 1}",
                                      notes.note(NotesStore.pedal(inp["input"], s["slot"]))))
        hits = []
        for si, su in enumerate(ctx.ec4.setups):
            for gi, g in enumerate(su["groups"]):
                for ei, e in enumerate(g["encoders"]):
                    n = e["name"].strip()
                    where = f"{su['name'].strip()} · {g['name'].strip()} · {n or f'enc {ei + 1}'}"
                    if not note and e["type"] == "cc" and e["channel"] == ch and e["number"] == cc:
                        hits.append(Source("EC4", where, notes.note(NotesStore.ec4(si, gi, ei))))
                    pu = e["push"]
                    if pu["type"] == want and pu["channel"] == ch and pu["number"] == cc:
                        hits.append(Source("EC4", where + " push", notes.note(NotesStore.ec4_push(si, gi, ei))))
        if hits:
            out.append(hits[0] if len(hits) == 1 else Source("EC4", hits[0].detail + f" +{len(hits) - 1} more", hits[0].note))
    if not out:
        what = "note" if kind == "note" else ("NRPN" if kind == "nrpn" else "CC")
        out.append(Source("MIDI", f"ch {ch} {what} {control}" + (" (trigger)" if kind == "trigger" else "")))
    return out


def sources(ctx, use: M.PortUse) -> List[Source]:
    return [s for r in use.refs for s in describe(ctx, r)]


def colors(srcs: List[Source]) -> List[QColor]:
    seen = []
    for s in srcs:
        if s.device not in seen:
            seen.append(s.device)
    return [device_color(d) for d in seen]


def bands(rect: QRectF, cols: List[QColor], opacity: float = 1.0) -> QBrush:
    """Hard-edged bands, one per colour (a port fed by DualSense + 16n is half cyan, half green)."""
    cols = cols or [Theme.dim]
    g = QLinearGradient(rect.topLeft(), rect.bottomRight())
    n = len(cols)
    for k, c in enumerate(cols):
        a, b = k / n, (k + 1) / n
        cc = _op(c, opacity)
        g.setColorAt(a + (1e-4 if k else 0), cc)
        g.setColorAt(b - (1e-4 if k < n - 1 else 0), cc)
    return QBrush(g)


class ER301Drawing(GrowingDrawing):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx

    def content(self):
        ctx = self.ctx
        m = ctx.mapping
        pmap = m.port_map()
        used = {u.id: u for u in pmap}
        view = ctx.er_view
        grid = self._grid(view, list(range(1, 100)), used)
        toggles = [self._toggle(k, sum(1 for u in pmap if u.kind == k), view == k) for k in ("CV", "TR")]
        left = D.Row([D.Text("ER-301", Theme.font(22, HEAVY, "title"), Theme.ink), D.Gap(4)] + toggles, 8)
        right = D.Row([_device_badge(d) for d in DEVICES], 5)
        width = max(grid.w, left.w + 20 + right.w)
        head = D.Custom(width, max(left.h, right.h), lambda p, r: (left.paint(p, r.left(), r.top() + (r.height() - left.h) / 2),
                                                                   right.paint(p, r.right() - right.w, r.top() + (r.height() - right.h) / 2)))
        parts = [head, grid]
        if view == "TR" and m.triggers_as_cv:
            parts.append(D.Text("Triggers currently go to SC.CV ports (ER-301 tab → Pitch & LFO → Output). "
                                "SC.TR ports can still be described here.", Theme.font(10, MEDIUM, "mono"), Theme.dim))
        return D.Col(parts, 10, "leading")

    def _toggle(self, kind: str, n: int, on: bool) -> D.Item:
        """SC.CV / SC.TR switch in the header."""
        col = Theme.cv if kind == "CV" else Theme.trig
        t = D.Text(f"SC.{kind} 1–99 · {n} in use", Theme.font(11, BOLD if on else SEMIBOLD, "mono"), col if on else Theme.dim)
        pill = D.Pill(t, 9, 3, _op(col, 0.16) if on else None, col if on else Theme.line, 1.2)
        return D.Decor(pill, hits=self.hits, on_click=lambda: self.ctx.select("er_view", kind),
                       tip=f"Show the SC.{kind} ports")

    def _grid(self, kind: str, ports: List[int], used) -> D.Grid:
        rows, row = [], []
        for p in ports:
            pid = f"{kind}{p}"
            srcs = sources(self.ctx, used[pid]) if pid in used else []
            row.append(self._tile(kind, p, srcs))
            if len(row) == 10:
                rows.append(row)
                row = []
        if row:
            rows.append(row)
        return D.Grid(rows, 5, 5, stretch=True, align="leading")

    def _tile(self, kind: str, port: int, srcs: List[Source]) -> D.Item:
        ctx = self.ctx
        pid = f"{kind}{port}"
        sel = ctx.er_port == pid
        cols = colors(srcs)
        col = cols[0] if cols else Theme.dim
        g = Theme.grow
        inner_w = 100 * g - 10
        num = D.Text(str(port), Theme.font(11, HEAVY, "mono"), _op(Theme.dim, 0.7) if not srcs else col)
        top = [num]
        if len(srcs) > 1:
            badge = _shared(len(srcs))
            top = [D.Custom(inner_w, max(num.h, badge.h), lambda p, r, n=num, b=badge: (
                n.paint(p, r.left(), r.top() + (r.height() - n.h) / 2), b.paint(p, r.right() - b.w, r.top() + (r.height() - b.h) / 2)))]
        lines = list(top)
        if srcs:
            s = srcs[0]
            devs = " + ".join(sorted(set(x.device for x in srcs)))
            lines.append(D.Text(devs, Theme.font(8, HEAVY, "mono"), col, max_w=inner_w, min_scale=0.8))
            lines.append(D.Text(s.note or s.detail, Theme.font(8.5, MEDIUM), Theme.ink if s.note is None else Theme.note,
                                max_w=inner_w, min_scale=0.8))
        desc = ctx.notes.note(NotesStore.port(kind, port))
        if desc:
            lines.append(D.Text("» " + NotesStore.one_line(desc), Theme.font(8.5, BOLD), Theme.special, max_w=inner_w, min_scale=0.8))
        content = D.Box(D.Col(lines, 2, "leading"), w=100 * g, min_h=52 * g, align="leading", valign="top", pad=5)

        def bg(p, r):
            path = QPainterPath()
            path.addRoundedRect(r, 7, 7)
            if not srcs:
                p.strokePath(path, D.pen(_op(Theme.ink, 0.8) if sel else Theme.c("body", 0.15), 1.6 if sel else 1, [2, 4]))
                return
            p.fillPath(path, bands(r, cols, 0.16))
            if sel:
                p.strokePath(path, D.pen(_op(Theme.ink, 0.8), 1.6))
            else:
                from PySide6.QtGui import QPen
                q = QPen(bands(r, cols, 0.6), 1.2)
                p.strokePath(path, q)

        tip = (f"SC.{kind} {port}: free" if not srcs else f"SC.{kind} {port}\n" + "\n".join(
            f"{s.device}: {s.detail}" + (f" — {s.note}" if s.note else "") for s in srcs))
        if desc:
            tip += f"\nSteers: {desc}"
        return D.Decor(content, before=bg, hits=self.hits, tip=tip, valign="top",
                       on_click=lambda: ctx.select("er_port", None if ctx.er_port == pid else pid, "Ports"))


def _device_badge(d: str) -> D.Item:
    c = device_color(d)
    return D.Pill(D.Text(d, Theme.font(9, HEAVY, "mono"), c), 5, 1.5, _op(c, 0.15))


def _shared(n: int) -> D.Item:
    return D.Pill(D.Text(f"×{n}", Theme.font(9, HEAVY, "mono"), Theme.ink), 4, 1, None, _op(Theme.ink, 0.45), 1)


# ── inspector ───────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    if tab == "Ports":
        ports_panel(ctx, col)
    elif tab == "Routes":
        routes_panel(ctx, col)
    else:
        pitch_lfo_panel(ctx, col)


def _badge_label(d: str) -> QLabel:
    c = device_color(d)
    lb = W.label(d, 9, HEAVY, "mono", c)
    lb.setStyleSheet(f"color: {W.css(c)}; background: {W.css(_op(c, 0.15))}; border-radius: 7px; padding: 1px 5px;")
    lb.setSizePolicy(lb.sizePolicy().horizontalPolicy(), lb.sizePolicy().verticalPolicy())
    return lb


class _PortRow(QFrame):
    """A row of the port list; clicking it selects the port (as in the drawing)."""

    def __init__(self, on_click):
        super().__init__()
        self._on_click = on_click
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._on_click()
        super().mousePressEvent(ev)


DESC_TITLE = "Steers in the ER-301"
DESC_PROMPT = "e.g. “grain size of the granular player, chain 2”"


def ports_panel(ctx, col: QVBoxLayout):
    col.addWidget(W.title("ER-301 ports"))
    col.addWidget(W.hint("Set these as the sources of your ER-301 unit inputs. Each entry shows which device the MIDI comes from. "
                         "Click a port to select it and note what it steers in your ER-301 patch. "
                         "“×2” marks a port with several sources: fine when intended (e.g. ✕ and the LFO both firing the grain "
                         "trigger); on a CV port the value sent last wins."))
    pmap = ctx.mapping.port_map()
    content = col.parentWidget()
    # a free port picked in the drawing: describe it here (e.g. while planning a patch)
    sel = ctx.er_port
    if sel and sel not in {u.id for u in pmap}:
        kind, port = sel[:2], int(sel[2:])
        card = W.Card(f"SC.{kind} {port} · free", Theme.cv if kind == "CV" else Theme.trig)
        card.add(W.hint("Nothing is routed to this port yet."))
        key = NotesStore.port(kind, port)
        card.add(W.NoteField(ctx.notes.raw(key), lambda t, k=key: ctx.set_note(k, t), DESC_TITLE, DESC_PROMPT))
        col.addWidget(card)
        content._focus = card
    if not pmap:
        col.addWidget(W.hint("Nothing routed yet."))
    for use in pmap:
        srcs = sources(ctx, use)
        selected = sel == use.id
        fr = _PortRow(lambda uid=use.id: ctx.er_port != uid and ctx.select("er_port", uid, "Ports"))
        fr.setObjectName("portrow")
        fr.setStyleSheet(f"#portrow {{ background: {W.css(Theme.card)}; border-radius: 10px; border: 1.2px solid "
                         f"{W.css(_op(Theme.ink, 0.7)) if selected else 'transparent'}; }}")
        outer = QVBoxLayout(fr)
        outer.setContentsMargins(12, 8, 12, 8)
        outer.setSpacing(8)
        h = QHBoxLayout()
        h.setSpacing(10)
        name = W.label(f"SC.{use.kind} {use.port}", 13, HEAVY, "mono", Theme.cv if use.kind == "CV" else Theme.trig)
        name.setFixedWidth(int(84 * Theme.grow))
        h.addWidget(name, 0, Qt.AlignTop)
        v = QVBoxLayout()
        v.setSpacing(5)
        for s in srcs:
            r = QHBoxLayout()
            r.setSpacing(6)
            r.addWidget(_badge_label(s.device), 0, Qt.AlignTop)
            r.addWidget(W.label(s.detail, 12, REGULAR, "ui", Theme.ink, wrap=True), 1)
            v.addLayout(r)
            if s.note:
                v.addWidget(W.label("✎ " + NotesStore.one_line(s.note), 10.5, MEDIUM, "ui", Theme.note, wrap=True, italic=True))
        key = NotesStore.port(use.kind, use.port)
        desc = ctx.notes.note(key)
        if desc and not selected:
            v.addWidget(W.label("» " + NotesStore.one_line(desc), 11, BOLD, "ui", Theme.special, wrap=True))
        h.addLayout(v, 1)
        if len(srcs) > 1:
            b = W.label(f"×{len(srcs)}", 9, HEAVY, "mono")
            b.setStyleSheet(f"color: {W.css(Theme.ink)}; border: 1px solid {W.css(_op(Theme.ink, 0.45))}; border-radius: 7px; padding: 0 4px;")
            b.setToolTip(f"{len(srcs)} sources share this port. On a CV port the value sent last wins; on a trigger port each source fires it.")
            h.addWidget(b, 0, Qt.AlignTop)
        outer.addLayout(h)
        if selected:
            outer.addWidget(W.NoteField(ctx.notes.raw(key), lambda t, k=key: ctx.set_note(k, t), DESC_TITLE, DESC_PROMPT))
            content._focus = fr
        elif not desc:
            fr.setToolTip("Click to select this port and describe what it steers")
        col.addWidget(fr)


def routes_panel(ctx, col: QVBoxLayout):
    m = ctx.mapping
    col.addWidget(W.title("MIDI routes"))
    col.addWidget(W.hint("Everything from the MIDI hub that goes to the ER-301 besides the DualSense: 16n, pedals, EC4 or any other "
                         "source. The device tabs add and remove their routes here for you; you can also add routes by hand (NRPN, "
                         "notes, triggers, other channels). Control is a CC / NRPN / note number, or a name from mappings.py."))
    for e in list(m.externals):
        card = W.Card(e.kind, Theme.cv if e.kind in (M.EXT_CV, M.EXT_NRPN) else Theme.trig)

        def set_kind(k, e=e):
            e.kind = k
            ctx.changed(True)

        def remove(e=e):
            m.externals = [x for x in m.externals if x.id != e.id]
            ctx.changed(True)

        card.add(W.hbox(W.Picker([(k, k) for k in M.EXT_KINDS], e.kind, set_kind, 150), "stretch",
                        W.icon_button("✕", remove, "Remove this route")))
        card.add(W.row("Channel", W.NumberField(e.channel, 1, 16, lambda v, e=e: (setattr(e, "channel", v), ctx.changed())), REGULAR))
        ed = QLineEdit(e.control)
        ed.setAlignment(Qt.AlignRight)
        ed.setFont(Theme.font(13, SEMIBOLD, "mono"))
        ed.setFixedWidth(int(140 * Theme.grow))
        ed.textEdited.connect(lambda t, e=e: (setattr(e, "control", t), ctx.changed()))
        card.add(W.row("Note" if e.kind == M.EXT_NOTE else ("NRPN" if e.kind == M.EXT_NRPN else "CC"), ed, REGULAR))
        card.add(W.row("SC.CV port" if e.kind in (M.EXT_CV, M.EXT_NRPN) else f"{m.trigger_port_name} port",
                       W.NumberField(e.port, 1, 99, lambda v, e=e: (setattr(e, "port", v), ctx.changed())), REGULAR))
        col.addWidget(card)

    def add():
        m.externals.append(M.ExternalRoute(M.EXT_CV, m.channel, "109", m.first_free_port()))
        ctx.changed(True)

    b = QPushButton("＋  Add route")
    b.setMinimumHeight(32)
    b.setCursor(Qt.PointingHandCursor)
    b.clicked.connect(add)
    col.addWidget(b)


NOTE_NAMES = ["C", "C♯", "D", "D♯", "E", "F", "F♯", "G", "G♯", "A", "A♯", "B"]


def pitch_lfo_panel(ctx, col: QVBoxLayout):
    m = ctx.mapping

    def seta(k, v, rebuild=False):
        setattr(m, k, v)
        ctx.changed(rebuild)

    card = W.Card("Pitch · V/Oct", Theme.cv)
    card.add(W.switch_row("Notes → pitch CV", m.pitch_enabled, lambda v: seta("pitch_enabled", v, True)))
    if m.pitch_enabled:
        card.add(W.row("SC.CV port", W.NumberField(m.pitch_port, 1, 99, lambda v: seta("pitch_port", v))))
        nn = W.label(f"{NOTE_NAMES[m.zero_note % 12]}{m.zero_note // 12 - 1}", 12, SEMIBOLD, "mono", Theme.cv)

        def set_zero(v):
            nn.setText(f"{NOTE_NAMES[v % 12]}{v // 12 - 1}")
            seta("zero_note", v)

        card.add(W.row("0 V note", W.hbox(nn, W.NumberField(m.zero_note, 0, 127, set_zero), spacing=8)))
        card.add(W.row("Semitones per octave step", W.NumberField(m.semis_per_step, 0, 48, lambda v: seta("semis_per_step", v))))
        oc = m.input_with(M.OCTAVE)
        card.add(W.row("Octave control", W.label(M.input_title(oc) if oc else "none", 12, REGULAR, "mono", Theme.octave)))
        card.add(W.hint("Pick the octave control on the D-pad (DualSense tab → Control → Octave)."))
    col.addWidget(card)

    col.addWidget(lfo_card(ctx))

    card = W.Card("Output")
    card.add(W.switch_row("Triggers as SC.CV (0 / 10 V)", m.triggers_as_cv, lambda v: seta("triggers_as_cv", v, True)))
    card.add(W.hint("Trigger ports are SC.CV ports. Use this when the ER-301 trigger input's source is an SC.CV." if m.triggers_as_cv
                    else "Trigger ports are SC.TR ports (TR / TR.PULSE)."))
    col.addWidget(card)


def lfo_card(ctx) -> W.Card:
    """The trigger LFO of the i2c2midi: rate / reset controls, output, range."""
    m = ctx.mapping

    def seta(k, v, rebuild=False):
        setattr(m, k, v)
        ctx.changed(rebuild)

    card = W.Card("Trigger LFO", Theme.special)

    from .pi import source_label

    def lfo_pick(role):
        attr = "lfo_rate_src" if role == M.LFO_RATE else "lfo_reset_src"

        def pick(iid):
            if iid == "ext":
                return
            cur = m.input_with(role)
            if cur:
                m.set_choice(cur, "midi")
            setattr(m, attr, None)
            if iid:
                m.set_choice(M.INPUT[iid], "er_lfo_rate" if role == M.LFO_RATE else "er_lfo_reset")
            ctx.changed(True)
        return pick

    def opts(role, none_text):
        src = m.lfo_rate_src if role == M.LFO_RATE else m.lfo_reset_src
        o = [("", none_text)] + [(i.id, "DualSense · " + M.input_title(i)) for i in M.ALL_INPUTS if role in i.roles]
        if src:
            o.append(("ext", f"{M.PI_DEVICE_TITLE[src[0]]} · {source_label(ctx, src[0], src[1], 'cc', src[2])}"))
        cur = m.input_with(role)
        return o, ("ext" if src else (cur.id if cur else ""))

    o, cur = opts(M.LFO_RATE, "none (LFO off)")
    card.add(W.row("Rate control", W.Picker(o, cur, lfo_pick(M.LFO_RATE), 190)))
    o, cur = opts(M.LFO_RESET, "none")
    card.add(W.row("Phase reset", W.Picker(o, cur, lfo_pick(M.LFO_RESET), 190)))
    card.add(W.hint(f"Listens on MIDI channel {m.lfo_channel}. A 16n fader, a pedal or an EC4 control: Pi tab → LFO → Patch a controller."))
    lp = m.lfo_problem()
    if lp:
        card.add(W.label("⚠︎ " + lp, 11, REGULAR, "ui", Theme.trig, wrap=True))
    card.add(W.divider())
    card.add(W.row("Output", W.Segmented([(True, "SC.CV"), (False, "SC.TR")], m.triggers_as_cv,
                                         lambda v: seta("triggers_as_cv", v, True))))
    card.add(W.row(f"{m.trigger_port_name} port", W.NumberField(m.lfo_port, 1, 99, lambda v: seta("lfo_port", v))))
    card.add(W.hint("SC.CV: 10 V pulses. SC.TR: trigger pulses. The i2c2midi has one setting for all triggers, so this also "
                    "moves the other triggers (buttons, pedals, EC4 …) to " + ("SC.CV." if m.triggers_as_cv else "SC.TR.")
                    + " A separate output per LFO needs a firmware update."))
    card.add(W.divider())
    card.add(W.row("Slowest (Hz)", W.DecimalField(m.lfo_min_hz, 0.0, 200.0, lambda v: seta("lfo_min_hz", v))))
    card.add(W.row("Fastest (Hz)", W.DecimalField(m.lfo_max_hz, 0.0, 200.0, lambda v: seta("lfo_max_hz", v))))
    card.add(W.row("Pulse length (ms)", W.NumberField(m.lfo_pulse_ms, 1, 127, lambda v: seta("lfo_pulse_ms", v))))
    card.add(W.hint("Exponential curve: finer control at slow speeds. Pulses get shorter automatically at fast rates."))
    return card
