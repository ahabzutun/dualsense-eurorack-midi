"""Teletype tab: the scene's scripts with what each line sends through the i2c2midi (and where it lands),
and what actually arrives from it at the Pi, live.

Scenes come from a USB stick the Teletype wrote (WRITE TO USB); edited scripts go back as tt##.txt
(READ FROM USB). See teletype.py for the format and the channel rules."""
from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional

from PySide6.QtCore import QTimer
from PySide6.QtGui import QColor, QPainterPath, QTextOption
from PySide6.QtWidgets import QPlainTextEdit, QVBoxLayout

from .. import draw as D
from .. import ssp as S
from .. import teletype as T
from .. import widgets as W
from ..stage import GrowingDrawing
from ..theme import BOLD, HEAVY, MEDIUM, REGULAR, SEMIBOLD, Theme

LIVE_FRESH = 5.0          # seconds a live value counts as "just now"
LIVE_ROWS = 14


def _op(c: QColor, a: float) -> QColor:
    q = QColor(c)
    q.setAlphaF(q.alphaF() * a)
    return q


def tt_color() -> QColor:
    return Theme.special


def scene_no(ctx) -> int:
    return getattr(ctx, "tt_scene", 0)


def current(ctx):
    """(scene, analysis) in use, or (None, None)."""
    sc = ctx.tt.scene(scene_no(ctx))
    return (sc, ctx.tt.analysis(scene_no(ctx))) if sc is not None else (None, None)


def ssp_targets(ctx, ch: int, kind: str, num: Optional[int]) -> List[str]:
    return [f"{S.module_title(m)} · {S.target_text(m, i)}" for m, i in S.listeners(ctx, ch, kind, num)]


def er_clashes(ctx, an: T.Analysis) -> dict:
    """SC ports the scene writes that the i2c2midi's routing also writes: port id → [what routes there]."""
    used = {u.id: u for u in ctx.mapping.port_map()}
    out = {}
    for u in an.ports:
        if u.port is not None and f"{u.kind}{u.port}" in used:
            out[f"{u.kind}{u.port}"] = used[f"{u.kind}{u.port}"]
    return out


def send_target(ctx, s: T.Send) -> str:
    """Where one send ends up, in words."""
    if s.trs:
        return "TRS jack (not connected to the Pi)"
    if s.bad_channel:
        return "no such channel"
    if s.usb is None:
        return "channel changes at run time"
    if s.kind in ("clock", "start", "stop", "continue"):
        return f"USB → hub → SSP, NerdSEQ ({s.kind})"
    hits = ssp_targets(ctx, s.usb, s.hub_kind, s.num) if s.hub_kind else []
    if hits:
        more = f" +{len(hits) - 1}" if len(hits) > 1 else ""
        return f"SSP {hits[0]}{more}"
    if ctx.ssp.active is not None:
        return "hub → SSP (no module listens to this)"
    return "hub → SSP / NerdSEQ"


def live_text(ctx, s: T.Send) -> Optional[str]:
    v = ctx.tt_mon.matches(s)
    if v is None:
        return None
    age = time.monotonic() - v.last
    return f"● {v.value}" if age < LIVE_FRESH else f"○ {v.value} · {int(age)} s ago"


# ── the drawing ────────────────────────────────────────────────────────────────────────────────

class TeletypeDrawing(GrowingDrawing):
    base = (900.0, 560.0)

    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx

    def content(self):
        ctx = self.ctx
        sc, an = current(ctx)
        head = [D.Text("TELETYPE", Theme.font(22, HEAVY, "title"), Theme.ink), D.Gap(6)]
        if sc is not None:
            nm = sc.name or "(no name)"
            head.append(D.Pill(D.Text(f"SCENE {sc.number:02d} · {nm}", Theme.font(11, BOLD, "mono"), tt_color()),
                               9, 3, _op(tt_color(), 0.14), tt_color(), 1.2))
        head.append(self._live_badge())
        parts = [D.Row(head, 8)]
        if not ctx.tt.loaded:
            parts.append(self._empty())
            return D.Col(parts, 14, "leading")
        if sc is None:
            parts.append(D.Text(f"Scene {scene_no(ctx)} isn't on the stick that was read. Pick another in the panel.",
                                Theme.font(11, MEDIUM, "mono"), Theme.dim))
            return D.Col(parts, 14, "leading")
        desc = [t for t in sc.text[1:] if t.strip()][:4]
        if desc:
            parts.append(D.Col([D.Text(t, Theme.font(11, MEDIUM, "mono"), Theme.dim) for t in desc], 1, "leading"))
        clashes = er_clashes(ctx, an)
        used = [s for s in T.SCRIPTS if sc.scripts.get(s) or getattr(ctx, "tt_script", "1") == s]
        cards = [self._script(sc, an, s, clashes) for s in used]
        per = 3
        parts.append(D.Grid([cards[k:k + per] for k in range(0, len(cards), per)], 10, 10, stretch=True, align="leading"))
        empty = [s for s in T.SCRIPTS if s not in used]
        if empty:
            chips = [D.Text("EMPTY", Theme.font(9, HEAVY, "mono"), Theme.dim, tracking=1)]
            for s in empty:
                pill = D.Pill(D.Text(s, Theme.font(10, BOLD, "mono"), Theme.dim), 7, 2, None, Theme.c("body", 0.35), 1)
                chips.append(D.Decor(pill, hits=self.hits, on_click=lambda s=s: ctx.select("tt_script", s, "Script"),
                                     tip=f"{T.SCRIPT_TITLE[s]}: write it in the panel"))
            parts.append(D.Row(chips, 5))
        parts.append(self._live())
        return D.Col(parts, 12, "leading")

    # header bits
    def _live_badge(self) -> D.Item:
        mon = self.ctx.tt_mon
        if mon.connected and mon.port:
            n = sum(1 for e in mon.log if time.monotonic() - e[0] < 1.0)
            t, c = f"● LIVE · {n} msg/s", Theme.cv
        elif mon.running and mon.connected:
            t, c = "LIVE · no XIAO link on the Pi", Theme.trig
        elif mon.running:
            t, c = "LIVE · connecting …", Theme.dim
        else:
            t, c = "○ not listening", Theme.dim
        pill = D.Pill(D.Text(t, Theme.font(10, BOLD, "mono"), c), 8, 3, _op(c, 0.12) if mon.running else None, _op(c, 0.6), 1)
        return D.Decor(pill, hits=self.hits, on_click=lambda: self.ctx.select("tt_tab", "Live", "Live"),
                       tip="What arrives from the i2c2midi at the Pi (panel → Live)")

    def _empty(self) -> D.Item:
        f = Theme.font(11, MEDIUM, "mono")
        lines = ["No scenes read yet.", "",
                 "1  Put a USB stick (FAT) into the Teletype.",
                 "2  In its USB menu choose WRITE TO USB (PARAM knob, then the button next to the port).",
                 "   Older firmware writes (and reads) by itself.",
                 "3  Put the stick into this computer and press  ↓ From stick.",
                 "", "The Teletype has no other way out: it only talks to keyboards, sticks and MIDI devices."]
        return D.Col([D.Text(t, f, Theme.ink if k else Theme.dim) for k, t in enumerate(lines)], 3, "leading")

    # one script card
    def _script(self, sc: T.Scene, an: T.Analysis, script: str, clashes: dict) -> D.Item:
        ctx = self.ctx
        g = Theme.grow
        cw = 320 * g
        sel = getattr(ctx, "tt_script", "1") == script
        lines = sc.lines(script)
        mono = Theme.font(12, SEMIBOLD, "mono")
        small = Theme.font(9.5, BOLD, "mono")
        rows = []
        title = D.Row([D.Text(script, Theme.font(15, HEAVY, "mono"), Theme.ink if lines else Theme.dim),
                       D.Text(T.SCRIPT_TITLE[script].upper() if script in "MI" else "", Theme.font(9, HEAVY, "mono"),
                              Theme.dim, tracking=1)], 6)
        rows.append(title)
        if not lines:
            rows.append(D.Text("—", mono, _op(Theme.dim, 0.6)))
        for li, ln in enumerate(lines):
            sends, ports, listens, chs = an.by_line(script, li)
            bad = bool(ln.errors) or any(s.trs or s.bad_channel for s in sends)
            col = Theme.trig if bad else (Theme.ink if (sends or ports or listens or chs) else _op(Theme.ink, 0.75))
            rows.append(D.Text(ln.text, mono, col, max_w=cw - 20, min_scale=0.75))
            tags = []
            for s in sends[:2]:
                if s.trs:
                    tags.append(self._tag(f"TRS ch {s.ch} ✕ not USB", Theme.trig))
                elif s.usb is not None:
                    tags.append(self._tag(f"USB {s.usb} · {s.label()}", tt_color()))
                    lv = live_text(ctx, s)
                    if lv:
                        tags.append(self._tag(lv, Theme.cv if lv.startswith("●") else Theme.dim, fill=False))
                else:
                    tags.append(self._tag(f"ch ? · {s.label()}", Theme.dim))
            if len(sends) > 2:
                tags.append(self._tag(f"+{len(sends) - 2}", Theme.dim, fill=False))
            for u in ports[:2]:
                pid = f"{u.kind}{u.port}" if u.port is not None else None
                clash = pid in clashes
                tags.append(self._tag(f"SC.{u.kind} {u.port if u.port is not None else u.port_text}" + (" ⚠" if clash else ""),
                                      Theme.trig if clash else (Theme.cv if u.kind == "CV" else Theme.trig)))
            for q in listens[:1]:
                tags.append(self._tag(f"reads {q.kind}{' ' + str(q.num) if q.num is not None else ''}", Theme.function))
            for c in chs[:1]:
                if c.query:
                    tags.append(self._tag(f"listens ch {c.text}", Theme.function))
                else:
                    tags.append(self._tag(f"sets ch {c.text}" + (f" = USB {c.ch - 16}" if c.ch and c.ch > 16 else
                                                                (" = TRS" if c.ch and c.ch <= 16 else "")),
                                          Theme.trig if c.ch and c.ch <= 16 else tt_color()))
            if ln.errors:
                rows.append(D.Text("⚠ " + ln.errors[0], small, Theme.trig, max_w=cw - 20, min_scale=0.7))
            if tags:
                rows.append(self._wrap_tags(tags, cw - 20))
            if sends and sends[0].usb is not None:
                rows.append(D.Text("→ " + send_target(ctx, sends[0]), Theme.font(9.5, MEDIUM, "mono"), Theme.dim,
                                   max_w=cw - 20, min_scale=0.75))
        content = D.Box(D.Col(rows, 4, "leading"), w=cw, min_h=120 * g, align="leading", valign="top", pad=12)
        has = bool(an.by_line(script, 0)[0]) or any(an.by_line(script, k)[0] for k in range(len(lines)))

        def bg(p, r):
            path = QPainterPath()
            path.addRoundedRect(r, 10, 10)
            if sel:
                D.glow(p, path, _op(tt_color(), 0.35), 8)
            p.fillPath(path, _op(tt_color(), 0.07) if has else Theme.c("body", 0.04))
            p.strokePath(path, D.pen(_op(Theme.ink, 0.75) if sel else (_op(tt_color(), 0.5) if has else Theme.c("body", 0.25)),
                                     1.5 if sel else 1.1))

        return D.Decor(content, before=bg, hits=self.hits, valign="top",
                       on_click=lambda: ctx.select("tt_script", script, "Script"),
                       tip=f"{T.SCRIPT_TITLE[script]}: edit and check it in the panel")

    def _tag(self, text: str, c: QColor, fill: bool = True) -> D.Item:
        return D.Pill(D.Text(text, Theme.font(9.5, BOLD, "mono"), c), 5, 1.5, _op(c, 0.15) if fill else None,
                      None if fill else _op(c, 0.5), 1)

    def _wrap_tags(self, tags: List[D.Item], max_w: float) -> D.Item:
        rows, cur, w = [], [], 0.0
        for t in tags:
            if cur and w + t.w + 4 > max_w:
                rows.append(D.Row(cur, 4))
                cur, w = [], 0.0
            cur.append(t)
            w += t.w + 4
        if cur:
            rows.append(D.Row(cur, 4))
        return D.Col(rows, 3, "leading")

    # what arrives
    def _live(self) -> D.Item:
        ctx = self.ctx
        mon = ctx.tt_mon
        g = Theme.grow
        f = Theme.font(10, SEMIBOLD, "mono")
        hdr = D.Text("ARRIVING FROM THE I2C2MIDI · USB → XIAO LINK → PI", Theme.font(10, HEAVY, "mono"), Theme.dim, tracking=1.2)
        if not mon.running and not mon.seen:
            return D.Col([hdr, D.Text("Not listening. Panel → Live → Listen shows what the Teletype really sends.",
                                      f, _op(Theme.dim, 0.8))], 6, "leading")
        sc, an = current(ctx)
        rows = []
        now = time.monotonic()
        items = sorted(mon.seen.values(), key=lambda v: -v.last)[:LIVE_ROWS]
        if not items:
            rows.append([D.Text("nothing yet" if mon.connected else (mon.error or "connecting …"), f, Theme.dim)])
        for v in items:
            age = now - v.last
            fresh = age < LIVE_FRESH
            what = f"ch {v.ch:<2} " + ({"cc": f"CC {v.num}", "note": f"note {v.num}", "poly AT": f"poly AT {v.num}"}
                                      .get(v.kind, v.kind))
            src = []
            if an is not None:
                for s in an.sends:
                    if s.usb == v.ch and (s.hub_kind == v.kind or s.kind == v.kind) and (s.num is None or s.num == v.num):
                        src.append(s.where.title)
            tg = ssp_targets(ctx, v.ch, v.kind, v.num) if v.kind in ("cc", "note") else []
            bar = D.Custom(70 * g, 8, lambda p, r, val=v.value, k=v.kind, fr=fresh: self._bar(p, r, val, k, fr))
            rows.append([
                D.Text(what, f, Theme.ink if fresh else Theme.dim),
                bar,
                D.Text(str(v.value), f, Theme.ink if fresh else Theme.dim),
                D.Text(f"×{v.count}", f, Theme.dim),
                D.Text("now" if age < 1 else f"{int(age)} s", f, Theme.cv if fresh else Theme.dim),
                D.Text(", ".join(src[:2]) if src else ("not in this scene" if an is not None else ""), f,
                       tt_color() if src else Theme.trig, max_w=170 * g, min_scale=0.75),
                D.Text("→ " + (tg[0] + (f" +{len(tg) - 1}" if len(tg) > 1 else "") if tg else "no SSP input"), f,
                       Theme.ink if tg else _op(Theme.dim, 0.8), max_w=260 * g, min_scale=0.75),
            ])
        grid = D.Grid(rows, 14, 4, stretch=False, align="leading")
        return D.Col([hdr, grid], 8, "leading")

    @staticmethod
    def _bar(p, r, val, kind, fresh):
        top = 16383 if kind == "pitch bend" else 127
        k = max(0.0, min(1.0, val / top))
        path = QPainterPath()
        path.addRoundedRect(r, 3, 3)
        p.fillPath(path, Theme.c("body", 0.12))
        if k > 0:
            fill = QPainterPath()
            fill.addRoundedRect(r.adjusted(0, 0, -r.width() * (1 - k), 0), 3, 3)
            p.fillPath(fill, _op(Theme.cv, 0.85 if fresh else 0.35))


# ── the panel ──────────────────────────────────────────────────────────────────────────────────

def build_panel(ctx, tab: str, col: QVBoxLayout):
    if tab == "Scene":
        scene_panel(ctx, col)
    elif tab == "Script":
        script_panel(ctx, col)
    else:
        live_panel(ctx, col)


def _when(t: float) -> str:
    return time.strftime("%d %b %H:%M", time.localtime(t)) if t else ""


def scene_panel(ctx, col: QVBoxLayout):
    st = ctx.tt
    c = tt_color()
    col.addWidget(W.title("Teletype"))
    card = W.Card("From the Teletype", c)
    card.add(W.hint("The Teletype only talks to sticks, keyboards and MIDI devices, so scenes travel by USB stick: "
                    "in its USB menu choose WRITE TO USB (every saved scene goes onto the stick as tt00s.txt …), "
                    "then put the stick into this computer. Scenes that were never saved on the Teletype aren't included."))
    if st.loaded:
        src = Path(st.imported_from).name or st.imported_from
        card.add(W.label(f"{len([s for s in st.scenes.values() if not s.empty])} scenes from “{src}”, {_when(st.imported_at)}",
                         11, MEDIUM, "ui", Theme.ink, wrap=True))
    card.add(W.hbox(W.small_button("↓  Read the stick", lambda: ctx.tt_import()),
                    W.small_button("Folder …", lambda: ctx.tt_import(pick=True)), stretch_end=True))
    col.addWidget(card)
    if not st.loaded:
        return

    card = W.Card("Scene in use", c)
    opts = []
    for n in range(T.SCENES):
        sc = st.scene(n)
        if sc is not None and not sc.empty:
            opts.append((n, f"{n:02d} · {sc.name or '(no name)'}"))
    if scene_no(ctx) not in dict(opts):
        opts.insert(0, (scene_no(ctx), f"{scene_no(ctx):02d} · (empty)"))
    card.add(W.Picker(opts, scene_no(ctx), lambda n: ctx.set_tt_scene(n)))
    card.add(W.hint("The Teletype doesn't say which scene is loaded: pick the one that runs. The Routing tab and the "
                    "SSP's “who sends this” lists use it."))
    guess = ctx.tt_guess()
    if guess is not None and guess != scene_no(ctx):
        sc = st.scene(guess)
        card.add(W.label(f"What arrives live fits scene {guess:02d} · {sc.name if sc else ''} better.", 11, MEDIUM, "ui",
                         Theme.trig, wrap=True))
        card.add(W.hbox(W.small_button(f"Use scene {guess:02d}", lambda: ctx.set_tt_scene(guess)), stretch_end=True))
    col.addWidget(card)

    sc, an = current(ctx)
    if sc is None:
        return
    warns = T.warnings(sc, an)
    clashes = er_clashes(ctx, an)
    for pid, use in clashes.items():
        warns.append((None, f"SC.{use.kind} {use.port} is written by this scene and by the i2c2midi's ER-301 routing "
                            "(ER-301 tab): the last one to write wins."))
    if warns:
        card = W.Card(f"Check ({len(warns)})", Theme.trig)
        for w, text in warns[:12]:
            card.add(W.label((f"{w.title}: " if w else "") + text, 11, REGULAR, "ui", Theme.ink, wrap=True))
        col.addWidget(card)

    usb = [s for s in an.sends if s.usb is not None]
    card = W.Card("Sends over USB", c)
    if not usb:
        card.add(W.hint("Nothing in this scene reaches the Pi (sends on ch 17–32 do)."))
    for s in usb[:16]:
        lv = live_text(ctx, s)
        card.add(W.label(f"{s.where.title}  ·  USB ch {s.usb} {s.label()}" + ("  (sometimes)" if s.guarded else ""),
                         11, BOLD, "mono", c))
        card.add(W.label("→ " + send_target(ctx, s) + (f"   {lv}" if lv else ""), 11, REGULAR, "ui", Theme.ink, wrap=True))
    if len(usb) > 16:
        card.add(W.hint(f"+{len(usb) - 16} more"))
    col.addWidget(card)

    card = W.Card("Description", c)
    ed = QPlainTextEdit("\n".join(sc.text))
    ed.setFont(Theme.font(11, MEDIUM, "mono"))
    ed.setFixedHeight(110)
    ed.setStyleSheet(f"QPlainTextEdit {{ background: {W.css(Theme.c('bg', 0.7))}; border: 1px solid "
                     f"{W.css(Theme.c('line'))}; border-radius: 6px; padding: 3px 4px; }}")
    ed.textChanged.connect(lambda: ctx.tt_set_text(ed.toPlainText().upper().split("\n")))
    card.add(ed)
    card.add(W.hint("The first line is the scene's name. Up to 32 lines of 32 characters."))
    col.addWidget(card)

    card = W.Card("Back to the Teletype", c)
    card.add(W.hint(f"↑ To stick writes tt{sc.number:02d}.txt next to the scenes that were read. On the Teletype: "
                    f"stick in, READ FROM USB — scene {sc.number:02d} is then replaced by it (DO BOTH would first "
                    "write the Teletype's scenes, then read)."))
    card.add(W.hbox(W.small_button(f"↑  Write tt{sc.number:02d}.txt", lambda: ctx.tt_write()), stretch_end=True))
    if ctx.tt_dirty.get(sc.number):
        card.add(W.label("Changed here, not on the Teletype yet.", 11, MEDIUM, "ui", Theme.trig))
    col.addWidget(card)


class _ScriptEdit(QPlainTextEdit):
    """Six lines, upper case, checked as you type."""

    def __init__(self, text: str, on_change, on_done=None):
        super().__init__(text)
        self._on_done = on_done
        self.setFont(Theme.font(12, SEMIBOLD, "mono"))
        self.setWordWrapMode(QTextOption.NoWrap)
        self.setTabChangesFocus(True)
        lh = self.fontMetrics().lineSpacing()
        self.setFixedHeight(int(lh * T.MAX_LINES + 18))
        self.setStyleSheet(f"QPlainTextEdit {{ background: {W.css(Theme.c('bg', 0.75))}; border: 1px solid "
                           f"{W.css(_op(tt_color(), 0.5))}; border-radius: 6px; padding: 4px 6px; }}")
        self._on_change = on_change
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(500)
        self._timer.timeout.connect(lambda: self._on_change(self.toPlainText().split("\n")))
        self.textChanged.connect(self._timer.start)

    def focusOutEvent(self, ev):
        if self._timer.isActive():
            self._timer.stop()
            self._on_change(self.toPlainText().split("\n"))
        super().focusOutEvent(ev)
        if self._on_done is not None:
            self._on_done()


def script_panel(ctx, col: QVBoxLayout):
    sc, an = current(ctx)
    c = tt_color()
    script = getattr(ctx, "tt_script", "1")
    col.addWidget(W.title(T.SCRIPT_TITLE[script]))
    col.addWidget(W.Segmented([(s, s) for s in T.SCRIPTS], script, lambda s: ctx.select("tt_script", s, "Script")))
    if sc is None:
        col.addWidget(W.hint("Read the scenes from the stick first (Scene tab)."))
        return
    card = W.Card(f"Scene {sc.number:02d} · {T.SCRIPT_TITLE[script]}", c)
    ed = _ScriptEdit("\n".join(sc.scripts.get(script, [])), lambda lines: ctx.tt_set_script(script, lines),
                     lambda: ctx.tabs["teletype"][2].rebuild())
    card.add(ed)
    card.add(W.hint(f"Up to {T.MAX_LINES} lines of {T.MAX_CHARS} characters, as on the Teletype. Checked as you type; "
                    "↑ To stick takes it to the Teletype."))
    col.addWidget(card)
    lines = sc.lines(script)
    for li, ln in enumerate(lines):
        sends, ports, listens, chs = an.by_line(script, li)
        if not (sends or ports or listens or chs or ln.errors):
            continue
        card = W.Card(f"Line {li + 1}", Theme.trig if ln.errors else c)
        card.add(W.label(ln.text, 12, BOLD, "mono", Theme.ink))
        for e in ln.errors:
            card.add(W.label("⚠ " + e, 11, REGULAR, "ui", Theme.trig, wrap=True))
        if ln.mod is not None:
            card.add(W.hint(f"Behind {ln.mod.tokens[0]}: runs only when that lets it."))
        for s in sends:
            if s.trs:
                where = f"ch {s.ch} = the i2c2midi's TRS jack, which isn't connected: nothing reaches the Pi. " \
                        f"Use ch {s.ch + 16} for USB ch {s.ch}."
                card.add(W.label(where, 11, REGULAR, "ui", Theme.trig, wrap=True))
                continue
            ch = f"USB ch {s.usb}" if s.usb is not None else "channel ?"
            card.add(W.label(f"{s.label()} on {ch}", 11, BOLD, "mono", c))
            card.add(W.hint(f"Channel: {s.ch_text}" + (f" · value: {s.value_text}" if s.value_text else "")))
            card.add(W.label("→ " + send_target(ctx, s), 11, REGULAR, "ui", Theme.ink, wrap=True))
            lv = live_text(ctx, s)
            if lv:
                card.add(W.label("live: " + lv, 11, MEDIUM, "mono", Theme.cv if lv.startswith("●") else Theme.dim))
        for u in ports:
            pid = f"{u.kind}{u.port}" if u.port is not None else None
            routed = pid in {x.id for x in ctx.mapping.port_map()} if pid else False
            card.add(W.label(f"ER-301 SC.{u.kind} {u.port if u.port is not None else u.port_text} — straight over I2C "
                             f"({u.op})", 11, BOLD, "mono", Theme.cv if u.kind == "CV" else Theme.trig))
            if routed:
                card.add(W.label("⚠ The i2c2midi's routing writes this port too (ER-301 tab).", 11, REGULAR, "ui",
                                 Theme.trig, wrap=True))
        for q in listens:
            card.add(W.label(f"Reads {q.kind}{' ' + str(q.num) if q.num is not None else ''} the i2c2midi received "
                             f"({q.op}) — from the MIDI hub: the controllers, on the channel set with I2M.Q.CH.",
                             11, REGULAR, "ui", Theme.ink, wrap=True))
        for cs in chs:
            if cs.query:
                card.add(W.hint(f"I2M.Q.CH {cs.text}: the I2M.Q ops listen to channel {cs.text}."))
            elif cs.ch is not None:
                card.add(W.label(f"Sets the channel for I2M ops without #: {cs.ch} = "
                                 + (f"USB ch {cs.ch - 16}" if cs.ch > 16 else "TRS (not connected)"), 11, MEDIUM, "ui",
                                 c if cs.ch > 16 else Theme.trig, wrap=True))
        col.addWidget(card)


def live_panel(ctx, col: QVBoxLayout):
    mon = ctx.tt_mon
    c = tt_color()
    col.addWidget(W.title("Live"))
    card = W.Card("What arrives from the i2c2midi", c)
    card.add(W.hint("Listens on the Pi to the XIAO-TT bridge: everything the Teletype sends on I2M ch 17–32 "
                    "(USB ch 1–16). The MIDI hub keeps getting it as before."))
    if mon.running:
        if not mon.connected:
            state = "Connecting to " + ctx.host.host + " …"
        elif mon.port:
            state = f"Listening on {mon.port.split(':')[0]}."
        else:
            state = "Connected, but the XIAO-TT isn't plugged into the Pi."
        card.add(W.label(state, 11, MEDIUM, "ui", Theme.ink if mon.port else Theme.trig, wrap=True))
    elif mon.error:
        card.add(W.label(mon.error, 11, MEDIUM, "ui", Theme.trig, wrap=True))
    card.add(W.hbox(W.small_button("■  Stop" if mon.running else "▶  Listen", lambda: ctx.tt_listen(not mon.running)),
                    W.small_button("Clear", lambda: mon.clear(), bool(mon.seen)), stretch_end=True))
    col.addWidget(card)

    sc, an = current(ctx)
    if an is not None and mon.seen:
        usb = [s for s in an.sends if s.usb is not None]
        missing = [s for s in usb if mon.matches(s) is None]
        extra = []
        for v in mon.seen.values():
            if not any(s.usb == v.ch and (s.hub_kind == v.kind or s.kind == v.kind) and (s.num is None or s.num == v.num)
                       for s in usb):
                extra.append(v)
        card = W.Card("Compared with the scene", c)
        if not missing and not extra:
            card.add(W.label(f"Everything that arrives is in scene {sc.number:02d}, and all of its USB sends arrived.",
                             11, MEDIUM, "ui", Theme.cv, wrap=True))
        for s in missing[:8]:
            card.add(W.label(f"Not seen yet: {s.where.title} · USB ch {s.usb} {s.label()}"
                             + (" (sometimes)" if s.guarded else ""), 11, REGULAR, "ui", Theme.ink, wrap=True))
        for v in extra[:8]:
            what = f"ch {v.ch} " + ({"cc": f"CC {v.num}", "note": "notes" if v.num is None else f"note {v.num}"}.get(v.kind, v.kind))
            card.add(W.label(f"Not in scene {sc.number:02d}: {what}", 11, REGULAR, "ui", Theme.trig, wrap=True))
        if missing:
            card.add(W.hint("Scripts run on triggers, metro or by hand, so a send can take a while to show up."))
        col.addWidget(card)
