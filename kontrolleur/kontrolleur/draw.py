"""
Tiny layout kit for the drawings (QPainter): text, pills, rows, columns, boxes — each measures
itself with the current fonts, so drawings make room for bigger text the way SwiftUI stacks do.
"""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QFontMetricsF, QPainter, QPainterPath, QPen

from .theme import Theme


def fm(font: QFont) -> QFontMetricsF:
    return QFontMetricsF(font)


def text_width(font: QFont, s: str) -> float:
    return fm(font).horizontalAdvance(s)


def tracked(font: QFont, tracking: float) -> QFont:
    f = QFont(font)
    if tracking:
        f.setLetterSpacing(QFont.AbsoluteSpacing, tracking)
    return f


class Item:
    w: float = 0.0
    h: float = 0.0

    def size(self) -> Tuple[float, float]:
        return self.w, self.h

    def paint(self, p: QPainter, x: float, y: float):
        pass

    def paint_center(self, p: QPainter, cx: float, cy: float):
        w, h = self.size()
        self.paint(p, cx - w / 2, cy - h / 2)
        return QRectF(cx - w / 2, cy - h / 2, w, h)


class Text(Item):
    def __init__(self, s: str, font: QFont, color: QColor, tracking: float = 0.0, max_w: Optional[float] = None,
                 lines: int = 1, align=Qt.AlignLeft, min_scale: float = 1.0):
        self.s, self.color, self.lines, self.align = s, color, lines, align
        self.font = tracked(font, tracking)
        m = fm(self.font)
        natural = m.horizontalAdvance(s)
        if max_w is not None and natural > max_w and min_scale < 1.0:
            # shrink first (like minimumScaleFactor), then elide
            k = max(min_scale, max_w / natural)
            self.font.setPixelSize(max(1, int(self.font.pixelSize() * k)))
            m = fm(self.font)
            natural = m.horizontalAdvance(s)
        self.max_w = max_w
        self.line_h = m.height()
        if max_w is not None and natural > max_w:
            if lines <= 1:
                self.text_lines = [m.elidedText(s, Qt.ElideRight, max_w)]
            else:
                self.text_lines = _wrap(s, m, max_w, lines)
            self.w = min(max_w, max(m.horizontalAdvance(t) for t in self.text_lines))
        else:
            self.text_lines = [s]
            self.w = natural
        self.h = self.line_h * len(self.text_lines)

    def paint(self, p, x, y):
        p.setFont(self.font)
        p.setPen(self.color)
        for k, t in enumerate(self.text_lines):
            p.drawText(QRectF(x, y + k * self.line_h, self.w + 2, self.line_h), int(self.align | Qt.AlignVCenter), t)


def _wrap(s: str, m: QFontMetricsF, max_w: float, lines: int) -> List[str]:
    words, out, cur = s.split(" "), [], ""
    for w in words:
        t = (cur + " " + w).strip()
        if m.horizontalAdvance(t) <= max_w or not cur:
            cur = t
        else:
            out.append(cur)
            cur = w
    out.append(cur)
    if len(out) > lines:
        out = out[:lines]
        out[-1] = m.elidedText(out[-1] + " …", Qt.ElideRight, max_w)
    return [m.elidedText(t, Qt.ElideRight, max_w) for t in out]


class Gap(Item):
    def __init__(self, w=0.0, h=0.0):
        self.w, self.h = w, h


class Row(Item):
    def __init__(self, items: Sequence[Item], spacing: float = 5.0, valign: str = "center"):
        self.items = [i for i in items if i is not None]
        self.spacing, self.valign = spacing, valign
        sizes = [i.size() for i in self.items]
        self.w = sum(s[0] for s in sizes) + spacing * max(0, len(sizes) - 1)
        self.h = max((s[1] for s in sizes), default=0.0)

    def paint(self, p, x, y):
        cx = x
        for i in self.items:
            w, h = i.size()
            dy = 0 if self.valign == "top" else (self.h - h if self.valign == "bottom" else (self.h - h) / 2)
            i.paint(p, cx, y + dy)
            cx += w + self.spacing


class Col(Item):
    def __init__(self, items: Sequence[Item], spacing: float = 3.0, align: str = "center"):
        self.items = [i for i in items if i is not None]
        self.spacing, self.align = spacing, align
        sizes = [i.size() for i in self.items]
        self.w = max((s[0] for s in sizes), default=0.0)
        self.h = sum(s[1] for s in sizes) + spacing * max(0, len(sizes) - 1)

    def paint(self, p, x, y):
        cy = y
        for i in self.items:
            w, h = i.size()
            dx = 0 if self.align == "leading" else (self.w - w if self.align == "trailing" else (self.w - w) / 2)
            i.paint(p, x + dx, cy)
            cy += h + self.spacing


class Pill(Item):
    """A child with padding and a capsule / rounded background."""

    def __init__(self, child: Item, padh=6.0, padv=2.0, fill: Optional[QColor] = None, stroke: Optional[QColor] = None,
                 width: float = 1.0, radius: Optional[float] = None, dash: Optional[List[float]] = None):
        self.child, self.padh, self.padv = child, padh, padv
        self.fill, self.stroke, self.width, self.radius, self.dash = fill, stroke, width, radius, dash
        cw, ch = child.size()
        self.w, self.h = cw + 2 * padh, ch + 2 * padv

    def paint(self, p, x, y):
        r = self.radius if self.radius is not None else self.h / 2
        rect = QRectF(x, y, self.w, self.h)
        rounded(p, rect, r, self.fill, self.stroke, self.width, self.dash)
        self.child.paint(p, x + self.padh, y + self.padv)


class Box(Item):
    """Fixed / minimum size around a child, aligned inside."""

    def __init__(self, child: Item, w: Optional[float] = None, h: Optional[float] = None, min_w=0.0, min_h=0.0,
                 align: str = "center", valign: str = "center", pad: float = 0.0):
        self.child, self.align, self.valign, self.pad = child, align, valign, pad
        cw, ch = child.size()
        self.w = w if w is not None else max(min_w, cw + 2 * pad)
        self.h = h if h is not None else max(min_h, ch + 2 * pad)

    def paint(self, p, x, y):
        cw, ch = self.child.size()
        dx = self.pad if self.align == "leading" else (self.w - cw - self.pad if self.align == "trailing" else (self.w - cw) / 2)
        dy = self.pad if self.valign == "top" else (self.h - ch - self.pad if self.valign == "bottom" else (self.h - ch) / 2)
        self.child.paint(p, x + dx, y + dy)


class Custom(Item):
    def __init__(self, w: float, h: float, fn: Callable[[QPainter, QRectF], None]):
        self.w, self.h, self.fn = w, h, fn

    def paint(self, p, x, y):
        self.fn(p, QRectF(x, y, self.w, self.h))


# ── shapes ──────────────────────────────────────────────────────────────────────────────────────

def pen(color: QColor, width: float = 1.0, dash: Optional[List[float]] = None, cap=Qt.RoundCap) -> QPen:
    q = QPen(color, width)
    q.setCapStyle(cap)
    q.setJoinStyle(Qt.RoundJoin)
    if dash:
        q.setDashPattern([d / max(width, 0.01) for d in dash])
    return q


def rounded(p: QPainter, rect: QRectF, r: float, fill: Optional[QColor] = None, stroke: Optional[QColor] = None,
            width: float = 1.0, dash: Optional[List[float]] = None):
    path = QPainterPath()
    path.addRoundedRect(rect, r, r)
    draw_path(p, path, fill, stroke, width, dash)
    return path


def draw_path(p: QPainter, path: QPainterPath, fill=None, stroke=None, width: float = 1.0, dash=None):
    if fill is not None and fill.alpha() > 0:
        p.fillPath(path, QBrush(fill))
    if stroke is not None and stroke.alpha() > 0:
        p.strokePath(path, pen(stroke, width, dash))


def circle(p: QPainter, c: QPointF, d: float, fill=None, stroke=None, width: float = 1.0, dash=None) -> QPainterPath:
    path = QPainterPath()
    path.addEllipse(c, d / 2, d / 2)
    draw_path(p, path, fill, stroke, width, dash)
    return path


def glow(p: QPainter, path: QPainterPath, color: QColor, radius: float = 10.0):
    """Soft halo around a shape (SwiftUI .shadow with a colour)."""
    if color.alpha() == 0:
        return
    steps = 6
    for k in range(steps, 0, -1):
        c = QColor(color)
        c.setAlphaF(color.alphaF() * 0.055)
        p.strokePath(path, pen(c, radius * 2 * k / steps))


def grid(p: QPainter, w: float, h: float):
    """Blueprint grid: 20-unit minor, 100-unit major lines."""
    minor, major = QPainterPath(), QPainterPath()
    x = 0.0
    while x <= w:
        (major if int(x) % 100 == 0 else minor).moveTo(x, 0)
        (major if int(x) % 100 == 0 else minor).lineTo(x, h)
        x += 20
    y = 0.0
    while y <= h:
        (major if int(y) % 100 == 0 else minor).moveTo(0, y)
        (major if int(y) % 100 == 0 else minor).lineTo(w, y)
        y += 20
    p.strokePath(minor, QPen(Theme.c("body", 0.035), 0.5))
    p.strokePath(major, QPen(Theme.c("body", 0.07), 0.7))


def vgradient(rect: QRectF, top: QColor, bottom: QColor):
    from PySide6.QtGui import QLinearGradient
    g = QLinearGradient(rect.topLeft(), rect.bottomLeft())
    g.setColorAt(0, top)
    g.setColorAt(1, bottom)
    return QBrush(g)


def note_badge(text: str, size: float = 9.5, max_w: Optional[float] = None, lines: int = 1) -> Text:
    from .store import NotesStore
    return Text("✎ " + NotesStore.one_line(text), Theme.font(size, 500, "ui", italic=True), Theme.note,
                max_w=max_w, lines=lines)


# ── hit areas ───────────────────────────────────────────────────────────────────────────────────

class Hits:
    """Clickable areas recorded while painting (drawing coordinates)."""

    def __init__(self):
        self.areas: List[Tuple[object, Callable, Optional[str]]] = []

    def clear(self):
        self.areas = []

    def add(self, shape, on_click: Optional[Callable], tip: Optional[str] = None):
        self.areas.append((shape, on_click, tip))

    def at(self, pt: QPointF):
        for shape, cb, tip in reversed(self.areas):
            inside = shape.contains(pt)
            if inside:
                return cb, tip
        return None, None


class Decor(Item):
    """A child with things painted behind / over it, optionally clickable (registers its rect)."""

    def __init__(self, child: Item, before: Optional[Callable] = None, after: Optional[Callable] = None,
                 hits: Optional[Hits] = None, on_click: Optional[Callable] = None, tip: Optional[str] = None,
                 w: Optional[float] = None, h: Optional[float] = None, valign: str = "center"):
        self.child, self.before, self.after, self.valign = child, before, after, valign
        self.hits, self.on_click, self.tip = hits, on_click, tip
        cw, ch = child.size()
        self.w = w if w is not None else cw
        self.h = h if h is not None else ch

    def paint(self, p, x, y):
        r = QRectF(x, y, self.w, self.h)
        if self.before:
            self.before(p, r)
        cw, ch = self.child.size()
        self.child.paint(p, x + (self.w - cw) / 2, y + (0 if self.valign == "top" else (self.h - ch) / 2))
        if self.after:
            self.after(p, r)
        if self.hits is not None and (self.on_click or self.tip):
            self.hits.add(r, self.on_click, self.tip)


class Grid(Item):
    """Rows of items; every column as wide as its widest cell, every row as tall as its tallest
    cell. Cells are stretched to the cell size when `stretch` (like .frame(maxHeight: .infinity))."""

    def __init__(self, rows: Sequence[Sequence[Item]], hspacing: float = 5.0, vspacing: float = 5.0,
                 stretch: bool = True, align: str = "center"):
        self.rows = [list(r) for r in rows]
        self.hs, self.vs, self.stretch, self.align = hspacing, vspacing, stretch, align
        ncol = max((len(r) for r in self.rows), default=0)
        self.cw = [0.0] * ncol
        self.rh = []
        for r in self.rows:
            h = 0.0
            for k, it in enumerate(r):
                w, hh = it.size()
                self.cw[k] = max(self.cw[k], w)
                h = max(h, hh)
            self.rh.append(h)
        self.w = sum(self.cw) + hspacing * max(0, ncol - 1)
        self.h = sum(self.rh) + vspacing * max(0, len(self.rows) - 1)

    def paint(self, p, x, y):
        cy = y
        for r, h in zip(self.rows, self.rh):
            cx = x
            for k, it in enumerate(r):
                if self.stretch and hasattr(it, "w"):
                    it.w, it.h = max(it.w, self.cw[k]) if self.align == "fill" else it.w, max(it.h, h)
                w, hh = it.size()
                dx = 0 if self.align in ("leading", "fill") else (self.cw[k] - w) / 2
                it.paint(p, cx + dx, cy)
                cx += self.cw[k] + self.hs
            cy += h + self.vs


def symbol(p: QPainter, name: str, c: QPointF, size: float, color: QColor, width: float = 1.6):
    """The few SF Symbols the drawings use, drawn as paths."""
    s = size / 2
    path = QPainterPath()
    if name == "triangle":
        path.moveTo(c.x(), c.y() - s * 0.95)
        path.lineTo(c.x() + s, c.y() + s * 0.75)
        path.lineTo(c.x() - s, c.y() + s * 0.75)
        path.closeSubpath()
    elif name == "circle":
        path.addEllipse(c, s * 0.95, s * 0.95)
    elif name == "xmark":
        k = s * 0.8
        path.moveTo(c.x() - k, c.y() - k); path.lineTo(c.x() + k, c.y() + k)
        path.moveTo(c.x() + k, c.y() - k); path.lineTo(c.x() - k, c.y() + k)
    elif name == "square":
        path.addRoundedRect(QRectF(c.x() - s * 0.85, c.y() - s * 0.85, s * 1.7, s * 1.7), 1.5, 1.5)
    elif name == "arrows4":
        k, a = s, s * 0.35
        path.moveTo(c.x(), c.y() - k); path.lineTo(c.x(), c.y() + k)
        path.moveTo(c.x() - k, c.y()); path.lineTo(c.x() + k, c.y())
        for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
            tip = QPointF(c.x() + dx * k, c.y() + dy * k)
            if dx == 0:
                path.moveTo(tip.x() - a, tip.y() - dy * a); path.lineTo(tip); path.lineTo(tip.x() + a, tip.y() - dy * a)
            else:
                path.moveTo(tip.x() - dx * a, tip.y() - a); path.lineTo(tip); path.lineTo(tip.x() - dx * a, tip.y() + a)
    elif name == "gyroscope":
        path.addEllipse(c, s * 0.9, s * 0.9)
        path.addEllipse(c, s * 0.9, s * 0.35)
        path.addEllipse(c, s * 0.18, s * 0.18)
    elif name == "grid":
        d = size / 3.6
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                path.addRect(QRectF(c.x() + i * d * 1.2 - d / 2, c.y() + j * d * 1.2 - d / 2, d, d))
        p.fillPath(path, color)
        return
    p.strokePath(path, pen(color, width))
