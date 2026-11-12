"""
ZoomStage: shows a drawing made for a fixed size, fitted and centred in the available space.
Zoom: pinch (trackpad), mouse / trackball scroll wheel (towards the pointer), Ctrl or Alt + scroll,
the buttons in the corner or Ctrl+ / Ctrl− / Ctrl0. Pan: drag, or two-finger scroll on a trackpad.
"""
from __future__ import annotations

import math
import os
from typing import Optional, Tuple

from PySide6.QtCore import QEvent, QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QCursor, QPainter, QTransform
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QToolTip, QWidget, QFrame

from . import draw
from .theme import Theme, A

MAX_ZOOM = 6.0


class Drawing:
    """Something drawn on the stage. Fixed drawings paint in `base` coordinates; growing drawings
    return a layout item from `content()` and get a stage at least that big, centred."""
    base: Tuple[float, float] = (1100.0, 640.0)
    grows = False

    def __init__(self):
        self.hits = draw.Hits()
        self.stage = None   # set by the Stage

    def content(self):
        return None

    def paint(self, p: QPainter, w: float, h: float):
        pass

    def update(self):
        if self.stage is not None:
            self.stage.update()


class Stage(QWidget):
    def __init__(self, drawing: Drawing, parent=None):
        super().__init__(parent)
        self.drawing = drawing
        drawing.stage = self
        self.zoom = float(os.environ.get("KONTROLLEUR_ZOOM", "1") or 1)
        self.offset = QPointF(0, 0)
        self._press: Optional[QPointF] = None
        self._press_offset = QPointF(0, 0)
        self._dragging = False
        self._xf = QTransform()
        self._stage_size = drawing.base
        self.setMouseTracking(True)
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.setMinimumSize(320, 240)
        self._controls()

    # ── geometry ──
    def _sizes(self):
        item = self.drawing.content() if self.drawing.grows else None
        bw, bh = self.drawing.base
        if item is not None:
            iw, ih = item.size()
            sw, sh = max(bw, iw + 60), max(bh, ih + 60)
        else:
            sw, sh = bw, bh
        return item, sw, sh

    def _fit(self, sw, sh) -> float:
        return max(0.05, min(self.width() / sw, self.height() / sh))

    def _clamp(self, o: QPointF, z: float) -> QPointF:
        sw, sh = self._stage_size
        fit = self._fit(sw, sh)
        w, h = sw * fit * z, sh * fit * z
        mx = max(0.0, (w - self.width()) / 2 + 40)
        my = max(0.0, (h - self.height()) / 2 + 40)
        return QPointF(min(mx, max(-mx, o.x())), min(my, max(-my, o.y())))

    # ── painting ──
    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)
        p.fillRect(self.rect(), Theme.bg)
        item, sw, sh = self._sizes()
        self._stage_size = (sw, sh)
        fit = self._fit(sw, sh)
        s = fit * self.zoom
        xf = QTransform()
        xf.translate(self.width() / 2 + self.offset.x(), self.height() / 2 + self.offset.y())
        xf.scale(s, s)
        xf.translate(-sw / 2, -sh / 2)
        self._xf = xf
        p.setTransform(xf)
        p.setClipRect(QRectF(0, 0, sw, sh))
        draw.grid(p, sw, sh)
        self.drawing.hits.clear()
        if item is not None:
            iw, ih = item.size()
            self.drawing.paint_item(p, item, (sw - iw) / 2, (sh - ih) / 2, sw, sh)
        else:
            self.drawing.paint(p, sw, sh)
        p.end()
        self._zoom_label.setText(f"{round(self.zoom * 100)}%")
        self._out.setEnabled(self.zoom > 1.001)
        self._in.setEnabled(self.zoom < MAX_ZOOM - 0.001)

    def to_drawing(self, pos: QPointF) -> QPointF:
        inv, ok = self._xf.inverted()
        return inv.map(pos) if ok else pos

    # ── mouse ──
    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._press = ev.position()
            self._press_offset = QPointF(self.offset)
            self._dragging = False

    def mouseMoveEvent(self, ev):
        if self._press is not None and ev.buttons() & Qt.LeftButton:
            d = ev.position() - self._press
            if not self._dragging and math.hypot(d.x(), d.y()) >= 4 and self.zoom > 1.001:
                self._dragging = True
                self.setCursor(Qt.ClosedHandCursor)
            if self._dragging:
                self.offset = self._clamp(self._press_offset + d, self.zoom)
                self.update()
            return
        cb, tip = self.drawing.hits.at(self.to_drawing(ev.position()))
        self.setCursor(Qt.PointingHandCursor if cb else Qt.ArrowCursor)
        if tip:
            QToolTip.showText(ev.globalPosition().toPoint(), tip, self)
        else:
            QToolTip.hideText()

    def mouseReleaseEvent(self, ev):
        if ev.button() != Qt.LeftButton or self._press is None:
            return
        was_drag = self._dragging
        self._press, self._dragging = None, False
        self.setCursor(Qt.ArrowCursor)
        if not was_drag:
            cb, _ = self.drawing.hits.at(self.to_drawing(ev.position()))
            if cb:
                cb()

    def wheelEvent(self, ev):
        mods = ev.modifiers()
        zoom_key = bool(mods & (Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier))
        pix = ev.pixelDelta()
        if not pix.isNull() and not zoom_key:
            # trackpad / smooth scrolling: pan while zoomed in
            if self.zoom > 1.001:
                self.offset = self._clamp(self.offset + QPointF(pix.x(), pix.y()), self.zoom)
                self.update()
            ev.accept()
            return
        steps = ev.angleDelta().y() / 120.0 if ev.angleDelta().y() else ev.angleDelta().x() / 120.0
        if not pix.isNull() and zoom_key:
            steps = pix.y() / 40.0
        if steps:
            self._zoom_at(ev.position(), self.zoom * (1.12 ** steps))
        ev.accept()

    def event(self, ev):
        if ev.type() == QEvent.NativeGesture and ev.gestureType() == Qt.ZoomNativeGesture:
            self._zoom_at(ev.position(), self.zoom * (1.0 + ev.value()))
            return True
        return super().event(ev)

    def _zoom_at(self, pos: QPointF, z: float):
        z = min(MAX_ZOOM, max(1.0, z))
        a = QPointF(pos.x() - self.width() / 2, pos.y() - self.height() / 2)
        r = z / self.zoom
        o = QPointF(a.x() - (a.x() - self.offset.x()) * r, a.y() - (a.y() - self.offset.y()) * r)
        self.zoom = z
        self.offset = self._clamp(o, z)
        self.update()

    def step(self, factor: float):
        z = min(MAX_ZOOM, max(1.0, self.zoom * factor))
        r = z / self.zoom
        self.zoom = z
        self.offset = self._clamp(QPointF(self.offset.x() * r, self.offset.y() * r), z)
        self.update()

    def reset(self):
        self.zoom, self.offset = 1.0, QPointF(0, 0)
        self.update()

    # ── zoom buttons in the corner ──
    def _controls(self):
        box = QFrame(self)
        box.setObjectName("zoombox")
        lay = QHBoxLayout(box)
        lay.setContentsMargins(8, 3, 8, 3)
        lay.setSpacing(2)
        self._out = QPushButton("−")
        self._zoom_label = QPushButton("100%")
        self._in = QPushButton("+")
        for b, tip in ((self._out, "Zoom out (Ctrl −)"), (self._zoom_label, "Fit to window (Ctrl 0)"),
                       (self._in, "Zoom in (Ctrl +). Pinch on a trackpad or use the scroll wheel to zoom where you point; drag to move around.")):
            b.setObjectName("flat")
            b.setToolTip(tip)
            b.setCursor(Qt.PointingHandCursor)
            b.setFocusPolicy(Qt.NoFocus)
            lay.addWidget(b)
        self._out.clicked.connect(lambda: self.step(1 / 1.25))
        self._in.clicked.connect(lambda: self.step(1.25))
        self._zoom_label.clicked.connect(self.reset)
        self._box = box
        self.restyle()

    def restyle(self):
        c = lambda s, o=1.0: draw_css(Theme.c(s, o))   # noqa: E731
        self._box.setStyleSheet(f"#zoombox {{ background: {c('panel', 0.92)}; border: 1px solid {c('line')}; border-radius: 13px; }}"
                                f"QPushButton {{ font-family: '{Theme.font(11, 600, 'mono').family()}'; font-weight: 600; min-width: 18px; }}")
        self._zoom_label.setMinimumWidth(44)
        self._box.adjustSize()

    def resizeEvent(self, ev):
        self._box.adjustSize()
        self._box.move(self.width() - self._box.width() - 10, self.height() - self._box.height() - 10)
        self.offset = self._clamp(self.offset, self.zoom)
        super().resizeEvent(ev)


def draw_css(q: QColor) -> str:
    return f"rgba({q.red()},{q.green()},{q.blue()},{q.alphaF():.3f})"


class GrowingDrawing(Drawing):
    """Flow layout drawings (EC4, ER-301): content() builds the item tree, paint_item draws it and
    registers hit areas the items collected while laying out."""
    grows = True

    def paint_item(self, p: QPainter, item, x: float, y: float, sw: float, sh: float):
        item.paint(p, x, y)
