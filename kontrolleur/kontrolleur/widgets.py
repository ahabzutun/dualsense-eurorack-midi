"""Building blocks of the side panels (cards, number fields, switches, pickers, notes)."""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence, Tuple

from PySide6.QtCore import QRectF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QTextOption
from PySide6.QtWidgets import (QAbstractButton, QAbstractSpinBox, QComboBox, QDoubleSpinBox, QFrame, QHBoxLayout,
                               QLabel, QLineEdit, QPlainTextEdit, QPushButton, QScrollArea, QSizePolicy, QSpinBox,
                               QVBoxLayout, QWidget)

from .theme import Theme, BOLD, HEAVY, MEDIUM, SEMIBOLD, REGULAR


def css(q: QColor) -> str:
    return f"rgba({q.red()},{q.green()},{q.blue()},{q.alphaF():.3f})"


def label(text: str, size: float = 12, weight: int = REGULAR, role: str = "ui", color: Optional[QColor] = None,
          wrap: bool = False, tracking: float = 0.0, italic: bool = False) -> QLabel:
    lb = QLabel(text)
    f = Theme.font(size, weight, role, italic)
    if tracking:
        from PySide6.QtGui import QFont
        f.setLetterSpacing(QFont.AbsoluteSpacing, tracking)
    lb.setFont(f)
    lb.setStyleSheet(f"color: {css(color or Theme.ink)}; background: transparent;")
    lb.setWordWrap(wrap)
    if wrap:
        lb.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)
    lb.setTextInteractionFlags(Qt.NoTextInteraction)
    return lb


def title(text: str) -> QLabel:
    return label(text, 24, BOLD, "title")


def hint(text: str) -> QLabel:
    return label(text, 11, REGULAR, "ui", Theme.dim, wrap=True)


def divider() -> QFrame:
    d = QFrame()
    d.setObjectName("divider")
    d.setFixedHeight(1)
    return d


def vdivider() -> QFrame:
    d = QFrame()
    d.setObjectName("divider")
    d.setFixedWidth(1)
    return d


class Card(QFrame):
    def __init__(self, title_text: str, accent: Optional[QColor] = None):
        super().__init__()
        self.setObjectName("card")
        self.lay = QVBoxLayout(self)
        self.lay.setContentsMargins(14, 14, 14, 14)
        self.lay.setSpacing(10)
        acc = QColor(accent or Theme.ink)
        acc.setAlphaF(acc.alphaF() * 0.9)
        self.lay.addWidget(label(title_text.upper(), 10, HEAVY, "mono", acc, tracking=1.2))

    def add(self, w):
        if isinstance(w, QWidget):
            self.lay.addWidget(w)
        else:
            self.lay.addLayout(w)
        return w


def row(title_text: str, widget: QWidget, weight: int = MEDIUM) -> QWidget:
    w = QWidget()
    h = QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(8)
    lb = label(title_text, 12, weight)
    lb.setWordWrap(True)
    h.addWidget(lb, 1)
    h.addWidget(widget, 0, Qt.AlignRight)
    return w


def hbox(*widgets, spacing: int = 6, stretch_end: bool = False) -> QWidget:
    w = QWidget()
    h = QHBoxLayout(w)
    h.setContentsMargins(0, 0, 0, 0)
    h.setSpacing(spacing)
    for x in widgets:
        if x == "stretch":
            h.addStretch(1)
        else:
            h.addWidget(x)
    if stretch_end:
        h.addStretch(1)
    return w


class NumberField(QSpinBox):
    def __init__(self, value: int, lo: int, hi: int, on_change: Callable[[int], None], width: int = 46):
        super().__init__()
        self.setRange(lo, hi)
        self.setValue(max(lo, min(hi, value)))
        self.setAlignment(Qt.AlignRight)
        self.setFont(Theme.font(13, SEMIBOLD, "mono"))
        self.setButtonSymbols(QAbstractSpinBox.UpDownArrows)
        self.setFixedWidth(int(width * Theme.grow) + 34)
        self.setKeyboardTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.valueChanged.connect(on_change)

    def wheelEvent(self, ev):      # don't change values while scrolling the panel
        if self.hasFocus():
            super().wheelEvent(ev)
        else:
            ev.ignore()


class DecimalField(QDoubleSpinBox):
    def __init__(self, value: float, lo: float, hi: float, on_change: Callable[[float], None], width: int = 70):
        super().__init__()
        self.setDecimals(2)
        self.setRange(lo, hi)
        self.setValue(value)
        self.setAlignment(Qt.AlignRight)
        self.setFont(Theme.font(13, SEMIBOLD, "mono"))
        self.setSingleStep(0.05)
        self.setFixedWidth(int(width * Theme.grow) + 34)
        self.setFocusPolicy(Qt.StrongFocus)
        self.valueChanged.connect(on_change)

    def wheelEvent(self, ev):
        if self.hasFocus():
            super().wheelEvent(ev)
        else:
            ev.ignore()


class Picker(QComboBox):
    def __init__(self, options: Sequence[Tuple[object, str]], current, on_change: Callable, width: Optional[int] = None):
        super().__init__()
        self._values = [v for v, _ in options]
        for _, t in options:
            self.addItem(t)
        if current in self._values:
            self.setCurrentIndex(self._values.index(current))
        if width:
            self.setFixedWidth(int(width * Theme.grow))
        self.setFocusPolicy(Qt.StrongFocus)
        self.currentIndexChanged.connect(lambda k: on_change(self._values[k]) if 0 <= k < len(self._values) else None)

    def wheelEvent(self, ev):
        if self.hasFocus() and self.view().isVisible():
            super().wheelEvent(ev)
        else:
            ev.ignore()


class Segmented(QWidget):
    """A row of buttons, one selected (SwiftUI segmented picker)."""

    def __init__(self, options: Sequence[Tuple[object, str]], current, on_change: Callable, tips: Optional[dict] = None):
        super().__init__()
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(0)
        self.buttons = []
        for k, (v, t) in enumerate(options):
            b = QPushButton(t.replace("&", "&&"))
            b.setCheckable(True)
            b.setChecked(v == current)
            b.setProperty("seg", True)
            b.setProperty("first", k == 0)
            b.setProperty("last", k == len(options) - 1)
            b.setFocusPolicy(Qt.NoFocus)
            b.setCursor(Qt.PointingHandCursor)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            if tips and v in tips:
                b.setToolTip(tips[v])
            b.clicked.connect(lambda _=False, v=v: self._pick(v, on_change))
            h.addWidget(b)
            self.buttons.append((v, b))

    def _pick(self, v, on_change):
        for vv, b in self.buttons:
            b.setChecked(vv == v)
        on_change(v)

    def set_current(self, v):
        for vv, b in self.buttons:
            b.setChecked(vv == v)


class Switch(QAbstractButton):
    def __init__(self, checked: bool = False):
        super().__init__()
        self.setCheckable(True)
        self.setChecked(checked)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedSize(38, 22)
        self.setFocusPolicy(Qt.NoFocus)

    def sizeHint(self):
        return QSize(38, 22)

    def paintEvent(self, ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(1, 1, self.width() - 2, self.height() - 2)
        on = self.isChecked()
        track = Theme.c("cv", 0.85 if self.isEnabled() else 0.35) if on else Theme.c("dim", 0.35)
        path = QPainterPath()
        path.addRoundedRect(r, r.height() / 2, r.height() / 2)
        p.fillPath(path, track)
        d = r.height() - 4
        x = r.right() - d - 2 if on else r.left() + 2
        p.setBrush(QColor(255, 255, 255, 240 if self.isEnabled() else 140))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QRectF(x, r.top() + 2, d, d))


def switch_row(text: str, checked: bool, on_change: Callable[[bool], None], enabled: bool = True) -> QWidget:
    sw = Switch(checked)
    sw.setEnabled(enabled)
    sw.toggled.connect(on_change)
    w = row(text, sw, REGULAR)
    if not enabled:
        w.setToolTip("Not supported by this device's firmware")
    return w


class NoteField(QWidget):
    """Note editor used in all inspectors."""

    def __init__(self, text: str, on_change: Callable[[str], None], title_text: str = "Note",
                 prompt: str = "What is this for? e.g. “grain duration on the ER-301”"):
        super().__init__()
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(5)
        v.addWidget(label("✎  " + title_text, 10, HEAVY, "mono", Theme.note, tracking=1))
        self.edit = QPlainTextEdit(text)
        self.edit.setPlaceholderText(prompt)
        self.edit.setFont(Theme.font(12))
        self.edit.setWordWrapMode(QTextOption.WrapAtWordBoundaryOrAnywhere)
        self.edit.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.edit.setTabChangesFocus(True)
        self._on_change = on_change
        self._style(bool(text.strip()))
        self.edit.textChanged.connect(self._changed)
        self.edit.document().documentLayout().documentSizeChanged.connect(lambda _: self._fit())
        v.addWidget(self.edit)
        QTimer.singleShot(0, self._fit)

    def _style(self, filled: bool):
        self.edit.setStyleSheet(f"QPlainTextEdit {{ background: {css(Theme.c('bg', 0.7))}; border: 1px solid "
                                f"{css(Theme.c('note', 0.45 if filled else 0.15))}; border-radius: 6px; padding: 3px 4px; }}")

    def _fit(self):
        lh = self.edit.fontMetrics().lineSpacing()
        lines = max(1, min(5, int(self.edit.document().size().height())))
        self.edit.setFixedHeight(int(lh * lines + 14))

    def _changed(self):
        t = self.edit.toPlainText()
        self._style(bool(t.strip()))
        self._on_change(t)


class NameField(QLineEdit):
    """4-character name (EC4 display)."""

    def __init__(self, text: str, on_change: Callable[[str], None]):
        super().__init__(text.strip())
        self.setMaxLength(4)
        self.setAlignment(Qt.AlignCenter)
        self.setFont(Theme.font(13, BOLD, "mono"))
        self.setFixedWidth(int(70 * Theme.grow))
        self.textEdited.connect(lambda t: on_change(t[:4]))


def small_button(text: str, on_click: Callable, enabled: bool = True) -> QPushButton:
    b = QPushButton(text)
    b.setObjectName("small")
    b.setEnabled(enabled)
    b.setCursor(Qt.PointingHandCursor)
    b.clicked.connect(lambda: on_click())
    return b


def icon_button(text: str, on_click: Callable, tip: str = "") -> QPushButton:
    b = QPushButton(text)
    b.setObjectName("flat")
    b.setToolTip(tip)
    b.setCursor(Qt.PointingHandCursor)
    b.clicked.connect(lambda: on_click())
    return b


class Inspector(QWidget):
    """Side panel: segmented tabs on top, a scrolling column of cards below. `build(column)` fills it;
    `rebuild()` redraws it (deferred, so a widget may trigger it from its own signal)."""

    def __init__(self, tabs: Sequence[str], build: Callable[[str, QVBoxLayout], None], tab: Optional[str] = None):
        super().__init__()
        self.setObjectName("panel")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setFixedWidth(int(384 * Theme.grow))
        self._build = build
        self.tabs = list(tabs)
        self.tab = tab or (self.tabs[0] if self.tabs else "")
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        self.seg = None
        if len(self.tabs) > 1:
            top = QWidget()
            tl = QVBoxLayout(top)
            tl.setContentsMargins(14, 14, 14, 14)
            self.seg = Segmented([(t, t) for t in self.tabs], self.tab, self.set_tab)
            tl.addWidget(self.seg)
            v.addWidget(top)
            v.addWidget(divider())
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setFrameShape(QFrame.NoFrame)
        v.addWidget(self.scroll, 1)
        self._pending = False
        self._fill()

    def set_tab(self, t: str):
        self.tab = t
        if self.seg:
            self.seg.set_current(t)
        self.rebuild()

    def rebuild(self):
        if not self._pending:
            self._pending = True
            QTimer.singleShot(0, self._fill)

    def _fill(self):
        self._pending = False
        self.setFixedWidth(int(384 * Theme.grow))
        pos = self.scroll.verticalScrollBar().value()
        content = QWidget()
        col = QVBoxLayout(content)
        col.setContentsMargins(14, 14, 14, 14)
        col.setSpacing(12)
        self._build(self.tab, col)
        col.addStretch(1)
        old = self.scroll.takeWidget()
        self.scroll.setWidget(content)
        if old is not None:
            old.deleteLater()
        QTimer.singleShot(0, lambda: self.scroll.verticalScrollBar().setValue(pos))
