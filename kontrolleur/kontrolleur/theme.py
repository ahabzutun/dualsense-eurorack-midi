"""Colours, fonts and text size (the paintbrush button). Same presets and settings as the Mac app."""
from __future__ import annotations

import json
import os
import sys
from typing import Dict, List, Optional, Tuple

from PySide6.QtGui import QColor, QFont, QFontDatabase, QFontInfo

from . import store

SLOTS: List[Tuple[str, str, str]] = [
    ("bg", "Background", "Surfaces"), ("panel", "Side panel & bars", "Surfaces"), ("card", "Cards", "Surfaces"),
    ("line", "Dividers & borders", "Surfaces"), ("ink", "Text", "Text & lines"), ("dim", "Secondary text", "Text & lines"),
    ("body", "Drawing lines", "Text & lines"), ("cv", "CV / main accent", "Accents"), ("trig", "Triggers", "Accents"),
    ("special", "LFO & special", "Accents"), ("octave", "Octave / pitch", "Accents"),
    ("function", "Button functions", "Accents"), ("note", "Notes", "Accents"),
]


def _c(r, g, b, a=1.0):
    return {"r": r, "g": g, "b": b, "a": a}


def _look(colors, ui="System", mono="System Mono", title="System Rounded", scale=1.0):
    return {"colors": colors, "uiFont": ui, "monoFont": mono, "titleFont": title, "scale": scale}


PRESETS = [
    ("Blueprint (dark)", _look({
        "bg": _c(0.047, 0.059, 0.078), "panel": _c(0.075, 0.090, 0.114), "card": _c(0.102, 0.122, 0.153),
        "line": _c(1, 1, 1, 0.09), "ink": _c(0.88, 0.92, 0.97), "dim": _c(1, 1, 1, 0.42),
        "body": _c(0.62, 0.74, 0.86), "cv": _c(0.27, 0.86, 0.96), "trig": _c(1.00, 0.63, 0.28),
        "special": _c(0.55, 0.94, 0.56), "octave": _c(0.76, 0.62, 1.00), "function": _c(0.98, 0.84, 0.36),
        "note": _c(0.98, 0.86, 0.55)})),
    ("Paper (light)", _look({
        "bg": _c(0.95, 0.94, 0.91), "panel": _c(0.91, 0.90, 0.87), "card": _c(0.99, 0.98, 0.96),
        "line": _c(0, 0, 0, 0.12), "ink": _c(0.11, 0.12, 0.14), "dim": _c(0, 0, 0, 0.48),
        "body": _c(0.27, 0.36, 0.48), "cv": _c(0.00, 0.47, 0.66), "trig": _c(0.84, 0.38, 0.05),
        "special": _c(0.13, 0.55, 0.20), "octave": _c(0.45, 0.30, 0.78), "function": _c(0.67, 0.50, 0.00),
        "note": _c(0.62, 0.42, 0.05)}, title="System Serif")),
    ("Amber terminal", _look({
        "bg": _c(0.04, 0.03, 0.01), "panel": _c(0.07, 0.05, 0.02), "card": _c(0.10, 0.07, 0.03),
        "line": _c(1.0, 0.70, 0.25, 0.14), "ink": _c(1.00, 0.76, 0.33), "dim": _c(1.0, 0.70, 0.30, 0.50),
        "body": _c(0.90, 0.60, 0.22), "cv": _c(1.00, 0.80, 0.35), "trig": _c(1.00, 0.48, 0.20),
        "special": _c(0.95, 0.90, 0.55), "octave": _c(1.00, 0.62, 0.45), "function": _c(1.00, 0.88, 0.50),
        "note": _c(1.00, 0.93, 0.70)}, ui="System Mono", mono="System Mono", title="System Mono")),
    ("High contrast", _look({
        "bg": _c(0, 0, 0), "panel": _c(0.05, 0.05, 0.05), "card": _c(0.10, 0.10, 0.10),
        "line": _c(1, 1, 1, 0.35), "ink": _c(1, 1, 1), "dim": _c(1, 1, 1, 0.75),
        "body": _c(1, 1, 1), "cv": _c(0.20, 0.90, 1.00), "trig": _c(1.00, 0.60, 0.10),
        "special": _c(0.40, 1.00, 0.40), "octave": _c(0.85, 0.65, 1.00), "function": _c(1.00, 0.95, 0.20),
        "note": _c(1.00, 1.00, 0.55)}, scale=1.1)),
    ("Eurorack panel", _look({
        "bg": _c(0.16, 0.16, 0.17), "panel": _c(0.20, 0.20, 0.21), "card": _c(0.25, 0.25, 0.26),
        "line": _c(1, 1, 1, 0.10), "ink": _c(0.93, 0.93, 0.91), "dim": _c(1, 1, 1, 0.50),
        "body": _c(0.80, 0.80, 0.78), "cv": _c(0.95, 0.35, 0.30), "trig": _c(1.00, 0.78, 0.20),
        "special": _c(0.35, 0.75, 0.95), "octave": _c(0.70, 0.55, 0.95), "function": _c(0.98, 0.90, 0.60),
        "note": _c(1.00, 0.85, 0.60)})),
    # after modules of the rack, picked from photos of the panels
    ("ER-301 (silver)", _look({          # silver panel, black print, blue + grey buttons, amber OLEDs, red / amber LEDs
        "bg": _c(0.78, 0.79, 0.80), "panel": _c(0.72, 0.73, 0.74), "card": _c(0.85, 0.855, 0.86),
        "line": _c(0, 0, 0, 0.17), "ink": _c(0.09, 0.09, 0.10), "dim": _c(0, 0, 0, 0.55),
        "body": _c(0.24, 0.25, 0.27), "cv": _c(0.09, 0.55, 0.82), "trig": _c(0.85, 0.44, 0.00),
        "special": _c(0.78, 0.13, 0.11), "octave": _c(0.36, 0.36, 0.60), "function": _c(0.30, 0.32, 0.35),
        "note": _c(0.62, 0.36, 0.00)})),
    ("ER-301 (black)", _look({           # black panel, white print, same buttons, OLEDs and LEDs
        "bg": _c(0.085, 0.09, 0.095), "panel": _c(0.115, 0.12, 0.125), "card": _c(0.155, 0.16, 0.165),
        "line": _c(1, 1, 1, 0.13), "ink": _c(0.95, 0.95, 0.94), "dim": _c(1, 1, 1, 0.52),
        "body": _c(0.62, 0.65, 0.68), "cv": _c(0.16, 0.68, 0.92), "trig": _c(1.00, 0.58, 0.10),
        "special": _c(0.94, 0.25, 0.22), "octave": _c(0.68, 0.70, 0.90), "function": _c(0.66, 0.70, 0.73),
        "note": _c(1.00, 0.80, 0.50)})),
    ("Percussa SSP", _look({             # navy panel, grey print, black screen with coloured tiles, red buttons
        "bg": _c(0.12, 0.135, 0.16), "panel": _c(0.10, 0.112, 0.135), "card": _c(0.04, 0.042, 0.05),
        "line": _c(1, 1, 1, 0.12), "ink": _c(0.95, 0.95, 0.96), "dim": _c(1, 1, 1, 0.50),
        "body": _c(0.56, 0.57, 0.60), "cv": _c(0.96, 0.27, 0.20), "trig": _c(0.94, 0.92, 0.34),
        "special": _c(0.33, 0.88, 0.36), "octave": _c(0.74, 0.47, 0.92), "function": _c(0.36, 0.85, 0.95),
        "note": _c(1.00, 0.60, 0.36)})),
    ("Teletype (silver)", _look({        # brushed silver panel, dark grey print, chartreuse OLED
        "bg": _c(0.73, 0.74, 0.75), "panel": _c(0.67, 0.68, 0.69), "card": _c(0.80, 0.805, 0.81),
        "line": _c(0, 0, 0, 0.16), "ink": _c(0.15, 0.15, 0.16), "dim": _c(0, 0, 0, 0.52),
        "body": _c(0.26, 0.27, 0.28), "cv": _c(0.36, 0.48, 0.00), "trig": _c(0.72, 0.38, 0.00),
        "special": _c(0.06, 0.40, 0.44), "octave": _c(0.40, 0.30, 0.56), "function": _c(0.20, 0.20, 0.22),
        "note": _c(0.42, 0.44, 0.00)}, ui="System Mono", title="System Mono")),
    ("Teletype (black)", _look({         # black panel, white print, silver jacks, amber OLED
        "bg": _c(0.065, 0.065, 0.07), "panel": _c(0.09, 0.09, 0.095), "card": _c(0.03, 0.03, 0.032),
        "line": _c(1, 1, 1, 0.13), "ink": _c(0.94, 0.94, 0.93), "dim": _c(1, 1, 1, 0.50),
        "body": _c(0.76, 0.77, 0.78), "cv": _c(1.00, 0.72, 0.00), "trig": _c(1.00, 0.48, 0.10),
        "special": _c(0.80, 0.90, 0.20), "octave": _c(1.00, 0.86, 0.56), "function": _c(0.74, 0.77, 0.80),
        "note": _c(1.00, 0.90, 0.66)}, ui="System Mono", title="System Mono")),
]

BUILTIN_FONTS = ["System", "System Mono", "System Rounded", "System Serif"]

# font weights as in SwiftUI
REGULAR, MEDIUM, SEMIBOLD, BOLD, HEAVY = 400, 500, 600, 700, 800


class Appearance:
    def __init__(self):
        self.revision = 0
        self.snapshot_only = False
        raw = store.settings().value("appearance")
        try:
            self.data = json.loads(raw) if raw else json.loads(json.dumps(PRESETS[0][1]))
        except Exception:
            self.data = json.loads(json.dumps(PRESETS[0][1]))
        env = os.environ
        if env.get("KONTROLLEUR_PRESET", "").isdigit() and int(env["KONTROLLEUR_PRESET"]) < len(PRESETS):
            self.snapshot_only = True
            self.data = json.loads(json.dumps(PRESETS[int(env["KONTROLLEUR_PRESET"])][1]))
        if env.get("KONTROLLEUR_SCALE"):
            self.snapshot_only = True
            self.data["scale"] = float(env["KONTROLLEUR_SCALE"])
        if env.get("KONTROLLEUR_FONT"):
            self.snapshot_only = True
            for k in ("uiFont", "monoFont", "titleFont"):
                self.data[k] = env["KONTROLLEUR_FONT"]
        self._families: Dict[str, str] = {}
        self._cache: Dict[tuple, QFont] = {}

    def set(self, data: dict):
        if data == self.data:
            return
        self.data = data
        self.revision += 1
        self._cache.clear()
        if not self.snapshot_only:
            store.settings().setValue("appearance", json.dumps(data))

    def update(self, **kw):
        d = json.loads(json.dumps(self.data))
        d.update(kw)
        self.set(d)

    def set_color(self, slot: str, c: QColor):
        d = json.loads(json.dumps(self.data))
        d["colors"][slot] = _c(c.redF(), c.greenF(), c.blueF(), c.alphaF())
        self.set(d)

    # colours
    def color(self, slot: str, opacity: float = 1.0) -> QColor:
        v = self.data["colors"].get(slot) or PRESETS[0][1]["colors"].get(slot) or _c(0.5, 0.5, 0.5)
        q = QColor.fromRgbF(v["r"], v["g"], v["b"], max(0.0, min(1.0, v["a"] * opacity)))
        return q

    @property
    def is_dark(self) -> bool:
        v = self.data["colors"].get("bg") or _c(0, 0, 0)
        return 0.2126 * v["r"] + 0.7152 * v["g"] + 0.0722 * v["b"] < 0.5

    @property
    def scale(self) -> float:
        return float(self.data.get("scale", 1.0))

    # fonts
    def _family(self, name: str) -> Tuple[str, Optional[QFont.StyleHint]]:
        if name == "System":
            return QFontDatabase.systemFont(QFontDatabase.GeneralFont).family(), None
        if name == "System Mono":
            return QFontDatabase.systemFont(QFontDatabase.FixedFont).family(), QFont.Monospace
        if name == "System Rounded":
            fams = set(QFontDatabase.families())
            for f in ("SF Pro Rounded", "Nunito", "Varela Round", "Quicksand", "M PLUS Rounded 1c", "Ubuntu"):
                if f in fams:
                    return f, None
            return QFontDatabase.systemFont(QFontDatabase.GeneralFont).family(), None
        if name == "System Serif":
            fams = set(QFontDatabase.families())
            for f in ("New York", "Georgia", "Noto Serif", "DejaVu Serif", "Liberation Serif", "Times New Roman"):
                if f in fams:
                    return f, QFont.Serif
            return "Serif", QFont.Serif
        return name, None

    def font(self, size: float, weight: int = REGULAR, role: str = "ui", italic: bool = False) -> QFont:
        key = (size, weight, role, italic)
        f = self._cache.get(key)
        if f is not None:
            return QFont(f)
        name = self.data["monoFont"] if role == "mono" else self.data["titleFont"] if role == "title" else self.data["uiFont"]
        fam, hint = self._family(name)
        f = QFont(fam)
        if hint is not None:
            f.setStyleHint(hint)
        f.setPixelSize(max(1, round(size * self.scale)))
        f.setWeight(QFont.Weight(weight))
        f.setItalic(italic)
        self._cache[key] = f
        return QFont(f)

    def _arrows(self) -> dict:
        """Small SVG arrows in the theme colours for spin boxes and drop-downs."""
        d = store.data_dir() / "ui"
        d.mkdir(exist_ok=True)
        out = {}
        for name, col in (("ink", self.color("ink")), ("dim", self.color("dim", 0.6))):
            hexc = col.name()
            op = f"{col.alphaF():.2f}"
            for kind, pts in (("up", "1,7 5,2 9,7"), ("down", "1,3 5,8 9,3")):
                f = d / f"{kind}-{hexc[1:]}-{op}.svg"
                if not f.exists():
                    f.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="10" height="10" viewBox="0 0 10 10">'
                                 f'<polyline points="{pts}" fill="none" stroke="{hexc}" stroke-opacity="{op}" stroke-width="1.6" '
                                 f'stroke-linecap="round" stroke-linejoin="round"/></svg>')
                out[f"{kind}-{name}"] = f.as_posix()
        return out

    def stylesheet(self) -> str:
        c = lambda s, o=1.0: _css(self.color(s, o))   # noqa: E731
        ar = self._arrows()
        field = "rgba(0,0,0,0.35)" if self.is_dark else "rgba(255,255,255,0.75)"
        ui = self.font(12)
        fam = ui.family()
        px = ui.pixelSize()
        return f"""
        QWidget {{ color: {c('ink')}; }}
        QMainWindow, #root {{ background: {c('bg')}; }}
        #panel, #bar, #status {{ background: {c('panel')}; }}
        QScrollArea, QScrollArea > QWidget > QWidget {{ background: transparent; border: none; }}
        QToolTip {{ background: {c('card')}; color: {c('ink')}; border: 1px solid {c('line')}; padding: 4px; }}
        #card {{ background: {c('card')}; border: 1px solid {c('line')}; border-radius: 12px; }}
        #divider {{ background: {c('line')}; }}
        QLineEdit, QSpinBox, QDoubleSpinBox, QPlainTextEdit {{
            background: {field}; border: 1px solid {c('line')}; border-radius: 6px; padding: 3px 6px;
            selection-background-color: {c('cv', 0.45)}; }}
        QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QPlainTextEdit:focus {{ border: 1px solid {c('cv', 0.8)}; }}
        QAbstractSpinBox {{ padding-right: 18px; }}
        QAbstractSpinBox::up-button {{ subcontrol-origin: border; subcontrol-position: top right; width: 16px;
            border: none; background: transparent; margin: 2px 2px 0 0; }}
        QAbstractSpinBox::down-button {{ subcontrol-origin: border; subcontrol-position: bottom right; width: 16px;
            border: none; background: transparent; margin: 0 2px 2px 0; }}
        QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover {{ background: {c('cv', 0.2)}; border-radius: 3px; }}
        QAbstractSpinBox::up-arrow {{ image: url("{ar['up-ink']}"); width: 8px; height: 8px; }}
        QAbstractSpinBox::down-arrow {{ image: url("{ar['down-ink']}"); width: 8px; height: 8px; }}
        QAbstractSpinBox::up-arrow:disabled, QAbstractSpinBox::up-arrow:off {{ image: url("{ar['up-dim']}"); }}
        QAbstractSpinBox::down-arrow:disabled, QAbstractSpinBox::down-arrow:off {{ image: url("{ar['down-dim']}"); }}
        QComboBox {{ background: {c('card')}; border: 1px solid {c('line')}; border-radius: 6px; padding: 3px 22px 3px 8px; }}
        QComboBox::drop-down {{ subcontrol-origin: padding; subcontrol-position: center right; width: 20px; border: none; }}
        QComboBox::down-arrow {{ image: url("{ar['down-ink']}"); width: 9px; height: 9px; }}
        QComboBox:hover {{ border: 1px solid {c('cv', 0.6)}; }}
        QComboBox QAbstractItemView {{ background: {c('card')}; border: 1px solid {c('line')};
            selection-background-color: {c('cv', 0.35)}; selection-color: {c('ink')}; outline: none; }}
        QPushButton, QToolButton {{ background: {c('card')}; border: 1px solid {c('line')}; border-radius: 6px; padding: 4px 10px; }}
        QToolButton::menu-indicator {{ image: none; width: 0; }}
        QPushButton:hover, QToolButton:hover {{ border: 1px solid {c('cv', 0.6)}; }}
        QPushButton:pressed, QToolButton:pressed {{ background: {c('cv', 0.25)}; }}
        QPushButton:disabled, QToolButton:disabled {{ color: {c('dim', 0.7)}; }}
        QPushButton#primary {{ background: {c('cv', 0.85)}; color: {c('bg')}; font-weight: 700; border: none; }}
        QPushButton#primary:disabled {{ background: {c('cv', 0.35)}; }}
        QPushButton#small {{ padding: 2px 7px; font-size: {max(9, round(px * 0.9))}px; }}
        QPushButton#flat, QToolButton#flat {{ background: transparent; border: none; padding: 2px; }}
        QPushButton[seg="true"] {{ border-radius: 0; border: 1px solid {c('line')}; border-left: none; padding: 4px 10px; }}
        QPushButton[seg="true"][first="true"] {{ border-top-left-radius: 6px; border-bottom-left-radius: 6px; border-left: 1px solid {c('line')}; }}
        QPushButton[seg="true"][last="true"] {{ border-top-right-radius: 6px; border-bottom-right-radius: 6px; }}
        QPushButton[seg="true"]:hover {{ border: 1px solid {c('line')}; background: {c('cv', 0.12)}; }}
        QPushButton[seg="true"]:checked {{ background: {c('cv', 0.28)}; color: {c('ink')}; font-weight: 600; }}
        QMenu {{ background: {c('card')}; border: 1px solid {c('line')}; padding: 4px; }}
        QMenu::item {{ padding: 5px 18px; border-radius: 4px; }}
        QMenu::item:selected {{ background: {c('cv', 0.3)}; }}
        QMenu::separator {{ height: 1px; background: {c('line')}; margin: 4px 6px; }}
        QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
        QScrollBar::handle:vertical {{ background: {c('dim', 0.5)}; border-radius: 3px; min-height: 30px; }}
        QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
        QScrollBar:horizontal {{ height: 0; }}
        QSlider::groove:horizontal {{ height: 4px; background: {c('line')}; border-radius: 2px; }}
        QSlider::sub-page:horizontal {{ background: {c('cv', 0.8)}; border-radius: 2px; }}
        QSlider::handle:horizontal {{ background: {c('ink')}; width: 14px; height: 14px; margin: -6px 0; border-radius: 7px; }}
        QCheckBox {{ spacing: 8px; }}
        QDialog {{ background: {c('panel')}; }}
        QGroupBox {{ border: 1px solid {c('line')}; border-radius: 8px; margin-top: 14px; padding: 8px; }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 8px; color: {c('dim')}; }}
        """


def _css(q: QColor) -> str:
    return f"rgba({q.red()},{q.green()},{q.blue()},{q.alphaF():.3f})"


A = Appearance.__new__(Appearance)   # replaced by init() once a QApplication exists


def init() -> Appearance:
    global A
    A = Appearance()
    return A


class _Theme:
    """Theme.cv, Theme.font(...) — reads the current appearance."""

    def __getattr__(self, name):
        return A.color(name)

    def c(self, slot: str, opacity: float = 1.0) -> QColor:
        return A.color(slot, opacity)

    def font(self, size: float, weight: int = REGULAR, role: str = "ui", italic: bool = False) -> QFont:
        return A.font(size, weight, role, italic)

    @property
    def grow(self) -> float:
        """Text-size factor for drawings: boxes that hold text grow with it (never below the design size)."""
        return max(1.0, A.scale)

    def role_color(self, r: str) -> QColor:
        from . import model as M
        if r == M.OFF:
            return A.color("dim")
        if r == M.CV:
            return A.color("cv")
        if r in (M.TRIGGER, M.NOTE_TRIGGER):
            return A.color("trig")
        if r in (M.LFO_RATE, M.LFO_RESET):
            return A.color("special")
        return A.color("octave")

    def token_color(self, k: tuple) -> QColor:
        if k[0] == "off":
            return A.color("dim")
        if k[0] == "route":
            return self.role_color(k[1])
        return A.color("function")


Theme = _Theme()
