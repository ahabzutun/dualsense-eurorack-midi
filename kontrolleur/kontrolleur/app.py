"""Kontrolleur main window."""
from __future__ import annotations

import datetime as dt
import json
import os
import plistlib
import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from PySide6.QtCore import QPoint, QSize, Qt, QTimer
from PySide6.QtGui import QAction, QColor, QFont, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QCheckBox, QColorDialog, QComboBox, QDialog, QDialogButtonBox, QFileDialog,
                               QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMenu, QMessageBox,
                               QPushButton, QScrollArea, QSizePolicy, QSlider, QStackedWidget, QToolButton, QVBoxLayout,
                               QWidget)

from . import clients, store, theme
from . import widgets as W
from .devices import EC4Board, Faderbank, PedalBoard
from .model import Mapping
from .stage import Stage
from .theme import BOLD, HEAVY, SEMIBOLD, REGULAR, Theme

MODES = [("dualsense", "DualSense"), ("faderbank", "16n"), ("pedals", "Pedals"), ("ec4", "EC4"), ("er301", "ER-301")]
ROUTED = ("faderbank", "pedals", "ec4")
ROUTE_KEY = {"faderbank": "fader16OnPi", "pedals": "pedalsOnPi", "ec4": "ec4OnPi"}
ICON = Path(__file__).resolve().parent / "icon.png"


class Status:
    def __init__(self, kind: str, text: str):
        self.kind, self.text, self.time = kind, text, dt.datetime.now()


class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kontrolleur")
        if ICON.exists():
            self.setWindowIcon(QIcon(str(ICON)))
        self.s = store.settings()
        self.mapping, self.bank, self.pedals, self.ec4 = Mapping(), Faderbank(), PedalBoard(), EC4Board()
        self.notes = store.NotesStore()
        self.offline = os.environ.get("KONTROLLEUR_OFFLINE")
        self.busy = False
        self.status = Status("info", "Loading …")
        self.synced = self.synced_bank = self.synced_pedals = self.synced_ec4 = None
        # selections
        self.ds_selected = os.environ.get("KONTROLLEUR_SELECT", "cross")
        self.fader_sel = 0
        self.pedal_sel = 0
        self.ec4_setup = self.ec4_group = self.ec4_enc = 0
        self.er_port: Optional[str] = None
        self.mode = os.environ.get("KONTROLLEUR_MODE") or self.s.value("deviceMode", "dualsense")
        if self.mode not in dict(MODES):
            self.mode = "dualsense"

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        self.bar = QWidget()
        self.bar.setObjectName("bar")
        self.bar.setAttribute(Qt.WA_StyledBackground, True)
        v.addWidget(self.bar)
        v.addWidget(W.divider())
        self.pages = QStackedWidget()
        v.addWidget(self.pages, 1)
        v.addWidget(W.divider())
        self.status_bar = QWidget()
        self.status_bar.setObjectName("status")
        self.status_bar.setAttribute(Qt.WA_StyledBackground, True)
        v.addWidget(self.status_bar)

        self._build_pages()
        self._build_bar()
        self._build_status()
        self._shortcuts()
        self.apply_appearance()
        self.resize(1400, 840)
        self.setMinimumSize(960, 640)
        QTimer.singleShot(0, self._startup)

    # ── settings ──
    def get(self, key, default):
        v = self.s.value(key, default)
        if isinstance(default, bool):
            return v in (True, "true", "1", 1)
        return v

    @property
    def host(self) -> clients.Host:
        return clients.Host(self.get("host", "ahab@ahab.local"), self.get("identity", "~/.ssh/id_ed25519_pi"),
                            self.get("repo", "~/dualsense-eurorack-midi"), self.get("python", sys.executable))

    def on_ssh(self, mode: str) -> bool:
        return self.get(ROUTE_KEY[mode], True)

    def route(self, mode: str) -> str:
        return f"SSH ({self.host.host})" if self.on_ssh(mode) else "cable"

    # ── pages ──
    def _build_pages(self):
        from .tabs import dualsense, ec4, er301, faderbank, pedals
        self.tabs = {}
        specs = {
            "dualsense": (dualsense.ControllerDrawing, ["Control", "Combos", "Overview"], dualsense.build_panel),
            "faderbank": (faderbank.FaderbankDrawing, ["Fader", "Device"], faderbank.build_panel),
            "pedals": (pedals.PedalDrawing, ["Pedal", "Device"], pedals.build_panel),
            "ec4": (ec4.EC4Drawing, ["Encoder", "Push", "Names"], ec4.build_panel),
            "er301": (er301.ER301Drawing, ["Ports", "Routes", "Pitch & LFO"], er301.build_panel),
        }
        for mode, (Drawing, tabs, build) in specs.items():
            page = QWidget()
            h = QHBoxLayout(page)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(0)
            drawing = Drawing(self)
            stage = Stage(drawing)
            holder = QWidget()
            hl = QVBoxLayout(holder)
            hl.setContentsMargins(18, 14, 18, 14)
            hl.addWidget(stage)
            h.addWidget(holder, 1)
            h.addWidget(W.vdivider())
            tab0 = os.environ.get("KONTROLLEUR_TAB") if os.environ.get("KONTROLLEUR_TAB") in tabs else None
            insp = W.Inspector(tabs, lambda t, col, b=build: b(self, t, col), tab0)
            h.addWidget(insp)
            self.pages.addWidget(page)
            self.tabs[mode] = (page, stage, insp, drawing)
            holder.setStyleSheet("background: transparent;")
        self.pages.setCurrentWidget(self.tabs[self.mode][0])

    @property
    def stage(self) -> Stage:
        return self.tabs[self.mode][1]

    @property
    def inspector(self) -> W.Inspector:
        return self.tabs[self.mode][2]

    # ── called by the tabs ──
    def changed(self, rebuild: bool = False):
        self.stage.update()
        if rebuild:
            self.inspector.rebuild()
        self._update_dirty()

    def set_note(self, key: str, text: str):
        self.notes.set(key, text)
        self.stage.update()

    def select_ds(self, g: str):
        self.ds_selected = g
        insp = self.tabs["dualsense"][2]
        if insp.tab != "Control":
            insp.set_tab("Control")
        else:
            insp.rebuild()
        self.stage.update()

    def select(self, attr: str, value, tab: Optional[str] = None):
        setattr(self, attr, value)
        if tab and self.inspector.tab != tab and tab in self.inspector.tabs:
            self.inspector.set_tab(tab)
        else:
            self.inspector.rebuild()
        self.stage.update()

    # ── top bar ──
    def _build_bar(self):
        h = QHBoxLayout(self.bar)
        h.setContentsMargins(16, 10, 16, 10)
        h.setSpacing(10)
        tb = QVBoxLayout()
        tb.setSpacing(1)
        self.title_lb = W.label("Kontrolleur", 17, BOLD, "title")
        self.sub_lb = W.label("→ i2c2midi → ER-301", 10, SEMIBOLD, "mono", Theme.dim, tracking=1)
        tb.addWidget(self.title_lb)
        tb.addWidget(self.sub_lb)
        h.addLayout(tb)
        h.addSpacing(8)
        self.mode_seg = W.Segmented(MODES, self.mode, self.set_mode)
        h.addWidget(self.mode_seg)
        self.route_seg = W.Segmented([(True, "SSH"), (False, "Cable")], True, self.set_route,
                                     tips={True: "Plugged into the SSH host", False: "Plugged into this computer"})
        self.route_seg.setFixedWidth(150)
        h.addWidget(self.route_seg)
        h.addStretch(1)
        self.dirty_lb = W.label("● Unsaved changes", 11, SEMIBOLD, "ui", Theme.trig)
        self.dirty_lb.setToolTip("Unsaved changes: Upload sends them")
        h.addWidget(self.dirty_lb)
        self.cfg_btn = QToolButton()
        self.cfg_btn.setText("Configs ▾")
        self.cfg_btn.setPopupMode(QToolButton.InstantPopup)
        self.cfg_btn.setToolTip("Save the whole setup (DualSense, 16n, pedals, EC4, notes) on this computer, or load one (Ctrl+Shift+S / Ctrl+O)")
        self.cfg_menu = QMenu(self)
        self.cfg_menu.aboutToShow.connect(self._fill_cfg_menu)
        self.cfg_btn.setMenu(self.cfg_menu)
        h.addWidget(self.cfg_btn)
        self.look_btn = QPushButton("Look")
        self.look_btn.setToolTip("Appearance: colours, fonts, text size")
        self.look_btn.clicked.connect(self.show_appearance)
        h.addWidget(self.look_btn)
        self.host_btn = QPushButton("SSH")
        self.host_btn.clicked.connect(self.show_settings)
        h.addWidget(self.host_btn)
        self.load_btn = QPushButton("Load")
        self.load_btn.clicked.connect(self.load_clicked)
        h.addWidget(self.load_btn)
        self.up_btn = QPushButton("Upload")
        self.up_btn.setObjectName("primary")
        self.up_btn.clicked.connect(self.upload_clicked)
        h.addWidget(self.up_btn)
        for b in (self.cfg_btn, self.look_btn, self.host_btn, self.load_btn, self.up_btn):
            b.setCursor(Qt.PointingHandCursor)
            b.setFocusPolicy(Qt.NoFocus)
        self._update_bar()

    def _update_bar(self):
        ec4 = self.mode == "ec4"
        self.load_btn.setText("↓  Receive" if ec4 else "↓  Load")
        self.up_btn.setText("↑  Send" if ec4 else "↑  Upload")
        self.load_btn.setToolTip("Receive the setups from the EC4 (Ctrl+R)" if ec4 else "Load from the device (Ctrl+R)")
        self.up_btn.setToolTip("Send the setups to the EC4 (Ctrl+S)" if ec4 else "Upload to the device (Ctrl+S)")
        self.route_seg.setVisible(self.mode in ROUTED)
        if self.mode in ROUTED:
            self.route_seg.set_current(self.on_ssh(self.mode))
            self.route_seg.setToolTip(f"How this device is reached: SSH = plugged into the SSH host ({self.host.host}), Cable = plugged into this computer")
        self.host_btn.setToolTip(f"SSH host: {self.host.host} (click to change)")
        for b in (self.load_btn, self.up_btn, self.route_seg):
            b.setEnabled(not self.busy)
        self._update_dirty()

    def resizeEvent(self, ev):
        compact = self.width() < 1290
        self.sub_lb.setVisible(not compact)
        super().resizeEvent(ev)

    def _build_status(self):
        h = QHBoxLayout(self.status_bar)
        h.setContentsMargins(16, 8, 16, 8)
        h.setSpacing(8)
        self.st_icon = W.label("ⓘ", 13)
        self.st_text = W.label("", 12)
        self.st_text.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.st_time = W.label("", 11, REGULAR, "mono", Theme.dim)
        h.addWidget(self.st_icon)
        h.addWidget(self.st_text, 1)
        h.addWidget(self.st_time)
        self._spin = 0
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._tick)
        self._spin_timer.start(120)

    def _tick(self):
        if self.busy:
            self._spin = (self._spin + 1) % 8
            self.st_icon.setText("⠋⠙⠹⠸⠼⠴⠦⠧"[self._spin])
            self.st_icon.setStyleSheet(f"color: {W.css(Theme.dim)}")

    def set_status(self, kind: str, text: str):
        self.status = Status(kind, text)
        icon, col = {"info": ("ⓘ", Theme.dim), "ok": ("✓", Theme.special), "warn": ("⚠", Theme.trig),
                     "error": ("✕", QColor.fromRgbF(1, 0.38, 0.38))}[kind]
        if not self.busy:
            self.st_icon.setText(icon)
        self.st_icon.setStyleSheet(f"color: {W.css(col)}; background: transparent;")
        self.st_text.setText(text)
        self.st_text.setToolTip(text)
        self.st_time.setText(self.status.time.strftime("%H:%M"))

    def set_busy(self, b: bool):
        self.busy = b
        self._update_bar()
        if not b:
            self.set_status(self.status.kind, self.status.text)

    # ── dirty ──
    def current_sources(self) -> str:
        return self.mapping.python_source() + "\n#----\n" + self.mapping.controls_source()

    @property
    def dirty(self) -> bool:
        return ((self.synced is not None and self.synced != self.current_sources())
                or (self.synced_bank is not None and self.synced_bank != self.bank.json)
                or (self.synced_pedals is not None and self.synced_pedals != self.pedals.json)
                or (self.synced_ec4 is not None and self.synced_ec4 != self.ec4.fingerprint))

    def _update_dirty(self):
        if hasattr(self, "dirty_lb"):
            self.dirty_lb.setVisible(self.dirty)

    # ── mode / route ──
    def set_mode(self, mode: str):
        if mode == self.mode:
            return
        self.mode = mode
        if not self.offline:
            self.s.setValue("deviceMode", mode)
        self.mode_seg.set_current(mode)
        self.pages.setCurrentWidget(self.tabs[mode][0])
        self._update_bar()
        if mode == "faderbank" and not self.bank.loaded:
            self.load_16n()
        elif mode == "pedals" and not self.pedals.loaded:
            self.load_pedals()
        elif mode == "ec4" and not self.ec4.loaded:
            self.load_ec4_cache()
        self.inspector.rebuild()

    def set_route(self, on_ssh: bool):
        if self.mode not in ROUTED:
            return
        self.s.setValue(ROUTE_KEY[self.mode], on_ssh)
        self._update_bar()
        if self.mode == "faderbank":
            self.load_16n()
        elif self.mode == "pedals":
            self.load_pedals()

    # ── startup ──
    def _startup(self):
        if self.mode == "faderbank":
            self.load_16n(then=lambda: self.load_routing(quiet=True))
        elif self.mode == "pedals":
            self.load_pedals(then=lambda: self.load_routing(quiet=True))
        elif self.mode == "ec4":
            self.load_ec4_cache()
            self.load_routing(quiet=True)
        else:
            self.load_routing()
        snap = os.environ.get("KONTROLLEUR_SNAPSHOT")
        if snap:
            QTimer.singleShot(int(float(os.environ.get("KONTROLLEUR_WAIT", "2")) * 1000), lambda: self._snapshot(snap))

    def _snapshot(self, path: str):
        w, h = (int(x) for x in os.environ.get("KONTROLLEUR_SIZE", "1400x840").split("x"))
        self.resize(w, h)
        QApplication.processEvents()
        QTimer.singleShot(300, lambda: (self.grab().save(path), QApplication.quit()))

    # ── running things in the background ──
    def run(self, fn: Callable, done: Callable, busy: bool = True):
        if busy:
            self.set_busy(True)

        def finish(r, e):
            if busy:
                self.busy = False
            done(r, e)
            if busy:
                self.set_busy(False)

        clients.background(fn, finish)

    def _fixture(self, name: str) -> dict:
        return json.loads((Path(self.offline) / name).read_text())

    # ── DualSense routing (always on the SSH host) ──
    def load_routing(self, quiet: bool = False, then: Optional[Callable] = None):
        before = self.status

        def fn():
            if self.offline:
                return self._fixture("export.json")
            return self.host.export()

        def done(e, err):
            if err is None and e.get("cc_names") is None:
                err = clients.ToolError(f"The SSH host answered, but: {e.get('error') or 'no data'}")
            if err is not None:
                if quiet and before.kind != "error":
                    self.set_status("warn", before.text + " The SSH host isn't reachable, so routes to the ER-301 can't be loaded or uploaded right now.")
                else:
                    self.set_status("error", f"Could not load from {self.host.host}: {err}")
            else:
                self.mapping.apply(e)
                self.synced = self.current_sources()
                if quiet:
                    self.set_status(before.kind, before.text)
                elif e.get("controls_error"):
                    self.set_status("warn", f"Loaded, but controls.py on the SSH host has a problem (defaults shown): {e['controls_error']}")
                elif not self.mapping.controls_on_pi:
                    self.set_status("warn", "Loaded the routing. The SSH host doesn't have the button functions installed yet: only the routing will be uploaded.")
                elif e.get("ok"):
                    self.set_status("ok", f"Loaded routing and button functions from {self.host.host}.")
                else:
                    self.set_status("warn", f"Loaded, but the file on the SSH host has a problem: {e.get('error') or ''}")
                self.refresh_all()
            if then:
                then()

        self.run(fn, done)

    def route_notes(self) -> dict:
        """Notes of routed controls, for the comments in er301_routes.py."""
        from .devices import hub_channel
        r = {k: self.notes.note(k) for k in self.notes.notes if k.startswith("ds:") and self.notes.note(k)}

        def ext(ch, cc, key):
            n = self.notes.note(key)
            if f"ext:{ch}:{cc}" not in r and n:
                r[f"ext:{ch}:{cc}"] = n

        for i, f in enumerate(self.bank.faders):
            ext(f["usb_channel"], f["usb_cc"], store.NotesStore.fader(i))
        for inp in self.pedals.inputs:
            for s in inp["slots"]:
                if s["on"] and s["kind"] == "cc":
                    ext(hub_channel(s["channel"], self.mapping), s["number"], store.NotesStore.pedal(inp["input"], s["slot"]))
        for si, su in enumerate(self.ec4.setups):
            for gi, g in enumerate(su["groups"]):
                for ei, e in enumerate(g["encoders"]):
                    if e["type"] == "cc":
                        ext(e["channel"], e["number"], store.NotesStore.ec4(si, gi, ei))
        return r

    def upload_routing(self, prefix: str = "", then: Optional[Callable] = None):
        p = self.mapping.problems()
        if p:
            self.set_status("error", (prefix + " " if prefix else "") + p[0])
            return
        source = self.mapping.python_source(self.route_notes())
        controls = self.mapping.controls_source()
        sent_all = self.current_sources()
        controls_on = self.mapping.controls_on_pi
        host = self.host

        def fn():
            if self.offline:
                return {"ok": True, "hub": "offline"}, {"ok": True, "hub": "offline"}
            r = host.install(source)
            if not r.get("ok"):
                return r, None
            c = host.install_controls(controls) if controls_on else None
            return r, c

        def done(res, err):
            pre = prefix + " " if prefix else ""
            if err is not None:
                self.set_status("error", pre + f"Upload to {host.host} failed: {err}")
                return
            r, c = res
            if not r.get("ok"):
                self.set_status("error", pre + f"The SSH host rejected the routing: {r.get('error') or 'unknown error'}")
                return
            parts, kind = [], "ok"
            if r.get("hub") is not None:
                parts.append("Routing sent to the i2c2midi.")
            else:
                parts.append("Routing saved, but the hub did not confirm (is the XIAO-TT connected?).")
                kind = "warn"
            if controls_on:
                if not c.get("ok"):
                    self.set_status("error", pre + " ".join(parts) + f" Button functions rejected: {c.get('error') or 'unknown error'}")
                    return
                line = c.get("hub")
                if line and "NOT applied" in line:
                    parts.append(f"The DualSense script refused the button functions: {line}")
                    kind = "error"
                elif line is not None:
                    parts.append("Button functions active.")
                else:
                    parts.append("Button functions saved; the DualSense script did not confirm (is it running?).")
                    kind = "warn" if kind == "ok" else kind
            else:
                parts.append("Button functions not uploaded (not installed on the SSH host yet).")
                kind = "warn" if kind == "ok" else kind
            self.synced = sent_all
            self.set_status(kind, pre + " ".join(parts))
            self._update_dirty()
            if then:
                then()

        self.run(fn, done)

    def routing_dirty(self) -> bool:
        return self.synced is not None and self.synced != self.current_sources()

    # ── 16n ──
    def load_16n(self, then: Optional[Callable] = None):
        on = self.on_ssh("faderbank")
        via = self.route("faderbank")

        def fn():
            if self.offline:
                return self._fixture("fader16.json")
            return self.host.tool(on, "fader16_tool.py", "export")

        def done(d, err):
            if err is not None:
                self.set_status("error", f"Could not read the 16n via {via}: {err}")
            elif not d.get("ok"):
                self.set_status("error", f"16n via {via}: {d.get('error') or 'no answer'}")
            else:
                self.bank.apply(d)
                self.synced_bank = self.bank.json
                fw = f"{self.bank.device}, firmware {self.bank.firmware}{' + curves' if self.bank.curves_supported else ' (no curves)'}"
                self.set_status("ok" if self.bank.curves_supported else "warn", f"Loaded the 16n ({fw}) via {via}.")
                self.refresh_all()
            if then:
                then()

        self.run(fn, done)

    def upload_16n(self):
        p = self.bank.problems()
        if p:
            return self.set_status("error", p[0])
        if not self.bank.loaded:
            return self.set_status("error", "Load the 16n first, so its current settings aren't overwritten with defaults.")
        on, via, js = self.on_ssh("faderbank"), self.route("faderbank"), self.bank.json

        def fn():
            if self.offline:
                return {"ok": True}
            return self.host.tool(on, "fader16_tool.py", "install", js)

        def done(r, err):
            if err is not None:
                return self.set_status("error", f"16n upload via {via} failed: {err}")
            if not r.get("ok"):
                why = r.get("error") or ("read-back differs: " + ", ".join(r.get("mismatch") or []))
                return self.set_status("error", f"16n not written correctly: {why}")
            self.synced_bank = js
            text = "16n written and verified."
            if r.get("skipped"):
                text += f" Not supported by its firmware: {', '.join(r['skipped'])}."
            self._then_routes(text)

        self.run(fn, done)

    def _then_routes(self, text: str):
        """After a device upload: send the ER-301 routing too if its routes changed."""
        if self.routing_dirty():
            self.upload_routing(prefix=text)
        else:
            self.set_status("ok", text)
            self._update_dirty()

    # ── MPC-20 ──
    def load_pedals(self, then: Optional[Callable] = None):
        on, via = self.on_ssh("pedals"), self.route("pedals")

        def fn():
            if self.offline:
                return self._fixture("mpc20.json")
            return self.host.tool(on, "mpc20_tool.py", "export")

        def done(d, err):
            if err is not None:
                self.set_status("error", f"Could not read the MPC-20 via {via}: {err}")
            elif not d.get("ok"):
                self.set_status("error", f"MPC-20 via {via}: {d.get('error') or 'no answer'}")
            else:
                self.pedals.apply(d)
                self.synced_pedals = self.pedals.json
                used = sum(1 for i in self.pedals.inputs[:4] if i["type_id"] != 0)
                self.set_status("ok", f"Loaded the {self.pedals.device} (firmware {self.pedals.firmware}) via {via}: {used} of 4 jacks in use.")
                self.refresh_all()
            if then:
                then()

        self.run(fn, done)

    def upload_pedals(self):
        p = self.pedals.problems()
        if p:
            return self.set_status("error", p[0])
        if not self.pedals.loaded:
            return self.set_status("error", "Load the MPC-20 first.")
        on, via, js = self.on_ssh("pedals"), self.route("pedals"), self.pedals.json

        def fn():
            if self.offline:
                return {"ok": True, "written": 0}
            return self.host.tool(on, "mpc20_tool.py", "install", js)

        def done(r, err):
            if err is not None:
                return self.set_status("error", f"MPC-20 upload via {via} failed: {err}")
            if not r.get("ok"):
                why = r.get("error") or ("read-back differs: " + ", ".join(r.get("mismatch") or []))
                return self.set_status("error", f"MPC-20 not written correctly: {why}")
            if r.get("inputs"):
                self.pedals.apply_inputs(r["inputs"])
            self.synced_pedals = self.pedals.json
            n = r.get("written") or 0
            text = f"MPC-20 written and verified ({n} message{'' if n == 1 else 's'} changed)."
            if r.get("skipped"):
                text += f" Skipped (nothing plugged in): {', '.join(r['skipped'])}."
            self.refresh_all()
            self._then_routes(text)

        self.run(fn, done)

    # ── EC4 ──
    def load_ec4_cache(self):
        if self.offline and (Path(self.offline) / "ec4-last.json").exists():
            self.ec4.apply(self._fixture("ec4-last.json"))
            self.ec4.received_at = dt.datetime.fromtimestamp((Path(self.offline) / "ec4-last.json").stat().st_mtime)
            ok = self.ec4.loaded
        else:
            ok = store.load_ec4_cache(self.ec4)
        if ok:
            self.synced_ec4 = self.ec4.fingerprint
            when = self.ec4.received_at.strftime("%-d %b %Y at %H:%M") if self.ec4.received_at else "earlier"
            self.set_status("info", f"Showing the EC4 setups received {when}. Press “Receive” to fetch them again.")
        else:
            self.set_status("info", "Press “Receive”, then on the EC4: Func → Setup → Send → “Send all setups” (hold the knob).")
        self.refresh_all()

    def receive_ec4(self):
        on, via = self.on_ssh("ec4"), self.route("ec4")
        self.set_status("info", f"Waiting for the EC4 (via {via}): on the device press Func → Setup → Send → “Send all setups” and hold the knob …")

        def fn():
            if self.offline:
                return self._fixture("ec4-last.json")
            return self.host.tool(on, "ec4_tool.py", ["receive", "120"], timeout=200)

        def done(d, err):
            if err is not None:
                return self.set_status("error", f"Could not receive from the EC4 via {via}: {err}")
            if not d.get("ok"):
                return self.set_status("error", f"EC4: {d.get('error') or 'nothing received'}")
            self.ec4.apply(d)
            self.ec4.received_at = dt.datetime.now()
            store.save_ec4_cache(self.ec4)
            self.synced_ec4 = self.ec4.fingerprint
            used = sum(1 for s in self.ec4.setups if not s["name"].startswith("SE") and not s["name"].startswith("--"))
            self.set_status("ok", f"Received all 16 setups from the EC4 via {via} ({used} named). Kept on this computer as a backup.")
            self.refresh_all()

        self.run(fn, done)

    def send_ec4(self):
        p = self.ec4.problems()
        if p:
            return self.set_status("error", p[0])
        if not self.ec4.loaded:
            return self.set_status("error", "Receive the setups from the EC4 first.")
        box = QMessageBox(self)
        box.setWindowTitle("Send all setups to the EC4?")
        box.setText("Send all setups to the EC4?")
        box.setInformativeText("First on the EC4: press Func, choose Setup, then Receive (the display shows “Work in progress”). "
                               "Then press Send. This replaces all 16 setups on the device; the last received data is kept on "
                               "this computer as a backup.")
        box.setStandardButtons(QMessageBox.Cancel | QMessageBox.Ok)
        box.button(QMessageBox.Ok).setText("Send")
        if box.exec() != QMessageBox.Ok:
            return
        on, via, js = self.on_ssh("ec4"), self.route("ec4"), self.ec4.json
        self.set_status("info", f"Sending all setups to the EC4 via {via} … (about a minute; the EC4 shows the progress)")

        def fn():
            if self.offline:
                return {"ok": True, "changed_bytes": 0}
            return self.host.tool(on, "ec4_tool.py", ["send"], js, timeout=300)

        def done(r, err):
            if err is not None:
                return self.set_status("error", f"Sending to the EC4 via {via} failed: {err}")
            if not r.get("ok"):
                return self.set_status("error", f"EC4 not written: {r.get('error') or 'unknown error'}")
            store.save_ec4_cache(self.ec4)
            self.synced_ec4 = self.ec4.fingerprint
            self._then_routes(f"Sent all setups to the EC4 ({r.get('changed_bytes') or 0} bytes changed). Check its display for “done”.")

        self.run(fn, done)

    # ── buttons ──
    def load_clicked(self):
        if self.busy:
            return
        if self.mode == "faderbank":
            self.load_16n(then=lambda: self.load_routing(quiet=True))
        elif self.mode == "pedals":
            self.load_pedals(then=lambda: self.load_routing(quiet=True))
        elif self.mode == "ec4":
            self.receive_ec4()
        else:
            self.load_routing()

    def upload_clicked(self):
        if self.busy:
            return
        if self.mode == "faderbank":
            self.upload_16n()
        elif self.mode == "pedals":
            self.upload_pedals()
        elif self.mode == "ec4":
            self.send_ec4()
        else:
            self.upload_routing()

    def refresh_all(self):
        for page, stage, insp, drawing in self.tabs.values():
            stage.update()
        self.inspector.rebuild()
        self._update_dirty()

    def _shortcuts(self):
        def sc(keys, fn):
            s = QShortcut(QKeySequence(keys), self)
            s.setContext(Qt.ApplicationShortcut)
            s.activated.connect(fn)

        sc("Ctrl+R", self.load_clicked)
        sc("Ctrl+S", self.upload_clicked)
        sc("Ctrl+Shift+S", self.save_config)
        sc("Ctrl+O", self.open_config)
        sc(QKeySequence.ZoomIn, lambda: self.stage.step(1.25))
        sc("Ctrl+=", lambda: self.stage.step(1.25))
        sc(QKeySequence.ZoomOut, lambda: self.stage.step(1 / 1.25))
        sc("Ctrl+0", lambda: self.stage.reset())

    # ── configurations ──
    def _fill_cfg_menu(self):
        m = self.cfg_menu
        m.clear()
        m.addAction("Save Configuration…", self.save_config)
        m.addAction("Open Configuration…", self.open_config)
        rec = store.recent()
        if rec:
            m.addSeparator()
            m.addSection("Recent")
            for p in rec:
                m.addAction(p.stem, lambda p=p: self.open_config_path(p))
        m.addSeparator()
        m.addAction("Show Configurations Folder", lambda: _reveal(store.config_folder()))

    def save_config(self):
        c = {"format": store.FORMAT, "version": 1, "saved": store.iso_now(), "mapping": self.mapping.snapshot()}
        if self.bank.loaded:
            c["faderbank"] = {"faders": self.bank.faders, "options": self.bank.options}
        if self.pedals.loaded:
            c["pedals"] = self.pedals.inputs
        if self.ec4.loaded:
            c["ec4"] = {"mem": self.ec4.mem, "version": self.ec4.version, "setups": self.ec4.setups}
        c["notes"] = self.notes.notes
        name = dt.datetime.now().strftime("Setup %Y-%m-%d %H%M")
        path, _ = QFileDialog.getSaveFileName(self, "Save configuration", str(store.config_folder() / f"{name}.dsmconfig"),
                                              "Kontrolleur configuration (*.dsmconfig)")
        if not path:
            return
        if not path.endswith(".dsmconfig"):
            path += ".dsmconfig"
        try:
            store.write_config(c, Path(path))
            parts = [t for k, t in store.PARTS if store.config_has(c, k)]
            self.set_status("ok", f"Saved “{Path(path).stem}”: {', '.join(parts)}.")
        except Exception as e:
            self.set_status("error", f"Could not save the configuration: {e}")

    def open_config(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open configuration", str(store.config_folder()),
                                              "Kontrolleur configuration (*.dsmconfig *.json)")
        if path:
            self.open_config_path(Path(path))

    def open_config_path(self, path: Path):
        try:
            c = store.read_config(path)
        except Exception as e:
            return self.set_status("error", f"Could not open “{path.name}”: {e}")
        dlg = ConfigLoadDialog(self, path, c)
        if dlg.exec() != QDialog.Accepted:
            return
        parts, done = dlg.parts(), []
        if "dualsense" in parts and c.get("mapping"):
            self.mapping.restore(c["mapping"])
            done.append("DualSense")
        if "faderbank" in parts and c.get("faderbank") and len(c["faderbank"]["faders"]) == 16:
            self.bank.faders, self.bank.options = c["faderbank"]["faders"], c["faderbank"]["options"]
            self.bank.loaded = True
            done.append("16n")
        if "pedals" in parts and c.get("pedals") and len(c["pedals"]) >= 4:
            self.pedals.apply_inputs(c["pedals"])
            done.append("pedals")
        if "ec4" in parts and c.get("ec4") and len(c["ec4"]["setups"]) == 16:
            self.ec4.apply({"version": c["ec4"]["version"], "mem": c["ec4"]["mem"], "setups": c["ec4"]["setups"]})
            done.append("EC4")
        if "notes" in parts and c.get("notes") is not None:
            self.notes.replace(c["notes"])
            done.append("notes")
        self.set_status("ok", f"Loaded {', '.join(done)} from “{path.stem}”. Send it to the devices with “Upload” on each tab.")
        self.refresh_all()

    # ── popovers ──
    def _popup(self, anchor: QWidget, content: QWidget):
        pop = QFrame(self, Qt.Popup)
        pop.setObjectName("panel")
        pop.setAttribute(Qt.WA_StyledBackground, True)
        pop.setStyleSheet(f"#panel {{ background: {W.css(Theme.panel)}; border: 1px solid {W.css(Theme.line)}; border-radius: 10px; }}")
        lay = QVBoxLayout(pop)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(content)
        pop.adjustSize()
        g = anchor.mapToGlobal(QPoint(anchor.width(), anchor.height() + 6))
        pop.move(g.x() - pop.width(), g.y())
        pop.show()
        return pop

    def show_settings(self):
        w = QWidget()
        w.setFixedWidth(400)
        v = QVBoxLayout(w)
        v.setContentsMargins(16, 16, 16, 16)
        v.setSpacing(10)
        v.addWidget(W.label("SSH host", 14, BOLD))

        def field(title_text, key, default, placeholder):
            e = QLineEdit(str(self.get(key, default)))
            e.setPlaceholderText(placeholder)
            e.setFixedWidth(230)
            e.textEdited.connect(lambda t, k=key: (self.s.setValue(k, t), self._update_bar()))
            v.addWidget(W.row(title_text, e))

        field("Host", "host", "ahab@ahab.local", "user@host")
        field("SSH key", "identity", "~/.ssh/id_ed25519_pi", "~/.ssh/…")
        field("Repo on the host", "repo", "~/dualsense-eurorack-midi", "~/dualsense-eurorack-midi")
        v.addWidget(W.hint("Used by every tab. Leave the key empty to use your ~/.ssh/config."))
        v.addWidget(W.divider())
        v.addWidget(W.label("How each device is reached", 14, BOLD))
        v.addWidget(W.hint("One setting per device, the same as the SSH | Cable switch on that device's tab. SSH: plugged into the "
                           "SSH host. Cable: plugged into this computer. The DualSense and the ER-301 routing always go through the SSH host."))
        for title_text, mode in (("16n faderbank", "faderbank"), ("MPC-20 pedals", "pedals"), ("Faderfox EC4", "ec4")):
            def pick(on, mode=mode):
                self.s.setValue(ROUTE_KEY[mode], on)
                self._update_bar()
                if mode == self.mode:
                    self.set_route(on)
            seg = W.Segmented([(True, "SSH"), (False, "Cable")], self.on_ssh(mode), pick)
            seg.setFixedWidth(150)
            v.addWidget(W.row(title_text, seg))
        v.addWidget(W.divider())
        v.addWidget(W.label("Cable", 14, BOLD))
        field("Python", "python", sys.executable, "python3 with python-rtmidi")
        v.addWidget(W.hint("For devices plugged into this computer. 16n and EC4 need python-rtmidi; the MPC-20 needs pyusb on Linux "
                           "(install-linux.sh sets both up)."))
        self._popup(self.host_btn, w)

    def show_appearance(self):
        AppearanceDialog(self).show()

    def apply_appearance(self):
        QApplication.instance().setFont(Theme.font(12))
        QApplication.instance().setStyleSheet(theme.A.stylesheet())
        self.title_lb.setFont(Theme.font(17, BOLD, "title"))
        self.title_lb.setStyleSheet(f"color: {W.css(Theme.ink)}")
        self.sub_lb.setFont(Theme.font(10, SEMIBOLD, "mono"))
        self.sub_lb.setStyleSheet(f"color: {W.css(Theme.dim)}")
        self.dirty_lb.setStyleSheet(f"color: {W.css(Theme.trig)}")
        self.st_time.setStyleSheet(f"color: {W.css(Theme.dim)}")
        for _, stage, insp, _ in self.tabs.values():
            stage.restyle()
            stage.update()
            insp.rebuild()
        self.set_status(self.status.kind, self.status.text)


class ConfigLoadDialog(QDialog):
    def __init__(self, parent, path: Path, c: dict):
        super().__init__(parent)
        self.setWindowTitle("Load configuration")
        self.setFixedWidth(460)
        v = QVBoxLayout(self)
        v.setContentsMargins(22, 22, 22, 22)
        v.setSpacing(12)
        v.addWidget(W.label("Load configuration", 18, BOLD))
        v.addWidget(W.label(path.stem, 13, SEMIBOLD))
        saved = store.parse_iso(c.get("saved", ""))
        if saved:
            v.addWidget(W.hint("Saved " + saved.strftime("%-d %b %Y at %H:%M")))
        self.boxes = {}
        for key, text in store.PARTS:
            has = store.config_has(c, key)
            b = QCheckBox(text + ("" if has else "   (not in this file)"))
            b.setChecked(has)
            b.setEnabled(has)
            v.addWidget(b)
            self.boxes[key] = b
        v.addWidget(W.hint("Loading only changes the app. Send it to the devices with “Upload” on each tab. Notes are replaced by the ones in the file."))
        bb = QDialogButtonBox(QDialogButtonBox.Cancel | QDialogButtonBox.Ok)
        bb.button(QDialogButtonBox.Ok).setText("Load")
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)

    def parts(self):
        return {k for k, b in self.boxes.items() if b.isChecked()}


class AppearanceDialog(QDialog):
    def __init__(self, main: Main):
        super().__init__(main)
        self.main = main
        self.setWindowTitle("Appearance")
        self.setMinimumSize(400, 560)
        self.resize(400, 660)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        sc = QScrollArea()
        sc.setWidgetResizable(True)
        sc.setFrameShape(QFrame.NoFrame)
        outer.addWidget(sc)
        self.sc = sc
        self.families = sorted(set(__import__("PySide6.QtGui", fromlist=["QFontDatabase"]).QFontDatabase.families()))
        self.fill()

    def fill(self):
        A = theme.A
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(18, 18, 18, 18)
        v.setSpacing(12)
        v.addWidget(W.label("Appearance", 20, BOLD))
        presets = QComboBox()
        presets.addItem("Choose a preset …")
        for name, _ in theme.PRESETS:
            presets.addItem(name)
        presets.currentIndexChanged.connect(lambda k: k > 0 and self.apply(json.loads(json.dumps(theme.PRESETS[k - 1][1]))))
        v.addWidget(W.row("Preset", presets))
        for group in ("Surfaces", "Text & lines", "Accents"):
            v.addWidget(W.label(group.upper(), 10, HEAVY, "mono", Theme.dim, tracking=1.2))
            for slot, text, g in theme.SLOTS:
                if g != group:
                    continue
                b = QPushButton()
                c = A.color(slot)
                b.setFixedSize(56, 22)
                b.setStyleSheet(f"background: {W.css(c)}; border: 1px solid {W.css(Theme.line)}; border-radius: 5px;")
                b.setCursor(Qt.PointingHandCursor)
                b.clicked.connect(lambda _=False, s=slot: self.pick_color(s))
                v.addWidget(W.row(text, b))
        v.addWidget(W.label("FONTS", 10, HEAVY, "mono", Theme.dim, tracking=1.2))
        for text, key in (("Interface", "uiFont"), ("Labels & numbers", "monoFont"), ("Titles", "titleFont")):
            cb = QComboBox()
            items = theme.BUILTIN_FONTS + self.families
            cb.addItems(items)
            cur = A.data.get(key, "System")
            if cur in items:
                cb.setCurrentIndex(items.index(cur))
            cb.setMaxVisibleItems(20)
            cb.setFixedWidth(200)
            cb.currentTextChanged.connect(lambda t, k=key: self.apply_kw(**{k: t}))
            v.addWidget(W.row(text, cb))
        sl = QSlider(Qt.Horizontal)
        sl.setRange(16, 27)               # 0.80 … 1.35 in 0.05 steps
        sl.setValue(round(A.scale * 20))
        val = W.label(f"{round(A.scale * 100)}%", 12, SEMIBOLD, "mono")
        sl.valueChanged.connect(lambda x: val.setText(f"{x * 5}%"))
        sl.sliderReleased.connect(lambda: self.apply_kw(scale=sl.value() / 20))
        sl.setFixedWidth(160)
        v.addWidget(W.row("Text size", W.hbox(sl, val)))
        v.addWidget(W.hint("Changes apply right away and are kept on this computer. The drawings make room for bigger text and fit "
                           "themselves to the window: zoom in there (pinch, scroll wheel, Ctrl +) to see them larger."))
        reset = QPushButton("Reset to Blueprint")
        reset.clicked.connect(lambda: self.apply(json.loads(json.dumps(theme.PRESETS[0][1]))))
        v.addWidget(reset, 0, Qt.AlignRight)
        v.addStretch(1)
        old = self.sc.takeWidget()
        self.sc.setWidget(w)
        if old:
            old.deleteLater()

    def pick_color(self, slot):
        c = QColorDialog.getColor(theme.A.color(slot), self, "Colour", QColorDialog.ShowAlphaChannel)
        if c.isValid():
            theme.A.set_color(slot, c)
            self.refresh()

    def apply(self, data):
        theme.A.set(data)
        self.refresh()

    def apply_kw(self, **kw):
        theme.A.update(**kw)
        self.refresh()

    def refresh(self):
        self.main.apply_appearance()
        QTimer.singleShot(0, self.fill)


def _reveal(path: Path):
    if sys.platform == "darwin":
        subprocess.Popen(["open", str(path)])
    elif sys.platform.startswith("linux"):
        subprocess.Popen(["xdg-open", str(path)])
    else:
        os.startfile(str(path))   # noqa


def _migrate_mac_settings(s):
    """First start on a Mac: take over the Swift app's settings (host, key, routes, appearance …)."""
    if sys.platform != "darwin" or s.value("migrated") in (True, "true"):
        return
    s.setValue("migrated", True)
    try:
        out = subprocess.run(["defaults", "export", "net.ahab.dualsense-mapper", "-"], capture_output=True, timeout=5).stdout
        d = plistlib.loads(out)
    except Exception:
        return
    for k in ("host", "identity", "repo", "fader16OnPi", "pedalsOnPi", "ec4OnPi", "deviceMode", "recentConfigs"):
        if k in d:
            s.setValue(k, d[k])
    if isinstance(d.get("appearance"), (bytes, bytearray)):
        try:
            s.setValue("appearance", json.dumps(json.loads(d["appearance"].decode())))
        except Exception:
            pass


def main():
    if sys.platform.startswith("linux") and not os.environ.get("QT_QPA_PLATFORM") and os.environ.get("WAYLAND_DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "wayland;xcb")
    app = QApplication(sys.argv)
    app.setApplicationName("Kontrolleur")
    app.setOrganizationName("Kontrolleur")
    app.setDesktopFileName("kontrolleur")
    _migrate_mac_settings(store.settings())    # before the theme reads its settings
    theme.init()
    w = Main()
    w.show()
    sys.exit(app.exec())
