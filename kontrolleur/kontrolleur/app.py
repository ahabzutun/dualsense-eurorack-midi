"""Kontrolleur main window."""
from __future__ import annotations

import datetime as dt
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Callable, Optional

from PySide6.QtCore import QEvent, QObject, QPoint, QSize, Qt, QTimer
from PySide6.QtGui import QAction, QColor, QFont, QIcon, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QCheckBox, QColorDialog, QComboBox, QDialog, QDialogButtonBox, QFileDialog, QPlainTextEdit,
                               QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMenu, QMessageBox,
                               QPushButton, QScrollArea, QSizePolicy, QSlider, QStackedWidget, QToolButton, QVBoxLayout,
                               QWidget)

from . import clients, ssp, store, teletype, theme
from . import macros as macros_model
from . import widgets as W
from .devices import EC4Board, Faderbank, PedalBoard
from .model import Mapping
from .stage import Stage
from .theme import BOLD, HEAVY, SEMIBOLD, REGULAR, Theme

MODES = [("dualsense", "DualSense"), ("faderbank", "16n"), ("pedals", "Pedals"), ("ec4", "EC4"), ("pi", "Routing"), ("er301", "ER-301"),
         ("teletype", "Teletype"), ("macros", "Macros")]
ROUTED = ("faderbank", "pedals", "ec4")
ROUTE_KEY = {"faderbank": "fader16OnPi", "pedals": "pedalsOnPi", "ec4": "ec4OnPi"}
ICON = Path(__file__).resolve().parent / "icon.png"


class Status:
    def __init__(self, kind: str, text: str):
        self.kind, self.text, self.time = kind, text, dt.datetime.now()


def _counts(st: dict) -> str:
    """Route counts from an i2c2midi status (its reply, else what was sent)."""
    c = st.get("reply") or st.get("expected") or {}
    return f"{c.get('cv', 0)} CV, {c.get('nrpn', 0)} NRPN, {c.get('triggers', 0)} trigger, {c.get('note_triggers', 0)} note trigger"


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
        self.pi_sha: dict = {}          # fingerprints of er301_routes.py / controls.py as loaded (export "sha")
        self.i2c2midi: Optional[dict] = None    # the i2c2midi's last answer to the routing (export "i2c2midi")
        self.links: Optional[list] = None       # the Pi's XIAO links of the MIDI bus (export "links")
        self.loglady: Optional[dict] = None     # Teletype's own XIAO (export "loglady")
        self.link_health: Optional[dict] = None # per XIAO link: is its USB host answering (export "link_health")
        self._loading: set = set()      # device loads running (never two at once for the same device)
        self._want_full: set = set()    # …and asked for by their tab while a background read ran
        self.bank_cached = self.pedals_cached = None    # when the shown 16n / MPC-20 settings were read
        # selections
        self.ds_selected = os.environ.get("KONTROLLEUR_SELECT", "cross")
        self.fader_sel = 0
        self.pedal_sel = 0
        self.ec4_setup = self.ec4_group = self.ec4_enc = 0
        self.er_port: Optional[str] = None
        self.er_view = "CV"            # which ER-301 ports the matrix shows: CV / TR
        self.pi_sel = os.environ.get("KONTROLLEUR_PI_SELECT", "mod_slice")   # Pi tab: mod_<module> | src_<device>
        self.pi_add = {}               # Pi tab: the “Patch a controller” form
        self.ssp = ssp.SSPState()      # Pi tab: the Percussa SSP's patch
        self.ssp.pick = str(self.s.value("sspPreset", "") or "")
        self.tt = teletype.TeletypeState()          # Teletype tab: scenes read from a USB stick
        self.tt_mon = teletype.Monitor(self._tt_live)
        self._tt_sig = None
        self.tt_dirty = {}                          # scene → edited here, not written to a stick yet
        self.macros = macros_model.MacrosState()    # Macros tab: grid_macros.json on the Pi
        self.mac_bank = int(os.environ.get("KONTROLLEUR_MAC_BANK", "1")) - 1
        self.mac_slot = int(os.environ.get("KONTROLLEUR_MAC_SLOT", "1")) - 1
        self.tt_script = "1"
        try:
            self.tt_scene = int(os.environ.get("KONTROLLEUR_TT_SCENE") or self.s.value("ttScene", -1))
        except (TypeError, ValueError):
            self.tt_scene = -1
        if self.tt_scene not in self.tt.scenes:
            self.tt_scene = next((n for n in sorted(self.tt.scenes) if not self.tt.scenes[n].empty), 0)
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
                            self.get("repo", "~/dualsense-eurorack-midi"), self.get("python", "" if clients.FROZEN else sys.executable))

    def on_ssh(self, mode: str) -> bool:
        return self.get(ROUTE_KEY[mode], True)

    def route(self, mode: str) -> str:
        return f"SSH ({self.host.host})" if self.on_ssh(mode) else "cable"

    # ── pages ──
    def _build_pages(self):
        from .tabs import dualsense, ec4, er301, faderbank, pedals, pi
        from .tabs import teletype as tt_tab
        from .tabs import macros as macros_tab
        self.tabs = {}
        specs = {
            "dualsense": (dualsense.ControllerDrawing, ["Control", "Combos", "Overview"], dualsense.build_panel),
            "pi": (pi.PiDrawing, ["Patch"], pi.build_panel),
            "faderbank": (faderbank.FaderbankDrawing, ["Fader", "Device"], faderbank.build_panel),
            "pedals": (pedals.PedalDrawing, ["Pedal", "Device"], pedals.build_panel),
            "ec4": (ec4.EC4Drawing, ["Encoder", "Push", "Names"], ec4.build_panel),
            "er301": (er301.ER301Drawing, ["Ports", "Routes", "Pitch & LFO"], er301.build_panel),
            "teletype": (tt_tab.TeletypeDrawing, ["Scene", "Script", "Live"], tt_tab.build_panel),
            "macros": (macros_tab.MacrosDrawing, ["Slot", "Bank", "Settings"], macros_tab.build_panel),
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
        self.data_gen = getattr(self, "data_gen", 0) + 1       # drawings that cache model data recompute
        self.stage.update()
        if rebuild:
            self.inspector.rebuild()
        self._update_dirty()

    def set_note(self, key: str, text: str):
        self.notes.set(key, text)
        self.data_gen = getattr(self, "data_gen", 0) + 1
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
        self.data_gen = getattr(self, "data_gen", 0) + 1
        if attr == "er_port" and value:
            self.er_view = value[:2]       # a TR port picked in the list: show the SC.TR matrix
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
        if self.mode == "macros":
            self.load_btn.setToolTip("Load the grid macros from the Pi (Ctrl+R)")
            self.up_btn.setToolTip("Upload names, slots and settings to the Pi (Ctrl+S)")
        if self.mode == "teletype":
            self.load_btn.setText("↓  From stick")
            self.up_btn.setText("↑  To stick")
            self.load_btn.setToolTip("Read the scenes the Teletype wrote to a USB stick (Ctrl+R)")
            self.up_btn.setToolTip("Write the scene in use to the stick as tt##.txt, for READ FROM USB (Ctrl+S)")
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
        """Jobs can start the next one when they finish (device upload → routing, Upload anyway …): count them,
        so the buttons stay off until the last one is done."""
        self._busy_n = max(0, getattr(self, "_busy_n", 0) + (1 if b else -1))
        self.busy = self._busy_n > 0
        b = self.busy
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
                or (self.synced_ec4 is not None and self.synced_ec4 != self.ec4.fingerprint)
                or self.macros.dirty)

    def _update_dirty(self):
        if hasattr(self, "dirty_lb"):
            self.dirty_lb.setVisible(self.dirty)

    # ── mode / route ──
    def set_mode(self, mode: str):
        if mode == self.mode:
            return
        self.mode = mode
        self.data_gen = getattr(self, "data_gen", 0) + 1     # edits made in other tabs show up in Routing
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
        elif mode == "pi" and not self.ec4.loaded:
            self.load_ec4_cache(quiet=True)      # names of the EC4 controls patched to Pi functions
        if mode in ("pi", "macros") and (not self.ssp.loaded or self.ssp.stale):
            self.load_ssp()
        if mode in ("pi", "macros") and not self.macros.loaded and not self.macros.loading:
            self.load_macros(quiet=mode == "pi")
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
    def _load_device_caches(self):
        """The last 16n / MPC-20 settings read on this computer, so Routing and ER-301 show the right sources
        right away. Marked as not loaded: uploading needs a fresh read."""
        if self.offline:
            return
        for name, dev, attr in (("16n", self.bank, "bank_cached"), ("mpc20", self.pedals, "pedals_cached")):
            if dev.loaded:
                continue
            c = store.load_device_cache(name)
            if c is None:
                continue
            try:
                dev.apply(c[0])
            except Exception:
                continue
            dev.loaded = False
            setattr(self, attr, c[1])

    def _refresh_devices_quietly(self):
        """In the overview tabs: read the 16n and the MPC-20 in the background (no status unless it works)."""
        if self.offline:
            return
        self.load_16n(quiet=True, then=lambda: self.load_pedals(quiet=True))

    def _startup(self):
        self._load_device_caches()
        if self.mode == "faderbank":
            self.load_16n(then=lambda: self.load_routing(quiet=True))
        elif self.mode == "pedals":
            self.load_pedals(then=lambda: self.load_routing(quiet=True))
        elif self.mode == "ec4":
            self.load_ec4_cache()
            self.load_routing(quiet=True)
        else:
            if self.mode in ("pi", "macros"):
                self.load_ec4_cache(quiet=True)
                self.load_ssp()
                self.load_macros(quiet=self.mode == "pi")
            self.load_routing(then=self._refresh_devices_quietly if self.mode in ("pi", "er301") else None)
        snap = os.environ.get("KONTROLLEUR_SNAPSHOT")
        if snap:
            QTimer.singleShot(int(float(os.environ.get("KONTROLLEUR_WAIT", "2")) * 1000), lambda: self._snapshot(snap))

    # ── Percussa SSP (its own ssh connection; never blocks the other tabs) ──
    def load_ssp(self):
        st = self.ssp
        if st.loading:
            return
        st.loading = True
        st.error = ""
        hosts, identity = self.get("sspHosts", ssp.DEFAULT_HOSTS), self.get("sspIdentity", "")

        def fn():
            if self.offline:
                return "offline", self._fixture("ssp-scan.json")
            return ssp.scan_remote(hosts, identity)

        def done(r, err):
            st.loading = False
            if err is None and not (r[1] or {}).get("ok"):
                err = clients.ToolError((r[1] or {}).get("error") or "no answer")
            if err is not None:
                st.error = f"SSP: {err}"
            else:
                st.host, st.scan, st.stale = r[0], r[1], False
                st.save_cache()
            self.refresh_all()

        self.run(fn, done, busy=False)
        self.refresh_all()

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
            try:
                done(r, e)              # may start the next job (busy again before this one counts down)
            finally:
                if busy:
                    self.set_busy(False)

        clients.background(fn, finish)

    def _fixture(self, name: str) -> dict:
        return json.loads((Path(self.offline) / name).read_text())

    # ── DualSense routing (always on the SSH host) ──
    def load_routing(self, quiet: bool = False, then: Optional[Callable] = None):
        before = self.status
        start = self.current_sources()          # to notice edits made while this loads

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
            elif self.synced is not None and self.current_sources() != start:
                # changed here while loading (a slow network): keep the edits, and the old fingerprints, so an
                # upload still notices if the Pi changed meanwhile
                self.set_status("warn", "You changed the DualSense routing while it was loading: kept your changes. "
                                        "Press Load to replace them with what's on the Pi.")
            else:
                edited_first = self.synced is None and self.current_sources() != start
                self.mapping.apply(e)
                self.pi_sha = dict(e.get("sha") or {})
                self.i2c2midi = e.get("i2c2midi")
                self.links = e.get("links") or self.links
                self.loglady = e.get("loglady")
                self.link_health = e.get("link_health")
                self.synced = self.current_sources()
                self._sync_shared_ec4()
                if edited_first:
                    self.set_status("warn", f"Loaded from {self.host.host}: what you changed before it arrived was replaced.")
                elif quiet:
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
        r = {k: self.notes.note(k) for k in self.notes.notes if k.startswith(("ds:", "er:")) and self.notes.note(k)}

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

    def upload_routing(self, prefix: str = "", then: Optional[Callable] = None, force: bool = False):
        """er301_routes.py + controls.py to the Pi. Checked before (nobody changed them there since they were
        loaded) and after (read back, and the i2c2midi's own answer)."""
        p = self.mapping.problems()
        if p:
            self.set_status("error", (prefix + " " if prefix else "") + p[0])
            return
        if self.synced is None and not self.offline:
            self.set_status("error", (prefix + " " if prefix else "") + "Load the routing from the Pi first (↓ Load): "
                                     "without it, the upload would replace what's there with defaults.")
            return
        source = self.mapping.python_source(self.route_notes())
        controls = self.mapping.controls_source()
        sent_all = self.current_sources()
        controls_on = self.mapping.controls_on_pi
        host = self.host
        feats = self.mapping.pi_features
        expect = {} if force or "expect" not in feats else dict(self.pi_sha)

        def fn():
            if self.offline:
                return {"ok": True, "hub": "offline"}, {"ok": True, "hub": "offline"}, None, None
            if expect:                          # both files unchanged on the Pi? (before writing either)
                now = host.export().get("sha") or {}
                for k, what in (("routes", "the ER-301 routing"), ("controls", "the button functions")):
                    if expect.get(k) and now.get(k) and now[k] != expect[k] and (k == "routes" or controls_on):
                        return {"ok": False, "conflict": what}, None, None, None
            r = host.install(source, expect.get("routes"))
            if not r.get("ok"):
                return r, None, None, None
            c = host.install_controls(controls, expect.get("controls")) if controls_on else None
            back = host.export()               # read back what the Pi has now
            m = Mapping()
            m.apply(back)
            got = m.python_source() + "\n#----\n" + m.controls_source()
            return r, c, back, got

        def done(res, err):
            pre = prefix + " " if prefix else ""
            if err is not None:
                self.set_status("error", pre + f"Upload to {host.host} failed: {err}")
                return
            r, c, back, got = res
            if r.get("conflict"):
                return self._upload_conflict(r["conflict"] if isinstance(r["conflict"], str) else "the ER-301 routing",
                                             prefix, then)
            if not r.get("ok"):
                self.set_status("error", pre + f"The Pi rejected the routing: {r.get('error') or 'unknown error'}")
                return
            if back is not None:
                self.pi_sha = dict(back.get("sha") or {})
                self.i2c2midi = r.get("i2c2midi") or back.get("i2c2midi")
            parts, kind = [], "ok"
            st = r.get("i2c2midi")
            if st:                              # the i2c2midi's own answer (hub with confirmation)
                state = st.get("state")
                if state == "confirmed":
                    parts.append(f"The i2c2midi confirmed the routing ({_counts(st)}).")
                elif state == "no answer":
                    parts.append("Saved on the Pi, but the i2c2midi didn't answer (rack off?): the Pi keeps sending it "
                                 "until it does.")
                    kind = "warn"
                elif state == "rejected":
                    parts.append(f"The i2c2midi REJECTED the routing: {st.get('message') or ''}.")
                    kind = "error"
                elif state == "mismatch":
                    parts.append(f"The i2c2midi took the routing but reports something else: {st.get('message') or ''}.")
                    kind = "error"
                else:
                    parts.append(f"Routing saved; the i2c2midi: {state}.")
                    kind = "warn"
            elif r.get("hub") is not None:
                parts.append("Routing sent to the i2c2midi (this Pi's hub doesn't confirm yet: restart it after updating).")
            else:
                parts.append("Routing saved, but the hub did not confirm (is the i2c2midi on a XIAO link?).")
                kind = "warn"
            if controls_on:
                if c is not None and c.get("conflict"):
                    return self._upload_conflict("the button functions", prefix, then)
                if c is None or not c.get("ok"):
                    self.set_status("error", pre + " ".join(parts) + f" Button functions rejected: {(c or {}).get('error') or 'unknown error'}")
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
            if got is not None and got != sent_all:
                parts.append("⚠ Reading it back from the Pi gave something else than was sent: press Load and check.")
                kind = "error"
            self.synced = sent_all
            self.set_status(kind, pre + " ".join(parts))
            self._update_dirty()
            self.refresh_all()
            if then:
                then()

        self.run(fn, done)

    def _upload_conflict(self, what: str, prefix: str, then: Optional[Callable]):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Changed on the Pi")
        box.setText(f"{what[0].upper() + what[1:]} on the Pi changed since you loaded it.")
        box.setInformativeText("Someone changed it there: by hand, or Kontrolleur on another computer. Nothing was "
                               "uploaded.\n\nLoad shows what's on the Pi now (your changes here are replaced). "
                               "Upload anyway replaces it with yours.")
        load = box.addButton("Load", QMessageBox.AcceptRole)
        force = box.addButton("Upload anyway", QMessageBox.DestructiveRole)
        box.addButton(QMessageBox.Cancel)
        box.exec()
        if box.clickedButton() is load:
            self.synced = None                  # a fresh load, not "kept your changes"
            self.load_routing()
        elif box.clickedButton() is force:
            self.upload_routing(prefix, then, force=True)
        else:
            self.set_status("warn", f"Not uploaded: {what} on the Pi changed since it was loaded.")

    def routing_dirty(self) -> bool:
        return self.synced is not None and self.synced != self.current_sources()

    # ── 16n ──
    def load_16n(self, then: Optional[Callable] = None, quiet: bool = False):
        on = self.on_ssh("faderbank")
        via = self.route("faderbank")
        if "16n" in self._loading:          # already reading it (in the background): that read counts
            if not quiet:
                self._want_full.add("16n")
            return then() if then else None
        self._loading.add("16n")
        start = self.bank.json

        def fn():
            if self.offline:
                return self._fixture("fader16.json")
            return self.host.tool(on, "fader16_tool.py", "export")

        def done(d, err):
            self._loading.discard("16n")
            nonlocal quiet
            if "16n" in self._want_full:     # the tab asked for it meanwhile: a full load after all
                self._want_full.discard("16n")
                quiet = False
            if err is not None or not d.get("ok"):
                if not quiet:
                    self.set_status("error", f"Could not read the 16n via {via}: {err}" if err is not None else
                                    f"16n via {via}: {d.get('error') or 'no answer'}")
            elif quiet and (self.bank.loaded or self.bank.json != start):
                pass                            # loaded or being edited meanwhile: leave it alone
            elif self.bank.json != start:
                self.set_status("warn", "You changed the 16n while it was loading: kept your changes. Press Load to "
                                        "replace them with what's on the 16n.")
            else:
                self.bank.apply(d)
                if quiet:                       # in the background: shown in Routing / ER-301, not "loaded"
                    self.bank.loaded = False
                    self.bank_cached = dt.datetime.now()
                    store.save_device_cache("16n", d)
                    self.refresh_all()
                    if then:
                        then()
                    return
                self.bank_cached = None
                store.save_device_cache("16n", d)
                self.synced_bank = self.bank.json
                fw = f"{self.bank.device}, {self.bank.firmware_text}" + ("" if self.bank.curves_supported else " (no curves)")
                note = "" if self.bank.ranges_supported else " Its firmware has no fader ranges yet (flash the 16n fork)."
                self.set_status("ok" if self.bank.ranges_supported else "warn", f"Loaded the 16n ({fw}) via {via}.{note}")
                self.refresh_all()
            if then:
                then()

        self.run(fn, done, busy=not quiet)

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
            store.save_device_cache("16n", dict(json.loads(js), ok=True, device=self.bank.device, firmware=self.bank.firmware,
                                                curves_supported=self.bank.curves_supported,
                                                ranges_supported=self.bank.ranges_supported,
                                                hires_supported=self.bank.hires_supported, port=self.bank.port))
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
    def load_pedals(self, then: Optional[Callable] = None, quiet: bool = False):
        on, via = self.on_ssh("pedals"), self.route("pedals")
        if "mpc20" in self._loading:          # already reading it (in the background): that read counts
            if not quiet:
                self._want_full.add("mpc20")
            return then() if then else None
        self._loading.add("mpc20")
        start = self.pedals.json

        def fn():
            if self.offline:
                return self._fixture("mpc20.json")
            return self.host.tool(on, "mpc20_tool.py", "export")

        def done(d, err):
            self._loading.discard("mpc20")
            nonlocal quiet
            if "mpc20" in self._want_full:     # the tab asked for it meanwhile: a full load after all
                self._want_full.discard("mpc20")
                quiet = False
            if err is not None or not d.get("ok"):
                if not quiet:
                    self.set_status("error", f"Could not read the MPC-20 via {via}: {err}" if err is not None else
                                    f"MPC-20 via {via}: {d.get('error') or 'no answer'}")
            elif quiet and (self.pedals.loaded or self.pedals.json != start):
                pass
            elif self.pedals.json != start:
                self.set_status("warn", "You changed the pedals while they were loading: kept your changes. Press Load "
                                        "to replace them with what's on the MPC-20.")
            else:
                self.pedals.apply(d)
                if quiet:
                    self.pedals.loaded = False
                    self.pedals_cached = dt.datetime.now()
                    store.save_device_cache("mpc20", d)
                    self.refresh_all()
                    if then:
                        then()
                    return
                self.pedals_cached = None
                store.save_device_cache("mpc20", d)
                self.synced_pedals = self.pedals.json
                used = sum(1 for i in self.pedals.inputs[:4] if i["type_id"] != 0)
                self.set_status("ok", f"Loaded the {self.pedals.device} (firmware {self.pedals.firmware}) via {via}: {used} of 4 jacks in use.")
                self.refresh_all()
            if then:
                then()

        self.run(fn, done, busy=not quiet)

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
            store.save_device_cache("mpc20", {"ok": True, "inputs": self.pedals.inputs, "device": self.pedals.device,
                                              "firmware": self.pedals.firmware, "port": self.pedals.port})
            n = r.get("written") or 0
            text = f"MPC-20 written and verified ({n} message{'' if n == 1 else 's'} changed)."
            if r.get("skipped"):
                text += f" Skipped (nothing plugged in): {', '.join(r['skipped'])}."
            self.refresh_all()
            self._then_routes(text)

        self.run(fn, done)

    # ── EC4 ──
    def load_ec4_cache(self, quiet: bool = False):
        if self.offline and (Path(self.offline) / "ec4-last.json").exists():
            self.ec4.apply(self._fixture("ec4-last.json"))
            self.ec4.received_at = dt.datetime.fromtimestamp((Path(self.offline) / "ec4-last.json").stat().st_mtime)
            ok = self.ec4.loaded
        else:
            ok = store.load_ec4_cache(self.ec4)
        if ok:
            self.synced_ec4 = self.ec4.fingerprint
        if quiet:
            self.refresh_all()
            return
        if ok:
            when = self.ec4.received_at.strftime("%-d %b %Y at %H:%M") if self.ec4.received_at else "earlier"
            self.set_status("info", f"Showing the EC4 setups received {when}. Press “Receive” to fetch them again.")
        else:
            self.set_status("info", "Press “Receive”, then on the EC4: Func → Setup → Send → “Send all setups” (hold the knob).")
        self.refresh_all()

    def _share_ec4(self):
        """Keep the EC4 setups on the Pi too, so the other computer shows the same ones (in the background)."""
        if self.offline or "shared" not in self.mapping.pi_features or not self.ec4.loaded:
            return
        data = {"ok": True, "version": self.ec4.version, "mem": self.ec4.mem, "setups": self.ec4.setups,
                "received_at": (self.ec4.received_at or dt.datetime.now()).timestamp()}
        host = self.host
        self.run(lambda: host.shared_put("ec4", data), lambda r, e: None, busy=False)

    def _sync_shared_ec4(self):
        """The EC4 setups kept on the Pi, when they are newer than the ones on this computer."""
        if self.offline or "shared" not in self.mapping.pi_features:
            return
        host = self.host

        def done(r, err):
            if err is not None or not (r or {}).get("ok") or not r.get("data"):
                if err is None and (r or {}).get("ok") and not r.get("data"):
                    self._share_ec4()           # nothing there yet: put ours
                return
            d = r["data"]
            when = dt.datetime.fromtimestamp(float(d.get("received_at") or r.get("time") or 0))
            mine = self.ec4.received_at
            if mine is not None and mine >= when - dt.timedelta(seconds=1):
                if mine > when + dt.timedelta(seconds=1):
                    self._share_ec4()           # ours is newer: update the Pi's copy
                return
            if self.ec4.loaded and self.synced_ec4 is not None and self.synced_ec4 != self.ec4.fingerprint:
                return                          # being edited here: leave it alone
            try:
                self.ec4.apply(d)
            except Exception:
                return
            self.ec4.received_at = when
            store.save_ec4_cache(self.ec4)
            self.synced_ec4 = self.ec4.fingerprint
            self.refresh_all()
            if self.mode in ("ec4", "pi", "er301"):
                self.set_status("info", f"Showing the EC4 setups received {when.strftime('%-d %b at %H:%M')} on another "
                                        "computer (kept on the Pi).")

        self.run(lambda: host.shared_get("ec4"), done, busy=False)

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
            self._share_ec4()
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
            self.ec4.received_at = dt.datetime.now()
            self._share_ec4()
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
        elif self.mode == "teletype":
            self.tt_import()
        elif self.mode == "macros":
            self.load_macros()
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
        elif self.mode == "teletype":
            self.tt_write()
        elif self.mode == "macros":
            self.upload_macros()
        else:
            self.upload_routing()

    # ── grid macros (grid_macros.json on the Pi, scripts/grid_macros_tool.py) ──
    def load_macros(self, quiet: bool = False, then: Optional[Callable] = None):
        st = self.macros
        if st.loading:
            return
        st.loading = True
        start = st.fingerprint if st.loaded else None

        def fn():
            if self.offline:
                p = Path(self.offline) / "grid-macros.json"
                return json.loads(p.read_text()) if p.exists() else {"ok": False, "error": "no grid-macros.json in the offline folder"}
            return self.host.macros_export()

        def done(e, err):
            st.loading = False
            if err is None and not (e or {}).get("ok"):
                err = clients.ToolError((e or {}).get("error") or "no answer")
            if err is not None:
                msg = str(err)
                if "grid_macros_tool.py" in msg and ("No such file" in msg or "can't open" in msg):
                    msg = "the Pi's checkout doesn't have the grid macros yet (git pull on the Pi)"
                st.error = f"Grid macros: {msg}"
                if not quiet:
                    self.set_status("error", f"Could not load the grid macros from {self.host.host}: {msg}")
            elif st.dirty and start is not None and st.fingerprint != start:
                self.set_status("warn", "You changed the macros while they were loading: kept your changes.")
            else:
                st.apply(e)
                if quiet:
                    pass                        # loaded alongside something else: its status stays
                elif not st.exists:
                    self.set_status("warn", "The Pi has no grid_macros.json yet: the grid-macros service creates it "
                                            "on its first start (install-pi.sh --grid-macros).")
                else:
                    sv = "service running" if st.service else "the grid-macros service isn't running"
                    self.set_status("ok", f"Loaded the grid macros from {self.host.host}: {st.used_slots()} slots · {sv}.")
            self.refresh_all()
            if then:
                then()

        self.run(fn, done, busy=not quiet)
        self.refresh_all()

    def upload_macros(self, force: bool = False):
        st = self.macros
        if not st.loaded:
            self.set_status("error", "Load the grid macros from the Pi first (↓ Load).")
            return
        data = json.loads(json.dumps(st.data))
        sent = st.fingerprint
        expect = None if force else st.sha

        def fn():
            if self.offline:
                return {"ok": True, "sha": "offline", "data": data, "service": st.service}
            return self.host.macros_install(data, expect)

        def done(r, err):
            if err is None and (r or {}).get("conflict"):
                self._macros_conflict()
                return
            if err is None and not (r or {}).get("ok"):
                err = clients.ToolError((r or {}).get("error") or "no answer")
            if err is not None:
                self.set_status("error", f"Could not upload the grid macros: {err}")
                return
            edited = st.fingerprint != sent
            st.sha = r.get("sha") or ""
            st.service = r.get("service")
            if edited:                      # changed here while uploading: keep those edits as unsaved
                st.synced = json.dumps(r.get("data") or data, sort_keys=True)
                self.set_status("warn", "Uploaded the grid macros; what you changed meanwhile isn't on the Pi yet.")
            else:
                st.data = macros_model._fill(r.get("data") or data)
                st.synced = st.fingerprint
                self.set_status("ok", "Uploaded the grid macros. The grid-macros service picks them up by itself.")
            self.refresh_all()

        self.run(fn, done)

    def _macros_conflict(self):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Changed on the Pi")
        box.setText("The grid macros on the Pi changed since you loaded them.")
        box.setInformativeText("Probably a slot was saved on the grid. Nothing was uploaded.\n\nLoad shows what's on "
                               "the Pi now (your changes here are replaced). Upload anyway replaces it with yours — "
                               "including slots saved on the grid since.")
        load = box.addButton("Load", QMessageBox.AcceptRole)
        force = box.addButton("Upload anyway", QMessageBox.DestructiveRole)
        box.addButton(QMessageBox.Cancel)
        box.exec()
        if box.clickedButton() is load:
            self.macros.synced = None
            self.load_macros()
        elif box.clickedButton() is force:
            self.upload_macros(force=True)
        else:
            self.set_status("warn", "Not uploaded: the grid macros on the Pi changed since they were loaded.")

    def macros_audition(self, bank: int, slot: int):
        """▶ in the Slot panel: the Pi loads the slot as if pressed on the grid (bank / slot 0-based here)."""
        def fn():
            if self.offline:
                return {"ok": True, "bank": bank, "active": {str(bank): slot}}
            return self.host.macros_audition(bank + 1, slot + 1)

        def done(r, err):
            if err is None and not (r or {}).get("ok"):
                err = clients.ToolError((r or {}).get("error") or "no answer")
            if err is not None:
                self.set_status("error", f"Slot {bank + 1}.{slot + 1} not loaded: {err}")
                return
            sv = dict(self.macros.service or {})
            sv.update({k: r[k] for k in ("bank", "active", "grid", "teletype") if k in r})
            self.macros.service = sv
            g = self.macros.glide(bank, slot)
            self.set_status("ok", f"Loading slot {bank + 1}.{slot + 1} on the Pi ({macros_model.fmt_secs(g)}).")
            self.refresh_all()

        self.run(fn, done)

    # ── Teletype ──
    def tt_import(self, pick: bool = False):
        folder = None
        if not pick:
            found = teletype.stick_candidates()
            if len(found) == 1:
                folder = found[0]
            elif found:
                self.set_status("info", "More than one stick with Teletype scenes: pick the folder.")
        if folder is None:
            start = str(teletype.stick_candidates()[0] if teletype.stick_candidates() else Path.home())
            d = QFileDialog.getExistingDirectory(self, "Folder with the Teletype's scenes (tt00s.txt …)", start)
            if not d:
                if not pick:
                    self.set_status("info", "No stick with Teletype scenes found. On the Teletype: stick in, WRITE TO USB; "
                                            "then put the stick into this computer.")
                return
            folder = Path(d)
        try:
            n = self.tt.import_folder(folder)
        except (OSError, ValueError) as e:
            return self.set_status("error", str(e))
        self.tt_dirty = {}
        if self.tt_scene not in self.tt.scenes or self.tt.scenes[self.tt_scene].empty:
            self.tt_scene = next((k for k in sorted(self.tt.scenes) if not self.tt.scenes[k].empty), 0)
        self.set_status("ok", f"Read {n} Teletype scenes from {folder}.")
        self.refresh_all()

    def tt_write(self):
        sc = self.tt.scene(self.tt_scene)
        if sc is None:
            return self.set_status("error", "Read the scenes from the stick first.")
        folder = Path(self.tt.imported_from) if self.tt.imported_from else None
        if folder is None or not folder.is_dir():
            found = teletype.stick_candidates()
            folder = found[0] if found else None
        if folder is None or not folder.is_dir():
            d = QFileDialog.getExistingDirectory(self, "Where to write the scene (the Teletype's stick)", str(Path.home()))
            if not d:
                return
            folder = Path(d)
        try:
            path = self.tt.write_for_teletype(self.tt_scene, folder)
        except OSError as e:
            return self.set_status("error", f"Couldn't write to {folder}: {e}")
        self.tt_dirty.pop(self.tt_scene, None)
        self.set_status("ok", f"Wrote {path}. On the Teletype: stick in, READ FROM USB → scene {self.tt_scene:02d}.")
        self.refresh_all()

    def set_tt_scene(self, n: int):
        self.tt_scene = int(n)
        self.s.setValue("ttScene", self.tt_scene)
        self.refresh_all()

    def tt_set_script(self, script: str, lines):
        sc = self.tt.scene(self.tt_scene)
        new = [t.strip().upper() for t in lines if t.strip()][:teletype.MAX_LINES]
        if sc is not None and sc.scripts.get(script, []) == new:
            return
        self.tt.set_script(self.tt_scene, script, new)
        self.tt_dirty[self.tt_scene] = True
        self.data_gen = getattr(self, "data_gen", 0) + 1
        self.stage.update()
        self._tt_rebuild_soon()

    def tt_set_text(self, lines):
        sc = self.tt.scene(self.tt_scene)
        if sc is None or lines == sc.text:
            return
        self.tt.set_text(self.tt_scene, lines)
        self.tt_dirty[self.tt_scene] = True
        self.stage.update()

    def _tt_rebuild_soon(self):
        """Rebuild the panel's line cards after typing, without taking the focus from the editor."""
        insp = self.tabs["teletype"][2]
        focus = QApplication.focusWidget()
        if focus is not None and isinstance(focus, QPlainTextEdit) and insp.isAncestorOf(focus):
            return          # the editor keeps its text; the drawing shows the result already
        insp.rebuild()

    def tt_listen(self, on: bool):
        if on:
            if self.offline:
                return self.set_status("info", "Offline: nothing to listen to.")
            self.tt_mon.start(self.host.host, self.host.identity)
        else:
            self.tt_mon.stop()
        self.inspector.rebuild()

    def _tt_live(self):
        """The monitor got something: repaint (throttled by the monitor), rebuild the panel only when its state changed."""
        if self.mode in ("teletype", "pi"):
            self.stage.update()
        mon = self.tt_mon
        sig = (mon.running, mon.connected, mon.port, mon.error, len(mon.seen) > 0)
        if sig != self._tt_sig:
            self._tt_sig = sig
            if self.mode == "teletype":
                self.inspector.rebuild()

    def tt_guess(self):
        """The scene whose USB sends explain most of what arrived live, if that's clear."""
        seen = self.tt_mon.seen
        if not seen or not self.tt.loaded:
            return None
        best, best_n = None, 0
        for n, sc in self.tt.scenes.items():
            an = self.tt.analysis(n)
            if an is None or sc.empty:
                continue
            usb = [s for s in an.sends if s.usb is not None]
            hit = sum(1 for v in seen.values()
                      if any(s.usb == v.ch and (s.hub_kind == v.kind or s.kind == v.kind) and (s.num is None or s.num == v.num)
                             for s in usb))
            if hit > best_n:
                best, best_n = n, hit
        return best

    def closeEvent(self, ev):
        self.tt_mon.stop()
        super().closeEvent(ev)

    def refresh_all(self):
        self.data_gen = getattr(self, "data_gen", 0) + 1
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
            self.bank.set_faders(c["faderbank"]["faders"])
            self.bank.options = c["faderbank"]["options"]
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
        if clients.FROZEN:
            v.addWidget(W.hint("For devices plugged into this computer. The tools for them are built into this app."))
        else:
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


class _WindowLog(QObject):
    """KONTROLLEUR_DEBUG_WINDOWS=1: log every window shown besides the main one (popups, tooltips, stray
    top-level widgets) to windows.log in the data folder, with where in the code it was shown from."""

    def __init__(self, main_window):
        super().__init__()
        self.main = main_window
        self.path = store.data_dir() / "windows.log"
        self.path.write_text(f"Kontrolleur window log, started {dt.datetime.now():%Y-%m-%d %H:%M:%S}\n")

    def eventFilter(self, o, e):
        if e.type() in (QEvent.Show, QEvent.Hide) and isinstance(o, QWidget) and o.isWindow() and o is not self.main:
            import traceback
            here = [ln.strip().splitlines()[0] for ln in traceback.format_stack(limit=12)[:-1] if "kontrolleur" in ln]
            par = o.parentWidget()
            g = o.geometry()
            line = (f"{dt.datetime.now():%H:%M:%S.%f}"[:-3] + f"  {'SHOW' if e.type() == QEvent.Show else 'hide'}  "
                    f"{type(o).__name__} '{o.objectName()}' flags={int(o.windowFlags().value) if hasattr(o.windowFlags(), 'value') else int(o.windowFlags()):#x} "
                    f"{g.width()}x{g.height()}+{g.x()}+{g.y()} parent={type(par).__name__ if par else None} "
                    f"mode={getattr(self.main, 'mode', '?')}\n")
            line += "".join(f"      {h}\n" for h in here[-4:])
            with open(self.path, "a") as fh:
                fh.write(line)
        return False


def main():
    if sys.platform.startswith("linux") and not os.environ.get("QT_QPA_PLATFORM") and os.environ.get("WAYLAND_DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "wayland;xcb")
    app = QApplication(sys.argv)
    app.setApplicationName("Kontrolleur")
    app.setOrganizationName("Kontrolleur")
    app.setDesktopFileName("kontrolleur")
    theme.init()
    w = Main()
    if os.environ.get("KONTROLLEUR_DEBUG_WINDOWS"):
        w._window_log = _WindowLog(w)
        app.installEventFilter(w._window_log)
    w.show()
    sys.exit(app.exec())
