"""Kontrolleur: the Macros tab and the grid in Routing, driven offscreen with offline data (no Pi needed).
Needs PySide6 (skipped without). Run:  python3 test/test_kontrolleur_macros.py"""
import json
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.join(HERE, "..", "kontrolleur"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PySide6.QtWidgets import QApplication
except ImportError:
    print("SKIP: PySide6 is not installed")
    sys.exit(0)

fails = 0


def check(cond, what):
    global fails
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        fails += 1


# offline data: the routing export from the default configs, and a grid_macros export
tmp = tempfile.mkdtemp()
fix = os.path.join(tmp, "fix")
os.makedirs(fix)
os.environ["DSEM_CONFIG_DIR"] = os.path.join(tmp, "cfg")
os.environ["XDG_CONFIG_HOME"] = os.path.join(tmp, "xdg")
os.environ["XDG_DATA_HOME"] = os.path.join(tmp, "xdgd")
os.environ["HOME"] = tmp                                  # macOS settings / data folders stay in here too
import subprocess  # noqa: E402

out = subprocess.run([sys.executable, os.path.join(HERE, "..", "scripts", "er301_routes_tool.py"), "export"],
                     capture_output=True, text=True, env=os.environ).stdout
with open(os.path.join(fix, "export.json"), "w") as f:
    f.write([ln for ln in out.splitlines() if ln.startswith("{")][-1])

from macros.store import default_data  # noqa: E402

d = default_data()
d["banks"][0][0] = {"saved_at": time.time(), "state": {
    "dualsense": {"block": 2, "section": 1, "fine": 3, "slice": 275, "freeze": {"left": True, "right": False}, "motion": False},
    "midi": {"values": [["cc", 16, 80, 100, "16n"], ["cc", 16, 81, 20, "16n"], ["cc", 1, 3, 64, "EC4"]]}}}
d["names"][0][0] = "intro"
d["banks"][0][2] = {"saved_at": time.time(), "state": {"midi": {"values": [["cc", 16, 80, 5, "16n"]]}}}
with open(os.path.join(fix, "grid-macros.json"), "w") as f:
    json.dump({"ok": True, "exists": True, "sha": "x", "path": "/home/ahab/.config/dualsense-eurorack-midi/grid_macros.json",
               "data": d, "service": {"ok": True, "bank": 0, "active": {"0": 0}, "grid": "/dev/ttyACM0", "teletype": "/dev/ttyGS0"}}, f)
os.environ["KONTROLLEUR_OFFLINE"] = fix
os.environ["KONTROLLEUR_MODE"] = "macros"

app = QApplication(sys.argv)
app.setApplicationName("Kontrolleur")
app.setOrganizationName("Kontrolleur")
from kontrolleur import theme  # noqa: E402
from kontrolleur.app import Main  # noqa: E402

theme.init()

w = Main()
w.resize(1400, 860)
w.show()


def settle(seconds=1.5):
    end = time.time() + seconds
    while time.time() < end:
        app.processEvents()
        time.sleep(0.02)


def grab_ok(what):
    img = w.grab()
    check(not img.isNull() and img.width() > 100, what)


settle(2.0)
mc = w.macros
check(mc.loaded and mc.used_slots() == 2, "macros: loaded from the offline export")
grab_ok("macros: tab paints")
for tab in ("Slot", "Bank", "Settings"):
    w.inspector.set_tab(tab)
    settle(0.3)
    grab_ok(f"macros: {tab} panel builds")
w.inspector.set_tab("Slot")

# names, slew, removing a value → unsaved changes
mc.set_name(0, 2, "drop")
mc.set_slew(0, 2, 3)
mc.remove_value(0, 0, 2)
w.changed(True)
settle(0.3)
check(w.dirty and w.dirty_lb.isVisible(), "macros: edits show as unsaved changes")
check([v[2] for v in __import__("kontrolleur.macros", fromlist=["x"]).midi_values(mc.slot(0, 0))] == [80, 81],
      "macros: a value taken out of a slot")
mc.copy_slot((0, 0), (3, 6))
mc.move_slot((0, 2), (0, 0))
check(mc.name(0, 0) == "drop" and mc.name(0, 2) == "intro" and mc.slot(3, 6) is not None,
      "macros: copy, and move swaps with the target")

# value names and destinations (16n defaults: ch 16 CC 80 …; default ER-301 routes)
from kontrolleur import macros as MC  # noqa: E402
from kontrolleur import ssp as S  # noqa: E402

snd = S.senders(w)
check(MC.value_name(w, "cc", 16, 80, "16n", snd) == "16n Fader 1", "macros: a 16n value is named by its fader")
check(MC.value_name(w, "cc", 9, 9, "EC4", snd) == "EC4 · ch 9 CC 9", "macros: unknown controls by their message")

# upload (offline) clears the dirty state; audition updates the active slot
w.upload_clicked()
settle(0.8)
check(not mc.dirty and not w.dirty, "macros: upload syncs")
w.macros_audition(0, 2)
settle(0.8)
check(mc.active(0) == 2 and "Loading slot 1.3" in w.status.text, "macros: audition reports the slot loading")

# backup round trip
blob = mc.export_set()
mc.clear(0, 0)
mc.import_set(json.loads(json.dumps(blob)))
check(mc.slot(0, 0) is not None and mc.name(0, 0) == "drop", "macros: backup file round trip")
try:
    mc.import_set({"nope": 1})
    check(False, "macros: a wrong file is refused")
except ValueError:
    check(True, "macros: a wrong file is refused")

# Routing: the grid card, at every level, folded and unfolded, with its panel
w.set_mode("pi")
settle(1.0)
drawing = w.tabs["pi"][3]
for level in ("normal", "scripts", "stages"):
    drawing.set_level(level)
    settle(0.3)
    grab_ok(f"routing: paints with the grid at level {level}")
    check("grid_jacks" in drawing._S, f"routing: the grid card is there ({level})")
drawing.toggle_fold("src_grid")
settle(0.2)
grab_ok("routing: grid card folded")
drawing.fold_all(True)
settle(0.2)
grab_ok("routing: everything folded")
drawing.fold_all(False)
w.select("pi_sel", "src_grid")
settle(0.3)
grab_ok("routing: grid panel builds")
mc.exists = False
w.refresh_all()
settle(0.3)
check("grid_jacks" not in drawing._S, "routing: no grid card where the grid macros aren't installed")
grab_ok("routing: paints without the grid")

w.close()
print(f"\n{'ALL PASSED' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
