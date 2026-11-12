"""Tests for midi/axis_router.py and AXES in controls_config.py (no controller needed).
Run:  python3 test/test_axis_router.py"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import controls_config  # noqa: E402
from midi.axis_router import AxisRouter  # noqa: E402

fails = 0


def check(cond, what):
    global fails
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        fails += 1


class Loop:
    playing = False
    loop_duration = 0.0
    quantize_subdivision = None


class CM:
    def __init__(self): self.loop = Loop()
    def get_current_loop_state(self): return self.loop


class Ctrl:
    def __init__(self):
        self.max_banks, self.max_blocks, self.slice_bank, self.sample_block = 7, 7, 0, 0
        self.current_fine_slice, self.window_position = 0, 0.0
        self.last_quantize_subdivision, self._last_quantize_toggle_time, self.quantize_on = 4, 0.0, True
        self.sent, self.dots = 0, 0
    def update_fine_slice(self, v): self.current_fine_slice = int((v / 255.0) * 15)
    def send_grain_slice(self, midiout): self.sent += 1
    def handle_dpad_left(self): self.slice_bank = max(0, self.slice_bank - 1)
    def handle_dpad_right(self): self.slice_bank = min(self.max_banks, self.slice_bank + 1)
    def update_player_dots(self): self.dots += 1


class Router:
    def __init__(self, axes): self.cfg = {"axes": axes}


def make(axes=None):
    c, cm = Ctrl(), CM()
    return c, cm, AxisRouter(c, cm, None, Router(axes) if axes is not None else None)


midi_calls = []
send = lambda: midi_calls.append(1)  # noqa: E731

# defaults = the behaviour before AXES existed
c, cm, a = make()
check(a.fn("left_x") == "slice_fine" and a.fn("dpad_x") == "slice_section" and a.fn("touch_x") == "loop_scrub"
      and a.fn("touch_y") == "quantize_grid" and a.fn("l2") == "midi", "defaults match the old fixed behaviour")
a.value("l2", 100, send)
check(midi_calls == [1], "midi: the axis sends its own CC")
a.value("left_x", 127, send)
check(c.current_fine_slice == 15 and c.sent == 1 and midi_calls == [1], "slice_fine: left stick sets the slice, no CC")
a.step("dpad_x", 1, send); a.step("dpad_x", 1, send); a.step("dpad_x", -1, send)
check(c.slice_bank == 1 and c.sent == 4, "slice_section on the D-pad: ±1 section, grain slice sent each time")

# re-assigned
c, cm, a = make({"right_x": "slice_block", "r2": "slice_section", "l2": "none", "left_y": "loop_scrub",
                 "right_y": "quantize_grid", "dpad_y": "slice_block"})
a.value("right_x", 127, send)
check(c.sample_block == 7 and c.sent == 1, "slice_block on the right stick: full right = block 7")
a.value("right_x", 127, send)
check(c.sent == 1, "slice_block: same block again sends nothing")
a.value("r2", 40, send)
check(c.slice_bank == 2, "slice_section on R2: 40/127 = section 2")
n = len(midi_calls)
a.value("l2", 90, send)
check(len(midi_calls) == n and c.sent == 2, "none: nothing happens")
a.value("left_y", 64, send)
check(c.window_position == 0.0, "loop_scrub: no loop playing, position untouched")
cm.loop.playing, cm.loop.loop_duration = True, 2.0
a.value("left_y", 127, send)
check(c.window_position == 1.0, "loop_scrub: playing loop follows the axis")
a.value("right_y", 0, send)
check(c.last_quantize_subdivision == 32 and cm.loop.quantize_subdivision == 32 and c.dots == 1, "quantize_grid: low = 1/32")
a.value("right_y", 127, send)
check(c.last_quantize_subdivision == 4, "quantize_grid: high = 1/4")
c.sample_block = 3
a.step("dpad_y", -1, send)
check(c.sample_block == 2, "slice_block on the D-pad: previous block")
a.step("dpad_y", -1, send); a.step("dpad_y", -1, send); a.step("dpad_y", -1, send)
check(c.sample_block == 0, "slice_block: stops at block 0")

# controls.py round trip through controls_config
d = tempfile.mkdtemp()
p = os.path.join(d, "controls.py")
open(p, "w").write("AXES = {'left_x': 'midi', 'right_x': 'slice_fine', 'dpad_x': 'midi'}\n")
cfg = controls_config.load(p)
check(cfg["axes"]["right_x"] == "slice_fine" and cfg["left_stick_x"] == "midi" and cfg["dpad_horizontal"] == "midi",
      "AXES loads; legacy keys follow it")
open(p, "w").write("DPAD_HORIZONTAL = 'midi'\nLEFT_STICK_X = 'slice_fine'\n")
cfg = controls_config.load(p)
check(cfg["axes"]["dpad_x"] == "midi" and cfg["axes"]["left_x"] == "slice_fine" and cfg["axes"]["touch_x"] == "loop_scrub",
      "old files without AXES keep working")
for bad in ("AXES = {'dpad_x': 'loop_scrub'}", "AXES = {'nose': 'midi'}", "AXES = {'l2': 'fly'}"):
    open(p, "w").write(bad + "\n")
    try:
        controls_config.load(p)
        check(False, f"rejects {bad}")
    except controls_config.ControlsError:
        check(True, f"rejects {bad}")

print("\nALL PASS" if not fails else f"\n{fails} FAILED")
sys.exit(1 if fails else 0)
