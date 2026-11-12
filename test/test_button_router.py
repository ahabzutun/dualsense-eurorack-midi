"""Tests for midi/button_router.py and controls_config.py (no controller needed).
Run:  python3 test/test_button_router.py"""
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import controls_config  # noqa: E402
from midi.button_router import ButtonRouter  # noqa: E402
from midi.dualsense_hid import DualSenseHID  # noqa: E402
from config.mappings import LONG_PRESS_DURATION  # noqa: E402

fails = 0


def check(cond, what):
    global fails
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        fails += 1


class Motors:
    def setLeftMotor(self, v): pass
    def setRightMotor(self, v): pass


class Loop:
    def __init__(self):
        self.recording = False; self.playing = False; self.midi_buffer = []; self.loop_duration = 0.0
        self.quantize_subdivision = None; self.log = []
    def start_recording(self): self.recording = True; self.log.append("rec_start")
    def stop_recording(self): self.recording = False; self.midi_buffer = [1]; self.log.append("rec_stop"); return True
    def start_playback(self, midiout, **kw): self.playing = True; self.log.append("play"); return True
    def stop_playback(self): self.playing = False; self.log.append("stop")
    def clear_loop(self): self.midi_buffer = []; self.log.append("clear"); return True
    def record_message(self, m): pass


class CM:
    def __init__(self):
        self.current_channel = 1; self.loop = Loop(); self.dpad_vertical_steps = [3, 3, 3]; self.log = []
    def get_current_loop_state(self): return self.loop
    def toggle_left_group_freeze(self): self.log.append("freezeL"); return "FROZEN"
    def toggle_right_stick_freeze(self): self.log.append("freezeR"); return "UNFROZEN"
    def step_to_cc_value(self, s): return s * 18


class Strum:
    def __init__(self): self.n = 0
    def arpeggiate(self, *a, **k): self.n += 1


class Ctrl:
    def __init__(self):
        self.current_channel = 1; self.motion_enabled = [False] * 3; self.smoothed_motion = {"tilt_x": 1, "tilt_y": 2}
        self.ds = Motors(); self.sample_block = 3; self.max_blocks = 7; self.quantize_on = False; self.window_position = 0
        self.strummer = Strum(); self.log = []; self.btn_repeat_interval = 0.05
    def get_midi_channel_byte(self, t): return t + 1          # MIDI ch 2
    def update_led_color(self): pass
    def update_player_dots(self): pass
    def is_motion_enabled(self): return self.motion_enabled[self.current_channel - 1]
    def send_grain_slice(self, m): self.log.append(("slice", self.sample_block))
    def handle_dpad_left(self): self.log.append("sec-")
    def handle_dpad_right(self): self.log.append("sec+")
    def toggle_quantize(self): self.log.append("quant")


class Out:
    def __init__(self): self.msgs = []
    def send_message(self, m): self.msgs.append(list(m))


class Clock:
    def get_current_bpm(self): return 120


def make(path):
    c, cm, out = Ctrl(), CM(), Out()
    return ButtonRouter(c, cm, Clock(), out, path=path), c, cm, out


tmp = tempfile.mkdtemp()
missing = os.path.join(tmp, "nope.py")

# ---- defaults (no file) = today's behaviour ----
r, c, cm, out = make(missing)
L = LONG_PRESS_DURATION
r.on_button("l1", True, now=0); check(cm.loop.log == [], "L1 press: nothing yet")
r.on_button("l1", False, now=0.2); check(cm.loop.log == ["clear"], "L1 short: loop cleared on release")
r.on_button("r1", True, now=1); r.on_button("r1", False, now=1 + L + 0.1)
check(cm.loop.log[-1] == "rec_start", "R1 hold: recording starts")
r.on_button("r1", True, now=5); r.on_button("r1", False, now=5 + L + 0.1)
check(cm.loop.log[-1] == "rec_stop", "R1 hold again: recording stops")
r.on_button("r1", True, now=9); r.on_button("r1", False, now=9.2)
check(cm.loop.log[-1] == "play", "R1 short: playback starts")
r.on_button("r1", True, now=10); r.on_button("r1", False, now=10.1)
check(cm.loop.log[-1] == "stop", "R1 short again: playback stops")

n = len(cm.loop.log)
r.on_button("l1", True, now=20); r.on_button("r1", True, now=20.05)
check(c.motion_enabled[0] is True, "L1+R1: motion on")
r.on_button("r1", False, now=20.3); r.on_button("l1", False, now=20.4)
check(len(cm.loop.log) == n and c.motion_enabled[0], "L1+R1 release: no clear / play, motion stays on")
r.on_button("r1", True, now=21); r.on_button("l1", True, now=21.05); r.on_button("l1", False, now=21.2); r.on_button("r1", False, now=21.3)
check(c.motion_enabled[0] is False and len(cm.loop.log) == n, "R1+L1 again: motion off, no solo actions")

r.on_button("create", True, now=30); check(c.sample_block == 2, "Create: block down on press")
r.on_button("options", True, now=30.1); check(c.sample_block == 0, "Create+Options: block reset")
r.on_button("options", False, now=30.2); r.on_button("create", False, now=30.3)
check(c.sample_block == 0, "releases change nothing")
r.on_button("options", True, now=31); r.on_button("options", False, now=31.1); check(c.sample_block == 1, "Options: block up")
r.on_button("ps", True, now=32); check(c.strummer.n == 1, "PS: strum on press")
r.on_button("l3", True, now=33); r.on_button("r3", True, now=33.5); check(cm.log == ["freezeL", "freezeR"], "L3 / R3: freeze")
r.on_button("touchpad", True, now=34); check(c.log[-1] == "quant", "touchpad click: quantize")
r.on_button("mic", True, now=35); r.on_button("mic", False, now=35.1); check(out.msgs == [], "mic: nothing by default")

r.on_button("cross", True, now=40); check(out.msgs[-1] == [0xB1, 14, 127], "✕: CC14 127 on ch 2")
r.tick(now=40.03); check(len(out.msgs) == 1, "no repeat before 50 ms")
r.tick(now=40.06); check(out.msgs[-1] == [0xB1, 14, 127] and len(out.msgs) == 2, "held ✕ repeats CC 127")
r.on_button("cross", False, now=40.1); check(out.msgs[-1] == [0xB1, 14, 0], "✕ release: CC14 0")
r.tick(now=40.3); check(len(out.msgs) == 3, "no repeats after release")

# ---- reassigned file + hot reload ----
path = os.path.join(tmp, "controls.py")
with open(path, "w") as f:
    f.write("BUTTONS = {'cross': dict(tap='loop_record'), 'mic': dict(tap='midi'), "
            "'l1': dict(tap='midi', hold='loop_clear'), 'ps': dict(tap='section_up')}\n"
            "CHORDS = [('triangle', 'square', 'motion')]\nDPAD_HORIZONTAL = 'midi'\n")
r, c, cm, out = make(path)
check(r.dpad_horizontal == "midi" and r.left_stick_x == "slice_fine", "modes from file (defaults for missing)")
r.on_button("cross", True, now=0); check(cm.loop.log == [], "✕ = loop_record: nothing on press")
r.on_button("cross", False, now=0.1); check(cm.loop.log == ["rec_start"] and out.msgs == [], "✕ release: recording, no CC")
r.post("mic", True); r.poll(); check(out.msgs[-1] == [0xB1, 29, 127], "mic via HID queue: CC29 127")
r.post("mic", False); r.poll(); check(out.msgs[-1] == [0xB1, 29, 0], "mic release: CC29 0")
r.on_button("l1", True, now=2); check(out.msgs[-1] == [0xB1, 24, 127], "L1 = midi: CC24 on press")
r.on_button("l1", False, now=2 + L + 0.1); check(out.msgs[-1] == [0xB1, 24, 0] and cm.loop.log[-1] == "clear", "L1 hold: CC24 0 + loop clear")
r.on_button("l1", True, now=5); r.on_button("l1", False, now=5.1); check(cm.loop.log[-1] == "clear" and cm.loop.log.count("clear") == 1, "L1 short: only the CC")
r.on_button("ps", True, now=6); check(c.log[-2:] == ["sec+", ("slice", 3)], "PS = section_up (+ slice sent)")
r.on_button("l1", True, now=7); r.on_button("r1", True, now=7.05); check(c.motion_enabled[0] is False, "old L1+R1 chord gone")
r.on_button("r1", False, now=7.1); r.on_button("l1", False, now=7.2)
r.on_button("triangle", True, now=8); r.on_button("square", True, now=8.05); check(c.motion_enabled[0] is True, "new △+□ chord: motion on")
r.on_button("square", False, now=8.1); r.on_button("triangle", False, now=8.2)
check(out.msgs[-1] == [0xB1, 22, 0], "△/□ still send their CCs inside the chord")

with open(path, "w") as f:
    f.write("BUTTONS = {'cross': dict(tap='midi')}\n")
os.utime(path, (time.time() + 5, time.time() + 5))
r.maybe_reload(now=time.time() + 10)
r.on_button("cross", True, now=20); check(out.msgs[-1] == [0xB1, 14, 127], "hot reload: ✕ back to MIDI")
r.on_button("cross", False, now=20.1)
with open(path, "w") as f:
    f.write("BUTTONS = {'cross': dict(tap='explode')}\n")
os.utime(path, (time.time() + 20, time.time() + 20))
r.maybe_reload(now=time.time() + 30)
r.on_button("cross", True, now=30); check(out.msgs[-1] == [0xB1, 14, 127], "broken file: previous setting kept")

# ---- validation ----
def err(text):
    p = os.path.join(tmp, "bad.py")
    with open(p, "w") as f:
        f.write(text)
    try:
        controls_config.load(p)
        return None
    except controls_config.ControlsError as e:
        return str(e)
check("unknown action" in (err("BUTTONS = {'cross': dict(tap='boom')}") or ""), "rejects unknown action")
check("unknown button" in (err("BUTTONS = {'xbox': dict(tap='midi')}") or ""), "rejects unknown button")
check("only works as tap" in (err("BUTTONS = {'l1': dict(tap='none', hold='midi')}") or ""), "rejects hold='midi'")
check("two different" in (err("CHORDS = [('l1', 'l1', 'motion')]") or ""), "rejects chord with one button")
check("DPAD_HORIZONTAL" in (err("DPAD_HORIZONTAL = 'x'") or ""), "rejects bad D-pad mode")
check("SyntaxError" in (err("BUTTONS = {") or ""), "reports syntax errors")

# ---- mic button in the HID input report ----
hid = DualSenseHID()
seen = []
hid.on_mic = seen.append
usb = bytearray(64); usb[0] = 0x01
hid._parse_input(bytes(usb)); check(seen == [], "USB report, mic up: no event")
usb[10] = 0x04; hid._parse_input(bytes(usb)); check(seen == [True], "USB report, mic down: pressed")
hid._parse_input(bytes(usb)); check(seen == [True], "unchanged: no repeat")
usb[10] = 0x01; hid._parse_input(bytes(usb)); check(seen == [True, False], "PS bit only: mic released")
bt = bytearray(78); bt[0] = 0x31; bt[11] = 0x04; hid._parse_input(bytes(bt)); check(seen[-1] is True, "Bluetooth report: mic pressed")

print(f"\n{'ALL PASS' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
