"""Tests for MIDI_INPUTS (controls_config) and midi/device_inputs.py (no MIDI hardware needed).
Run:  python3 test/test_device_inputs.py"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import controls_config  # noqa: E402
from midi.device_inputs import DeviceInputs, Dispatcher, parse  # noqa: E402

fails = 0


def check(cond, what):
    global fails
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        fails += 1


def load(text):
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as f:
        f.write(text)
    try:
        return controls_config.load(f.name)
    finally:
        os.unlink(f.name)


def error(text):
    try:
        load(text)
    except controls_config.ControlsError as e:
        return str(e)
    return None


# ---- config ----------------------------------------------------------------------------
check(controls_config.load("/nonexistent/controls.py")["midi_inputs"] == [], "no file: no MIDI inputs")
check(load("BUTTONS = {}\n")["midi_inputs"] == [], "file without MIDI_INPUTS: none")
cfg = load('MIDI_INPUTS = [("16n", 16, "cc", 82, "slice_section"), ("pedal", 2, "cc", 109, "loop_record"),'
           ' ("ec4", 1, "note", 36, "block_up")]\n')
check(cfg["midi_inputs"] == [("16n", 16, "cc", 82, "slice_section"), ("pedal", 2, "cc", 109, "loop_record"),
                             ("ec4", 1, "note", 36, "block_up")], "MIDI_INPUTS loaded")
check("unknown device" in (error('MIDI_INPUTS = [("mpc", 1, "cc", 1, "loop_play")]') or ""), "unknown device refused")
check("channel" in (error('MIDI_INPUTS = [("16n", 17, "cc", 1, "loop_play")]') or ""), "channel 17 refused")
check("kind" in (error('MIDI_INPUTS = [("16n", 1, "pc", 1, "loop_play")]') or ""), "unknown kind refused")
check("number" in (error('MIDI_INPUTS = [("16n", 1, "cc", 128, "loop_play")]') or ""), "CC 128 refused")
check("unknown function" in (error('MIDI_INPUTS = [("16n", 1, "cc", 1, "midi")]') or ""), "'midi' refused as input function")
check("only trigger an action" in (error('MIDI_INPUTS = [("ec4", 1, "note", 1, "slice_fine")]') or ""),
      "a note can't drive a continuous function")
check("used twice" in (error('MIDI_INPUTS = [("16n", 1, "cc", 1, "loop_play"), ("16n", 1, "cc", 1, "slice_fine")]') or ""),
      "same message twice refused")
check(error('MIDI_INPUTS = [("16n", 1, "cc", 1, "loop_play"), ("ec4", 1, "cc", 1, "slice_fine")]') is None,
      "same CC on two devices is fine")

# ---- parse -------------------------------------------------------------------------------
check(parse([0xBF, 82, 64]) == (16, "cc", 82, 64), "CC parsed (channel 16)")
check(parse([0x90, 36, 100]) == (1, "note", 36, 127), "note on = 127")
check(parse([0x90, 36, 0]) == (1, "note", 36, 0), "note on velocity 0 = off")
check(parse([0x80, 36, 64]) == (1, "note", 36, 0), "note off = 0")
check(parse([0xE0, 0, 64]) is None and parse([0xB0, 1]) is None, "pitch bend / short messages ignored")


# ---- dispatch ----------------------------------------------------------------------------
class Router:
    def __init__(self, entries):
        self.cfg = {"midi_inputs": entries}
        self.ran = []
    def run(self, action, source=""): self.ran.append((action, source))


class Axes:
    def __init__(self): self.applied = []
    def apply(self, f, v, source=""): self.applied.append((f, v))


r = Router([("16n", 16, "cc", 82, "slice_section"), ("pedal", 2, "cc", 109, "loop_record"),
            ("ec4", 1, "note", 36, "block_up")])
a = Axes()
d = Dispatcher(r, a)
check(d.devices() == {"16n", "pedal", "ec4"}, "devices in use")
d.handle("16n", [0xBF, 82, 100])
check(a.applied == [("slice_section", 100)], "fader -> continuous function with its value")
d.handle("16n", [0xBE, 82, 100]); d.handle("ec4", [0xBF, 82, 100])
check(len(a.applied) == 1, "other channel / other device ignored")
d.handle("pedal", [0xB1, 109, 127])
d.handle("pedal", [0xB1, 109, 127])
check(r.ran == [("loop_record", "pedal ch2 cc109")], "footswitch fires once on the rising edge")
d.handle("pedal", [0xB1, 109, 0]); d.handle("pedal", [0xB1, 109, 127])
check(len(r.ran) == 2, "fires again after release")
d.handle("ec4", [0x90, 36, 90]); d.handle("ec4", [0x80, 36, 0]); d.handle("ec4", [0x90, 36, 90])
check([x[0] for x in r.ran[2:]] == ["block_up", "block_up"], "note on/off works as a button")
d.handle("pedal", [0xB1, 109, 30]); d.handle("pedal", [0xB1, 109, 70])
check(len(r.ran) == 5, "crossing 64 counts as a press")


class NullRouter:
    cfg = None


Dispatcher(NullRouter(), None).handle("16n", [0xBF, 82, 100])
check(True, "no router / no axes: nothing happens, no crash")

# ---- DeviceInputs: queue + hot-plug with a fake rtmidi ----------------------------------
PORTS = ["Midi Through:Midi Through Port-0 14:0", "16n:16n MIDI 1 28:0", "Faderfox EC4:Faderfox EC4 MIDI 1 32:0"]


class FakeIn:
    made = []
    def __init__(self, name=None):
        self.name, self.port, self.cb, self.closed = name, None, None, False
        FakeIn.made.append(self)
    def get_ports(self): return list(PORTS)
    def ignore_types(self, **k): pass
    def open_port(self, i): self.port = PORTS[i]
    def set_callback(self, cb): self.cb = cb
    def cancel_callback(self): self.cb = None
    def close_port(self): self.closed = True
    def delete(self): self.closed = True


class FakeRtmidi:
    MidiIn = FakeIn


r = Router([("16n", 16, "cc", 82, "slice_section"), ("pedal", 2, "cc", 109, "loop_record")])
a = Axes()
inp = DeviceInputs(FakeRtmidi, Dispatcher(r, a))
inp.scanner = FakeIn()
inp.sync()
check(set(inp.inputs) == {"16n"}, "only the 16n opened (EC4 unused)")
fader = inp.inputs["16n"][0]
check(fader.port == "16n:16n MIDI 1 28:0" and fader.name.startswith("DS 16n In "), "16n opened by name in its own client")
fader.cb(([0xBF, 82, 10], 0.0))
inp.feed("pedal", [0xB1, 109, 127])
check(a.applied == [] and r.ran == [], "callbacks only queue")
inp.poll()
check(a.applied == [("slice_section", 10)] and [x[0] for x in r.ran] == ["loop_record"], "poll() runs them")
r.cfg["midi_inputs"].append(("ec4", 1, "cc", 1, "loop_scrub"))
inp.sync()
check(set(inp.inputs) == {"16n", "ec4"}, "EC4 opened once MIDI_INPUTS uses it")
PORTS.remove("16n:16n MIDI 1 28:0")
inp.sync()
check(set(inp.inputs) == {"ec4"} and fader.closed, "unplugged 16n closed")
r.cfg["midi_inputs"] = []
inp.feed("ec4", [0xB0, 1, 5])
check(inp.events.empty(), "nothing queued without MIDI_INPUTS")
inp.sync()
check(inp.inputs == {}, "inputs closed when no longer used")
r.cfg["midi_inputs"] = [("ec4", 1, "cc", 1, "loop_scrub")]
inp.sync()
inp.shutdown()
check(inp.inputs == {} and inp.scanner is None and all(f.closed for f in FakeIn.made), "shutdown closes everything")
inp.sync()
check(inp.inputs == {}, "nothing reopens after shutdown")

print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
