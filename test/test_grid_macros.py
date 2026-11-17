"""Tests for the grid macros (src/macros/, state/snapshot_io.py) — no grid, Teletype or MIDI needed.
Run:  python3 test/test_grid_macros.py"""
import json
import os
import pty
import select
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
sys.path.insert(0, SRC)

from macros import mext  # noqa: E402
from macros.engine import MacroEngine, fader_row  # noqa: E402
from macros.glide import Glide  # noqa: E402
from macros.hub_pickup import HubPickup  # noqa: E402
from macros.pickup import Pickup  # noqa: E402
from macros.providers import DualSenseClient, DualSenseState, MidiState, Provider, TeletypeLeds  # noqa: E402
from macros.proxy import GridSplitter  # noqa: E402
from macros.store import MacroStore  # noqa: E402

fails = 0


def check(cond, what):
    global fails
    print(("PASS " if cond else "FAIL ") + what)
    if not cond:
        fails += 1


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


# ---- mext ----------------------------------------------------------------------------------------
p = mext.StreamParser(mext.DEVICE_LEN)
msgs = p.feed(bytes([0x21, 3, 4, 0x20]))
check(msgs == [bytes([0x21, 3, 4])], "parser: complete key message out, partial kept")
check(p.feed(bytes([3, 4])) == [bytes([0x20, 3, 4])], "parser: partial completed by next chunk")
check(p.feed(bytes([0xEE, 0x21, 1, 1])) == [bytes([0x21, 1, 1])] and p.dropped == 1, "parser: resyncs over junk")
lv = list(range(16)) * 4
m = mext.level_map(8, 0, lv)
check(len(m) == 35 and m[0] == 0x1A and m[1] == 8, "level map: 35 bytes, quad x 8")
check(mext.unpack_level_map(m) == (8, 0, lv), "level map round trip")

# ---- splitter ------------------------------------------------------------------------------------
pressed = []
sp = GridSplitter(on_macro_key=lambda x, y, d: pressed.append((x, y, d)))
out = sp.from_grid(bytes([0x21, 2, 5, 0x21, 9, 1, 0x20, 2, 5, 0x03, 16, 8]))
check(pressed == [(2, 5, True), (2, 5, False)], "splitter: left keys go to the macros")
check(out == bytes([0x21, 9, 1]), "splitter: right keys go to Teletype, size answer stays with the Pi")
ans, to_grid = sp.from_teletype(bytes([0x05]))
check(ans == bytes([0x03, 16, 8]) and to_grid == b"", "splitter: answers Teletype's size query itself")
seen = []
sp.on_teletype_leds = lambda qx, qy, l: seen.append((qx, l[0]))
left = mext.level_map(0, 0, [15] * 64)
right = mext.level_map(8, 0, [7] * 64)
ans, to_grid = sp.from_teletype(left + right[:20])
check(ans == b"" and to_grid == b"", "splitter: Teletype's left quad dropped, half a right quad waits")
ans, to_grid = sp.from_teletype(right[20:])
check(to_grid == right and seen == [(8, 7)], "splitter: right quad passed on and mirrored")
check(sp.resend_teletype_half() == right, "splitter: keeps Teletype's half for a grid reconnect")
_, to_grid = sp.from_teletype(bytes([0x12, 0x11, 2, 2, 0x11, 12, 2]))
check(to_grid == bytes([0x11, 12, 2]), "splitter: all-off and left LEDs dropped, right LED passed")

# ---- glide ---------------------------------------------------------------------------------------
g = Glide()
got = []
g.glide("a", 0, 100, 1.0, 0.0, got.append)
g.tick(0.5)
check(got == [50], "glide: halfway at half time")
g.tick(1.2)
check(got[-1] == 100 and not g.active, "glide: lands exactly on the target and ends")
got.clear()
g.glide("b", None, 64, 2.0, 0.0, got.append)
check(got == [64] and not g.active, "glide: unknown start value is set at once")
jumps = []
g.jump("j", "scene", 2.0, 0.0, jumps.append, at="end")
g.tick(1.0)
check(jumps == [], "jump at end: nothing before the end")
g.tick(2.0)
check(jumps == ["scene"], "jump at end: set at the end")
g.jump("j", "x", 2.0, 0.0, jumps.append, at="start")
check(jumps[-1] == "x", "jump at start: set at once")
g.glide("c", 0, 100, 1.0, 0.0, got.append)
check(g.cancel("c") and not g.gliding("c"), "glide: cancel (hand takes over)")

# ---- pickup --------------------------------------------------------------------------------------
pk = Pickup(window=2)
pk.arm("f", 64)
check(pk.accept("f", 10) is False, "pickup: far below is held back")
check(pk.accept("f", 40) is False, "pickup: still below")
check(pk.accept("f", 90) is True and not pk.armed("f"), "pickup: crossing the value picks up")
pk.arm("f", 64)
check(pk.accept("f", 63) is True, "pickup: within the window picks up")
pk.arm("f", 64)
pk.accept("f", 100)
pk.update_target("f", 110)
check(pk.accept("f", 105) is True, "pickup: target follows the glide (fader catches a moving value)")
check(Pickup().accept("never armed", 3) is True, "pickup: unarmed controls pass")

# ---- hub pickup ----------------------------------------------------------------------------------
hp = HubPickup(port=0, start_thread=False)
hp.handle(json.dumps({"source": "16n", "pickup": [[16, 80, 100]]}).encode())
check(hp.allow("16n:16n MIDI 1 20:0", [0xBF, 80, 10]) is False, "hub: 16n fader held back after a load")
check(hp.allow("Faderfox EC4", [0xBF, 80, 10]) is True, "hub: other controllers unaffected")
check(hp.allow("16n:16n", [0x9F, 60, 100]) is True, "hub: notes unaffected")
hp.note_macro([0xBF, 80, 20])
check(hp.allow("16n:16n", [0xBF, 80, 21]) is True, "hub: picks up the value the macros glided to")
check(hp.allow("16n:16n", [0xBF, 80, 0]) is True, "hub: free afterwards")

# ---- store ---------------------------------------------------------------------------------------
tmpdir = tempfile.mkdtemp()
path = os.path.join(tmpdir, "grid_macros.json")
st = MacroStore(path)
st.set_slew(0, 2, 4)
st.set_slot(0, 2, {"midi": {"values": []}})
st2 = MacroStore(path)
check(st2.slew(0, 2) == 4 and st2.slot(0, 2) is not None, "store: slot and slew saved to the file")
check(st2.glide_time(0, 2) == 8.0, "store: slew step 4 = 8 s by default")
with open(path, "w") as f:
    json.dump({"settings": {"glide_times": [0, 1, 2, 3, 4, 5], "bogus": 1}, "banks": "junk",
               "slew": [[9, -1]]}, f)
time.sleep(0.01)
os.utime(path, (time.time() + 5, time.time() + 5))
check(st2.reload_if_changed(), "store: notices an outside change")
check(st2.glide_time(0, 0) == 5 and st2.slew(0, 1) == 0 and "bogus" not in st2.settings,
      "store: bad values clamped / ignored")
st2.data["settings"]["bank_providers"] = {"2": ["midi"]}
check(st2.providers_for(2) == ["midi"] and "dualsense" in st2.providers_for(0), "store: per-bank providers")


# ---- MIDI state ----------------------------------------------------------------------------------
class Out:
    def __init__(self):
        self.msgs = []

    def __call__(self, m):
        self.msgs.append(list(m))


out = Out()
fb = []
armed = []
settings = MacroStore().settings
ms = MidiState(out, settings, feedback=lambda s, m: fb.append((s, list(m))), arm_pickup=armed.append)
ms.glide = Glide()
ms.pickup = Pickup()
ms.observe("16n:16n MIDI 1 20:0", [0xBF, 80, 100])
ms.observe("Faderfox EC4:EC4 MIDI 1", [0xB0, 99, 0])
ms.observe("Faderfox EC4:EC4 MIDI 1", [0xB0, 98, 5])
ms.observe("Faderfox EC4:EC4 MIDI 1", [0xB0, 6, 64])
ms.observe("Faderfox EC4:EC4 MIDI 1", [0xB0, 38, 1])
ms.observe("Some Other Device", [0xB0, 1, 1])
check(ms.values == {("cc", 16, 80): 100, ("nrpn", 1, 5): (64 << 7) | 1},
      "midi: CC and NRPN remembered from the configured controllers only")
snap = ms.capture()
ms.observe("16n:16n MIDI 1 20:0", [0xBF, 80, 0])
ms.observe("Faderfox EC4:EC4 MIDI 1", [0xB0, 38, 3])
ms.restore(snap, ms.glide, 1.0, 0.0, "end")
check(armed and armed[0] == [[16, 80, 0]], "midi: 16n fader armed for pickup at its current value")
ms2 = MidiState(Out(), settings, arm_pickup=armed.append)
ms2.glide, ms2.pickup = Glide(), Pickup()
ms2.observe("16n:16n MIDI 1 20:0", [0xBF, 81, 33])
armed_before = len(armed)
ms2.restore({"values": [["cc", 16, 81, 33, "16n"]]}, ms2.glide, 1.0, 0.0, "end")
check(len(armed) == armed_before and not ms2.pickup.armed(("cc", 16, 81)),
      "midi: a fader whose value doesn't change isn't armed")
ms.glide.tick(0.5)
check([0xBF, 80, 50] in out.msgs, "midi: 16n value glides through the macro port")
check(any(s == "EC4" for s, _ in fb), "midi: EC4 gets its value back")
ms.observe("16n:16n MIDI 1 20:0", [0xBF, 80, 10])
check(ms.glide.gliding(("cc", 16, 80)), "midi: fader below the gliding value doesn't take over")
ms.observe("16n:16n MIDI 1 20:0", [0xBF, 80, 70])
check(not ms.glide.gliding(("cc", 16, 80)) and ms.values[("cc", 16, 80)] == 70,
      "midi: fader crossing the value takes over and stops its glide")
ms.glide.tick(1.5)
check(ms.values[("nrpn", 1, 5)] == (64 << 7) | 1, "midi: NRPN ends on the saved 14-bit value")
check(out.msgs[-4:] == [[0xB0, 99, 0], [0xB0, 98, 5], [0xB0, 6, 64], [0xB0, 38, 1]],
      "midi: NRPN sent as the 4-message sequence")


# ---- engine --------------------------------------------------------------------------------------
class Fake(Provider):
    def __init__(self, name):
        self.name = name
        self.value = 0
        self.restored = []

    def capture(self):
        return {"v": self.value}

    def restore(self, data, glide, duration, now, jump_at):
        self.restored.append((data["v"], duration))
        glide.glide((self.name,), self.value, data["v"], duration, now, self._set)

    def _set(self, v):
        self.value = v


clk = Clock()
st = MacroStore(os.path.join(tmpdir, "engine.json"))
st.data["settings"]["providers"] = ["fake"]
fake = Fake("fake")
eng = MacroEngine(st, {"fake": fake}, clock=clk, log=None)
fake.value = 42
eng.key(1, 0, True)                      # save slot 0: press …
clk.advance(0.2)
eng.tick()
check(st.slot(0, 0) is None, "engine: a short press of save doesn't save")
clk.advance(0.4)
eng.tick()
check(st.slot(0, 0) is not None and st.slot(0, 0)["state"]["fake"] == {"v": 42}, "engine: hold save = save")
eng.key(1, 0, False)
leds = eng.leds()
check(leds[0] == 15 and leds[1] == 15, "engine: bank 1 lit, save key flashes after saving")
fake.value = 0
eng.key(1, fader_row(3), True)           # slew step 3 = 2 s
check(st.slew(0, 0) == 3, "engine: fader sets the slew")
eng.key(1, 7, True)                      # load
check(fake.restored == [(42, 2.0)], "engine: load restores with the slot's glide time")
clk.advance(1.0)
eng.tick()
check(fake.value == 21, "engine: glides halfway in a second")
check(8 <= eng.leds()[7 * 8 + 1] <= 15 and eng.loading, "engine: load key pulses while gliding")
clk.advance(1.5)
eng.tick()
check(fake.value == 42 and eng.loading is None and eng.leds()[7 * 8 + 1] == 15,
      "engine: arrives, load key stays bright")
check(eng.load(3) is False, "engine: loading an empty slot does nothing")
eng.key(0, 4, True)
check(eng.bank == 4 and eng.leds()[4 * 8] == 15 and eng.leds()[0] == 5, "engine: bank switch, used bank dim")
eng.key(1, 7, True)
check(fake.restored[-1:] == [(42, 2.0)] and len(fake.restored) == 1, "engine: other bank, slot 1 is empty")
eng.key(0, 0, True)
eng.key(1, 0, True)
eng.key(1, 7, True)                      # save held + load = clear
clk.advance(1.0)
eng.tick()
eng.key(1, 0, False)
check(st.slot(0, 0) is None, "engine: save held + load clears the slot (and doesn't save again)")
eng.force_redraw()
check(eng.tick() is not None and eng.tick() is None, "engine: tick returns LEDs only when they change")

# ---- teletype provider ---------------------------------------------------------------------------
tl = TeletypeLeds()
check(tl.capture() is None, "teletype: nothing to remember before Teletype drew")
tl.update([0] * 63 + [9])
check(tl.capture() == {"leds": [0] * 63 + [9]} and tl.describe(tl.capture()) == "1 lit", "teletype: LEDs kept")

# ---- DualSense script state (real controller classes, no hardware) ------------------------------
from state.channel_manager import ChannelManager  # noqa: E402
from midi.controller import MIDIController  # noqa: E402
from state import snapshot_io  # noqa: E402


class MidiOut:
    def __init__(self):
        self.msgs = []

    def send_message(self, m):
        self.msgs.append(list(m))


cm = ChannelManager()
ctl = MIDIController(cm)
ctl.sample_block, ctl.slice_bank, ctl.current_fine_slice = 3, 2, 5
cm.current_l2, cm.current_r2, cm.current_left_x, cm.current_left_y = 100, 20, 64, 30
cm.toggle_left_group_freeze()
state = snapshot_io.capture(ctl, cm)
check(state["slice"] == 3 * 128 + 2 * 16 + 5 and state["freeze"]["left"] and state["freeze"]["l2"] == 100,
      "dualsense: capture slice position and freeze")
ctl.sample_block, ctl.slice_bank, ctl.current_fine_slice = 0, 0, 0
cm.toggle_left_group_freeze()
mo = MidiOut()
snapshot_io.apply(ctl, cm, json.loads(json.dumps(state)), mo)
check(ctl.compute_grain_slice() == state["slice"], "dualsense: apply restores the slice")
check(cm.get_current_freeze_state().left_stick_frozen and cm.get_current_freeze_state().l2_value == 100,
      "dualsense: apply restores freeze with its held values")
nrpn_val = (state["slice"] * 16383) // 1023
check([0xB1, 6, (nrpn_val >> 7) & 0x7F] in mo.msgs, "dualsense: the slice NRPN is sent")
check([0xB1, 7, 100] in mo.msgs, "dualsense: the frozen L2 value is sent")
snapshot_io.apply(ctl, cm, {"block": 99, "freeze": {"left": False}}, MidiOut())
check(ctl.sample_block == ctl.max_blocks and not cm.get_current_freeze_state().left_stick_frozen,
      "dualsense: out-of-range block clamped, unfreeze works")

# the slice can be driven from anything the DualSense script listens to (here: pedals via MIDI_INPUTS);
# the snapshot holds the script's slice state, so it doesn't matter which control moved it
from midi.axis_router import AxisRouter  # noqa: E402
from midi.button_router import ButtonRouter  # noqa: E402
from midi.device_inputs import Dispatcher  # noqa: E402

with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as f:
    f.write("MIDI_INPUTS = [('pedal', 2, 'cc', 105, 'section_up'), ('pedal', 2, 'cc', 106, 'block_up'),\n"
            "               ('pedal', 2, 'cc', 107, 'slice_fine')]\n")
    pedal_cfg = f.name
cm_p = ChannelManager()
ctl_p = MIDIController(cm_p)
mo_p = MidiOut()
rt_p = ButtonRouter(ctl_p, cm_p, None, mo_p, path=pedal_cfg)
disp = Dispatcher(rt_p, AxisRouter(ctl_p, cm_p, mo_p, rt_p))
for cc in (105, 105, 106):
    disp.handle("pedal", [0xB1, cc, 127])
    disp.handle("pedal", [0xB1, cc, 0])
disp.handle("pedal", [0xB1, 107, 127])            # an expression pedal as fine slice
saved = snapshot_io.capture(ctl_p, cm_p)
check((saved["block"], saved["section"], saved["fine"]) == (1, 2, 15),
      "pedals: slice moved by pedals is in the snapshot")
disp.handle("pedal", [0xB1, 106, 127])
disp.handle("pedal", [0xB1, 107, 0])
snapshot_io.apply(ctl_p, cm_p, saved, mo_p)
check(ctl_p.compute_grain_slice() == saved["slice"], "pedals: loading restores it, whoever moves it next")
os.unlink(pedal_cfg)

srv = snapshot_io.SnapshotServer(port=0)
port = srv.sock.getsockname()[1]
client = DualSenseClient(port=port, timeout=0.05)
import threading  # noqa: E402
stop = threading.Event()


def serve():
    while not stop.is_set():
        srv.poll(ctl, cm, MidiOut())
        time.sleep(0.005)


th = threading.Thread(target=serve, daemon=True)
th.start()
ds = DualSenseState(client)
cap = ds.capture()
check(cap is not None and cap["block"] == ctl.max_blocks, "dualsense: capture over the local socket")
cap["block"] = 1
g = Glide()
ds.restore(cap, g, 0.0, 0.0, "end")
time.sleep(0.05)
check(ctl.sample_block == 1, "dualsense: restore over the local socket")
stop.set()
th.join()
check(DualSenseState(DualSenseClient(port=1, timeout=0.05)).capture() is None,
      "dualsense: script not running = nothing captured (no hang)")

# ---- keyboard forwarder --------------------------------------------------------------------------
from keyboard_forward import ReportState  # noqa: E402

ks = ReportState()
check(ks.event(42, 1) == bytes([0x02, 0, 0, 0, 0, 0, 0, 0]), "keyboard: left shift = modifier bit 1")
check(ks.event(30, 1) == bytes([0x02, 0, 0x04, 0, 0, 0, 0, 0]), "keyboard: shift + a")
check(ks.event(30, 2) is None, "keyboard: autorepeat not sent (Teletype repeats itself)")
check(ks.event(30, 0) == bytes([0x02, 0, 0, 0, 0, 0, 0, 0]), "keyboard: a released")
check(ks.event(42, 0) == bytes(8), "keyboard: all released")
check(ks.event(240, 1) is None, "keyboard: unknown keys ignored")
for code in (2, 3, 4, 5, 6, 7, 8):
    last = ks.event(code, 1)
check(last == bytes([0, 0] + [0x01] * 6), "keyboard: more than 6 keys = roll-over report")
check(ks.release_all() == bytes(8), "keyboard: release all when a keyboard goes away")
check(ks.event(59, 1)[2] == 0x3A and ks.event(103, 1)[3] == 0x52, "keyboard: F1 and arrow up")

# the writer waits for Teletype to collect the previous report instead of dropping this one
import contextlib, io, tempfile, threading  # noqa: E402
from keyboard_forward import HidWriter  # noqa: E402
_fifo = os.path.join(tempfile.mkdtemp(), "hidg")
os.mkfifo(_fifo)
_rd = os.open(_fifo, os.O_RDONLY | os.O_NONBLOCK)
hw = HidWriter(_fifo, timeout=0.2)
with contextlib.redirect_stdout(io.StringIO()):
    while hw.write(bytes(4096)):
        pass                                  # fill the pipe: Teletype "not collecting"
check(hw.dropped == 1, "keyboard: gives up after the timeout when nothing is collected")
threading.Timer(0.05, lambda: os.read(_rd, 1 << 20)).start()   # Teletype collects a moment later
check(hw.write(bytes(8)) and hw.waited == 1, "keyboard: waits for the previous report instead of dropping")
os.close(_rd)
check(HidWriter("/nonexistent/hidg0").write(bytes(8)) is False, "keyboard: no gadget = quiet no-op")

# ---- the hub (midi_passthrough.py) with a stand-in rtmidi ---------------------------------------
import types  # noqa: E402


class _FakeMidi:
    def __init__(self, *a, **k):
        self.sent = []

    def get_ports(self):
        return []

    def send_message(self, m):
        self.sent.append(list(m))

    def open_port(self, *a):
        pass

    def set_callback(self, *a):
        pass

    def ignore_types(self, **k):
        pass


sys.modules.setdefault("rtmidi", types.SimpleNamespace(MidiOut=_FakeMidi, MidiIn=_FakeMidi))
import midi_passthrough  # noqa: E402

hub = midi_passthrough.MIDIHub()
if hub.pickup is None or hub.pickup.sock is None:      # port taken by a running hub: use a private one
    hub.pickup = HubPickup(port=0, start_thread=False)
ssp = _FakeMidi()
hub.outputs = {"SSP": ssp}
from_16n = hub.make_callback("16n:16n MIDI 1 20:0")
from_macros = hub.make_callback("RtMidiOut Client:Grid_Macros 130:0")
from_16n(([0xBF, 81, 30], 0), None)
check(ssp.sent[-1] == [0xBF, 81, 30], "hub: 16n passes normally")
hub.pickup.handle(json.dumps({"source": "16n", "pickup": [[16, 81, 90]]}).encode())
from_macros(([0xBF, 81, 90], 0), None)
check(ssp.sent[-1] == [0xBF, 81, 90], "hub: the macros' messages are forwarded")
from_16n(([0xBF, 81, 31], 0), None)
check(ssp.sent[-1] == [0xBF, 81, 90], "hub: 16n held back until it picks up")
from_16n(([0xBF, 81, 95], 0), None)
check(ssp.sent[-1] == [0xBF, 81, 95], "hub: 16n back once it passed the value")
hub.input_scanner.get_ports = lambda: ["RtMidiOut Client:Grid_Macros 130:0", "RtMidi output 131:0"]
hub.scan_and_update_inputs()
check(any("Grid_Macros" in n for n in hub.inputs) and not any("RtMidi output" in n for n in hub.inputs),
      "hub: subscribes to Grid_Macros, still skips other rtmidi outputs")
del sys.modules["rtmidi"]

# ---- the service end to end over pseudo-terminals ------------------------------------------------
gm, gs = pty.openpty()          # "grid": we write keys into gm, the service opens gs
tm, ts = pty.openpty()          # "Teletype": we play Teletype on tm, the service opens ts
for fd in (gm, tm):
    import tty  # noqa: E402
    tty.setraw(fd)
cfg_dir = os.path.join(tmpdir, "cfg")
svc_env = dict(os.environ, DSEM_CONFIG_DIR=cfg_dir, GRID_MACROS_PORT="47931")
proc = subprocess.Popen([sys.executable, os.path.join(SRC, "grid_macros.py"), "--no-midi", "--control-port", "47931",
                         "--grid", os.ttyname(gs), "--teletype", os.ttyname(ts)],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=svc_env)
TOOL = os.path.join(HERE, "..", "scripts", "grid_macros_tool.py")


def tool(*args, stdin=""):
    r = subprocess.run([sys.executable, TOOL, *args], input=stdin, capture_output=True, text=True, env=svc_env)
    return json.loads([ln for ln in r.stdout.splitlines() if ln.startswith("{")][-1])


def read_for(fd, seconds):
    buf = b""
    end = time.time() + seconds
    while time.time() < end:
        r = select.select([fd], [], [], 0.05)[0]
        if r:
            buf += os.read(fd, 4096)
    return buf


try:
    first = read_for(gm, 1.5)
    check(first[:1] == b"\x05" and bytes([0x1A, 0, 0]) in first, "service: queries the grid, draws the macro half")
    os.write(tm, bytes([0x05]))
    check(read_for(tm, 0.5) == bytes([0x03, 16, 8]), "service: answers Teletype's size query")
    os.write(gm, bytes([0x21, 12, 3, 0x20, 12, 3]))
    check(read_for(tm, 0.5) == bytes([0x21, 12, 3, 0x20, 12, 3]), "service: right-half keys reach Teletype")
    os.write(gm, bytes([0x21, 0, 5]))
    got = read_for(gm, 0.5)
    frame = got[got.rfind(bytes([0x1A, 0, 0])):][:35]
    check(len(frame) == 35 and mext.unpack_level_map(frame)[2][5 * 8] == 15,
          "service: bank key on the left half lights bank 6 on the grid")
    check(read_for(tm, 0.3) == b"", "service: left-half keys never reach Teletype")
    os.write(tm, mext.level_map(0, 0, [15] * 64) + mext.level_map(8, 0, [3] * 64))
    got = read_for(gm, 0.5)
    check(got == mext.level_map(8, 0, [3] * 64), "service: only Teletype's right quad reaches the grid")
    # Kontrolleur's tool over the same files and the service's control socket
    ex = tool("export")
    check(ex["ok"] and ex["exists"] and ex["service"] and ex["service"]["grid"], "tool: export with the service's state")
    data = ex["data"]
    data["names"][2][4] = "drop"
    data["banks"][2][4] = {"saved_at": 1, "state": {}}
    data["slew"][2][4] = 9
    r = tool("install", "--expect", "0000", stdin=json.dumps(data))
    check(not r["ok"] and r.get("conflict"), "tool: refuses when the file changed since it was loaded")
    r = tool("install", "--expect", ex["sha"], stdin=json.dumps(data))
    check(r["ok"] and r["data"]["names"][2][4] == "drop" and r["data"]["slew"][2][4] == 5,
          "tool: install cleans up (slew clamped) and keeps names")
    check(any(f.startswith("grid_macros-") for f in os.listdir(os.path.join(cfg_dir, "backups"))), "tool: backup kept")
    r = tool("audition", "3", "5")
    check(r["ok"] and r["bank"] == 2 and r["active"].get("2") == 4, "tool: audition loads the slot in the service")
    got = read_for(gm, 0.4)
    frame = got[got.rfind(bytes([0x1A, 0, 0])):][:35]
    check(len(frame) == 35 and mext.unpack_level_map(frame)[2][2 * 8] == 15, "tool: the grid switched to bank 3")
    check(not tool("audition", "1", "1")["ok"], "tool: auditioning an empty slot says so")
finally:
    proc.terminate()
    try:
        proc.wait(2)
    except subprocess.TimeoutExpired:
        proc.kill()
    log_out = proc.stdout.read().decode(errors="replace")
    if fails:
        print(log_out)

print(f"\n{'ALL PASSED' if not fails else f'{fails} FAILED'}")
sys.exit(1 if fails else 0)
