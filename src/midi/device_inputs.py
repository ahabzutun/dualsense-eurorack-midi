"""
MIDI from the 16n, the pedals (MPC-20 via DOREMiDi/CH345) and the EC4 -> Pi functions.

MIDI_INPUTS in config/controls.py says which incoming message drives which function
(see controls_config.py). The devices' MIDI keeps going to the i2c2midi through the MIDI hub
as before: the ALSA sequencer allows several readers on one port, so we only listen in.

- The pedal already has its own input in dualsense_midi.py (it records into the loop);
  its callback hands each message to feed("pedal", ...).
- The 16n and the EC4 get their own inputs here, opened only while MIDI_INPUTS uses them,
  with the same hot-plug rules as the pedal: one scanner client reused, one input per device
  in its own uniquely named ALSA client, opened by port NAME, re-opened when the
  subscription is gone, closed with cancel_callback / close_port / delete.
- MIDI callbacks run on rtmidi threads: they only queue. poll() runs the functions in the
  main loop, so the controller state is only ever touched from one thread.
"""
import os
import queue
import re
import threading

import controls_config

OWN_INPUTS = ("16n", "ec4")      # opened here; the pedal input lives in dualsense_midi.py
SCAN_INTERVAL = 2.0


def _close(midiin):
    if midiin is None:
        return
    for step in ("cancel_callback", "close_port", "delete"):
        try:
            getattr(midiin, step)()
        except Exception:
            pass


def _subscribed(client_name):
    """True if our ALSA client still has a live subscription (see dualsense_midi._pedal_input_alive)."""
    try:
        with open("/proc/asound/seq/clients") as f:
            txt = f.read()
    except OSError:
        return True
    for block in re.split(r"\n(?=Client\s+\d+ :)", txt):
        head = block.split("\n", 1)[0]
        if f'"{client_name}"' in head and "Connected From:" in block:
            return True
    return False


def parse(msg):
    """MIDI bytes -> (channel 1-16, 'cc' | 'note', number, value 0-127) or None."""
    if len(msg) < 3:
        return None
    status, a, b = msg[0], msg[1], msg[2]
    kind, ch = status & 0xF0, (status & 0x0F) + 1
    if kind == 0xB0:
        return ch, "cc", a, b
    if kind == 0x90:
        return ch, "note", a, 127 if b > 0 else 0
    if kind == 0x80:
        return ch, "note", a, 0
    return None


class Dispatcher:
    """Matches incoming messages against MIDI_INPUTS and runs the functions (main thread)."""

    def __init__(self, router=None, axes=None):
        self.router = router
        self.axes = axes
        self.level = {}           # (device, ch, kind, num) -> last value, for rising edges

    def entries(self):
        cfg = getattr(self.router, "cfg", None)
        return (cfg.get("midi_inputs") or []) if isinstance(cfg, dict) else []

    def devices(self):
        return {e[0] for e in self.entries()}

    def handle(self, device, msg):
        p = parse(msg)
        if p is None:
            return
        ch, kind, num, val = p
        key = (device, ch, kind, num)
        prev = self.level.get(key, 0)
        self.level[key] = val
        for dev, ech, ekind, enum, fn in self.entries():
            if (dev, ech, ekind, enum) != key:
                continue
            source = f"{device} ch{ch} {kind}{num}"
            if fn in controls_config.CONTINUOUS_FUNCTIONS:
                if self.axes is not None:
                    self.axes.apply(fn, val, source)
            elif prev < 64 <= val and hasattr(self.router, "run"):
                self.router.run(fn, source)


class DeviceInputs:
    def __init__(self, rtmidi, dispatcher=None):
        self.rtmidi = rtmidi
        self.dispatcher = dispatcher or Dispatcher()
        self.events = queue.Queue()
        self.inputs = {}          # device -> (MidiIn, port name, client name)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.scanner = None
        self.thread = None

    # ---- any thread --------------------------------------------------------------------

    def feed(self, device, msg):
        """Queue one message from `device` (called from MIDI callbacks)."""
        if self.dispatcher.entries():
            self.events.put((device, list(msg)))

    # ---- main thread -------------------------------------------------------------------

    def bind(self, router, axes):
        self.dispatcher.router = router
        self.dispatcher.axes = axes

    def poll(self):
        while True:
            try:
                device, msg = self.events.get_nowait()
            except queue.Empty:
                return
            try:
                self.dispatcher.handle(device, msg)
            except Exception as e:          # a broken function must never stop the controller
                print(f"⚠️  MIDI input {device}: {type(e).__name__}: {e}")

    # ---- hot-plug ----------------------------------------------------------------------

    def start(self):
        try:
            self.scanner = self.rtmidi.MidiIn()
        except Exception as e:
            print(f"⚠️  MIDI inputs disabled: {e}")
            return
        self.sync()
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()

    def _monitor(self):
        while not self.stop.wait(SCAN_INTERVAL):
            try:
                self.sync()
            except Exception as e:
                print(f"⚠️  MIDI input scan: {e}")

    def _find(self, device):
        pats = controls_config.INPUT_DEVICES[device]
        try:
            for n in self.scanner.get_ports():
                if any(p in n for p in pats) and "DualSense" not in n:
                    return n
        except Exception:
            pass
        return None

    def sync(self):
        """Open the 16n / EC4 inputs MIDI_INPUTS uses (if plugged in), close the rest."""
        wanted = self.dispatcher.devices()
        with self.lock:
            if self.stop.is_set():
                return
            for device in OWN_INPUTS:
                name = self._find(device) if device in wanted else None
                cur = self.inputs.get(device)
                if cur and cur[1] == name and name is not None and _subscribed(cur[2]):
                    continue
                if cur:
                    _close(cur[0])
                    del self.inputs[device]
                    print(f"🔌 {device} input closed")
                if name:
                    opened = self._open(device, name)
                    if opened:
                        self.inputs[device] = opened
                        print(f"🎚️  {device} input: {name}")

    def _open(self, device, name):
        client = f"DS {device} In {os.getpid()}"
        midiin = self.rtmidi.MidiIn(name=client)
        try:
            midiin.ignore_types(sysex=True, timing=True, active_sense=True)
            idx = next(i for i, n in enumerate(midiin.get_ports()) if n == name)
            midiin.open_port(idx)
            midiin.set_callback(lambda message, data=None, d=device: self.feed(d, message[0]))
            return midiin, name, client
        except Exception as e:
            print(f"⚠️  Could not open {device} port {name}: {e}")
            _close(midiin)
            return None

    def shutdown(self):
        self.stop.set()
        if self.thread and self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)
        with self.lock:
            for midiin, _, _ in self.inputs.values():
                _close(midiin)
            self.inputs.clear()
            _close(self.scanner)
            self.scanner = None
