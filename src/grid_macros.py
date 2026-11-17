#!/usr/bin/env python3
"""
Grid macros service (grid-macros.service): the Pi between the monome grid and Teletype.

  real grid  ── USB (/dev/ttyACM*) ──▶ Pi ── USB gadget (/dev/ttyGS0) ──▶ Teletype
  left 8×8:  macro snapshots of the rig (macros/engine.py) — banks, save / load, slew
  right 8×8: passed through to Teletype (Grid Control mode or grid ops) — macros/proxy.py

MIDI: listens to the remembered controllers (16n, EC4 … — grid_macros.json "midi_sources"), sends restored
values through its own port Grid_Macros into the hub, sends values back to the EC4, and tells the hub
which 16n faders have to pick up first. The DualSense script's own state goes over a local socket.

Runs without a grid or without the gadget (whatever is missing is retried every 2 s).
    python3 src/grid_macros.py [--grid /dev/ttyACM0] [--teletype /dev/ttyGS0] [--no-midi]
"""
import argparse
import glob
import json
import os
import queue
import select
import socket
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_config  # noqa: E402
from macros.engine import MacroEngine  # noqa: E402
from macros.hub_pickup import PORT as HUB_PICKUP_PORT, MACRO_PORT_NAME  # noqa: E402
from macros.providers import DualSenseClient, DualSenseState, MidiState, TeletypeLeds  # noqa: E402
from macros.proxy import GridSplitter  # noqa: E402
from macros.store import MacroStore  # noqa: E402

STORE_FILE = "grid_macros.json"
RESCAN = 2.0
CONTROL_PORT = 47123          # UDP on 127.0.0.1: scripts/grid_macros_tool.py (Kontrolleur) asks / auditions


def log(*a):
    print(*a, flush=True)


# ---- serial ports --------------------------------------------------------------------------------
def open_raw(path):
    """Open a tty raw and non-blocking (no echo, no CR/LF or XON/XOFF handling: the protocol is binary)."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.ICRNL | termios.INLCR | termios.IGNCR)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except termios.error:
        pass                                   # pseudo-terminals in tests, fine either way
    return fd


def find_grid(explicit=None):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    for pattern in ("/dev/serial/by-id/*monome*", "/dev/serial/by-id/*grid*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


class Link:
    """One serial end that may come and go."""

    def __init__(self, name, finder):
        self.name = name
        self.finder = finder
        self.fd = None
        self.path = None
        self.next_try = 0.0

    def ensure(self, now):
        if self.fd is not None or now < self.next_try:
            return False
        self.next_try = now + RESCAN
        path = self.finder()
        if not path:
            return False
        try:
            self.fd = open_raw(path)
            self.path = path
            log(f"✅ {self.name}: {path}")
            return True
        except OSError as e:
            log(f"⚠️  {self.name}: {path}: {e}")
            return False

    def close(self, why=""):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            log(f"❌ {self.name} gone{(': ' + why) if why else ''}")
        self.fd = None
        self.next_try = time.monotonic() + RESCAN

    def write(self, data):
        if self.fd is None or not data:
            return
        try:
            view = memoryview(data)
            deadline = time.monotonic() + 0.05
            while view:
                try:
                    n = os.write(self.fd, view)
                    view = view[n:]
                except BlockingIOError:
                    if time.monotonic() > deadline:
                        return                 # the other side isn't reading: drop this frame
                    select.select([], [self.fd], [], 0.01)
        except OSError as e:
            self.close(str(e))

    def read(self):
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return b""
        except OSError as e:
            self.close(str(e))
            return b""
        if not data:                           # hang-up: reopen later
            self.close("hang-up")
        return data


# ---- MIDI ----------------------------------------------------------------------------------------
class Midi:
    """rtmidi plumbing: one virtual output into the hub, listeners on the remembered controllers,
    feedback outputs to controllers that keep their own value (EC4)."""

    def __init__(self, settings, incoming: queue.Queue):
        import rtmidi
        self.rtmidi = rtmidi
        self.incoming = incoming
        self.out = rtmidi.MidiOut()
        self.out.open_virtual_port(MACRO_PORT_NAME)
        self.in_scanner = rtmidi.MidiIn()      # created once, reused (ALSA client leaks otherwise)
        self.out_scanner = rtmidi.MidiOut()
        self.inputs = {}
        self.feedback_outs = {}
        self.settings = settings
        self.pickup_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log(f"✅ MIDI port {MACRO_PORT_NAME}")

    def send(self, msg):
        try:
            self.out.send_message(msg)
        except Exception as e:
            log(f"⚠️  MIDI out: {e}")

    def feedback(self, source, msg):
        out = self.feedback_outs.get(source)
        if out is not None:
            try:
                out.send_message(msg)
            except Exception:
                self.feedback_outs.pop(source, None)

    def arm_pickup(self, entries):
        for src in self.settings.get("pickup_sources") or []:
            payload = json.dumps({"source": src, "pickup": entries}).encode()
            try:
                self.pickup_sock.sendto(payload, ("127.0.0.1", HUB_PICKUP_PORT))
            except OSError:
                pass

    def scan(self):
        sources = [s.lower() for s in self.settings.get("midi_sources") or []]
        try:
            ports = self.in_scanner.get_ports()
        except Exception:
            ports = []
        wanted = {p: i for i, p in enumerate(ports)
                  if any(s in p.lower() for s in sources) and MACRO_PORT_NAME not in p}
        for name in list(self.inputs):
            if name not in wanted:
                mi = self.inputs.pop(name)
                try:
                    mi.cancel_callback()
                    mi.close_port()
                except Exception:
                    pass
                log(f"❌ listening no more: {name}")
        for name, idx in wanted.items():
            if name in self.inputs:
                continue
            try:
                mi = self.rtmidi.MidiIn()
                mi.open_port(idx)
                mi.set_callback(lambda m, d, n=name: self.incoming.put((n, m[0])))
                self.inputs[name] = mi
                log(f"👂 listening to {name}")
            except Exception as e:
                log(f"⚠️  {name}: {e}")
        # feedback outputs
        fb = list(self.settings.get("feedback_sources") or [])
        try:
            outs = self.out_scanner.get_ports()
        except Exception:
            outs = []
        for src in fb:
            port = next((i for i, p in enumerate(outs) if src.lower() in p.lower()), None)
            if port is None:
                self.feedback_outs.pop(src, None)
                continue
            if src not in self.feedback_outs:
                try:
                    mo = self.rtmidi.MidiOut()
                    mo.open_port(port)
                    self.feedback_outs[src] = mo
                    log(f"↩️  feedback to {outs[port]}")
                except Exception as e:
                    log(f"⚠️  feedback {src}: {e}")


class Control:
    """Local requests (JSON over UDP), answered from the main loop:
        {"cmd": "status"}                       → bank, slot loaded last, grid / Teletype connected
        {"cmd": "load", "bank": b, "slot": s}   → load a slot (the grid switches to that bank)"""

    def __init__(self, port=CONTROL_PORT):
        self.sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", port))
            s.setblocking(False)
            self.sock = s
        except OSError as e:
            log(f"⚠️  control socket not available ({e})")

    def poll(self, engine, store, grid, tt):
        if self.sock is None:
            return
        while True:
            try:
                data, addr = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError, OSError):
                return
            try:
                req = json.loads(data.decode())
                cmd = req.get("cmd")
                if cmd == "load":
                    store.reload_if_changed()            # a slot Kontrolleur just uploaded
                    ok = engine.load_at(int(req["bank"]), int(req["slot"]))
                    engine.force_redraw()
                    reply = {"ok": ok, "error": None if ok else "that slot is empty"}
                elif cmd == "status":
                    reply = {"ok": True}
                else:
                    reply = {"ok": False, "error": f"unknown command {cmd!r}"}
                reply.update(bank=engine.bank, active={str(b): s for b, s in engine.active.items()},
                             grid=grid.path if grid.fd is not None else None,
                             teletype=tt.path if tt.fd is not None else None)
            except Exception as e:
                reply = {"ok": False, "error": str(e)}
            try:
                self.sock.sendto(json.dumps(reply).encode(), addr)
            except OSError:
                pass


# ---- service -------------------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", help="grid serial device (default: /dev/serial/by-id/*monome*)")
    ap.add_argument("--teletype", default="/dev/ttyGS0", help="USB gadget serial towards Teletype")
    ap.add_argument("--store", default=None, help=f"macro file (default: ~/.config/…/{STORE_FILE})")
    ap.add_argument("--no-midi", action="store_true", help="no MIDI (testing the grid side)")
    ap.add_argument("--control-port", type=int, default=CONTROL_PORT, help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    store = MacroStore(args.store or os.path.join(live_config.DIR, STORE_FILE))
    if not os.path.exists(store.path):
        store.save()                           # so Kontrolleur finds a file with the defaults
    settings = store.settings

    incoming = queue.Queue()
    midi = None
    if not args.no_midi:
        try:
            midi = Midi(settings, incoming)
        except Exception as e:
            log(f"⚠️  MIDI not available ({e}); running without")

    tt_leds = TeletypeLeds()
    providers = {
        "dualsense": DualSenseState(DualSenseClient()),
        "teletype": tt_leds,
    }
    midi_state = MidiState(midi.send if midi else (lambda m: None), settings,
                           feedback=midi.feedback if midi else None,
                           arm_pickup=midi.arm_pickup if midi else None)
    providers["midi"] = midi_state
    engine = MacroEngine(store, providers, log=log)

    splitter = GridSplitter(on_macro_key=engine.key)
    splitter.on_teletype_leds = lambda qx, qy, levels: tt_leds.update(levels)

    grid = Link("grid", lambda: find_grid(args.grid))
    tt = Link("Teletype (gadget)", lambda: args.teletype if os.path.exists(args.teletype) else None)

    control = Control(args.control_port)
    tick = 1.0 / max(10, int(settings.get("tick_hz", 50)))
    next_scan = 0.0
    log("🟩 grid macros running")
    try:
        while True:
            now = time.monotonic()
            if grid.ensure(now):
                grid.write(bytes((0x05,)))                       # size query, answer only logged
                engine.force_redraw()
                grid.write(splitter.resend_teletype_half())
            tt.ensure(now)
            if now >= next_scan:
                next_scan = now + RESCAN
                if midi:
                    midi.scan()
                if store.reload_if_changed():
                    midi_state.apply_settings(store.settings)
                    engine.force_redraw()
                    log("🔄 grid_macros.json reloaded")

            fds = [l.fd for l in (grid, tt) if l.fd is not None]
            r = select.select(fds, [], [], tick)[0] if fds else (time.sleep(tick) or [])
            if grid.fd is not None and grid.fd in r:
                data = grid.read()
                if data:
                    tt.write(splitter.from_grid(data))
            if tt.fd is not None and tt.fd in r:
                data = tt.read()
                if data:
                    answer, to_grid = splitter.from_teletype(data)
                    tt.write(answer)
                    grid.write(to_grid)

            control.poll(engine, store, grid, tt)
            while True:
                try:
                    port, msg = incoming.get_nowait()
                except queue.Empty:
                    break
                midi_state.observe(port, msg)

            leds = engine.tick()
            if leds is not None:
                grid.write(splitter.macro_frame(leds))
    except KeyboardInterrupt:
        log("👋 grid macros stopped")


if __name__ == "__main__":
    main()
