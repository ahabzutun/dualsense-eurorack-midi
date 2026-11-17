"""
State providers: the parts of the rig a snapshot is made of. Each one can capture its state and restore it
(gliding what can glide, jumping the rest).

  midi       last value of every CC / NRPN that came in from the remembered controllers (16n, EC4 …);
             restored through the macros' own MIDI port, with pickup for faders and feedback for encoders
  dualsense  the DualSense script's own state: sample block, section, slice, freeze, motion on/off;
             asked for and set over a small local socket (state/snapshot_io.py in the DualSense script)
  teletype   what the right half of the grid showed (Teletype's LEDs): kept with the snapshot, nothing is
             restored yet (that needs a look at what Grid Control mode can do on the real hardware)
"""
import json
import socket


class Provider:
    name = ""

    def capture(self):
        """The current state (JSON-able), or None when there is nothing to remember right now."""
        return None

    def restore(self, data, glide, duration, now, jump_at):
        pass

    def describe(self, data) -> str:
        return ""


# ---- MIDI ----------------------------------------------------------------------------------------
def _match(port_name, names):
    for n in names:
        if n and n.lower() in port_name.lower():
            return n
    return None


class MidiState(Provider):
    """Remembers the latest value per (channel, CC) and per (channel, NRPN) from the configured inputs.

    out(msg)               sends a message through the macros' MIDI port (into the hub)
    feedback(source, msg)  sends a message back to a controller (EC4), optional
    arm_pickup(list)       tells the hub which (channel, cc, value) to hold back until picked up, optional
    """
    name = "midi"

    def __init__(self, out, settings, feedback=None, arm_pickup=None):
        self.out = out
        self.feedback = feedback
        self.arm_pickup = arm_pickup
        self.values = {}       # ("cc", ch, num) / ("nrpn", ch, param) -> value
        self.source = {}       # key -> source name it came from
        self._nrpn = {}        # (port, ch) -> [param_msb, param_lsb, value_msb]
        self.glide = None      # set by the engine
        self.pickup = None     # set by the engine (macros.pickup.Pickup)
        self.apply_settings(settings)

    def apply_settings(self, settings):
        self.sources = list(settings.get("midi_sources") or [])
        self.pickup_sources = list(settings.get("pickup_sources") or [])
        self.feedback_sources = list(settings.get("feedback_sources") or [])

    # -- incoming --
    def observe(self, port_name, msg):
        """A message arrived from an input. Returns the key it set (or None)."""
        src = _match(port_name, self.sources)
        if src is None or len(msg) < 3 or (msg[0] & 0xF0) != 0xB0:
            return None
        ch = (msg[0] & 0x0F) + 1
        num, val = msg[1], msg[2]
        st = self._nrpn.setdefault((port_name, ch), [None, None, None])
        if num == 99:
            st[0] = val
            return None
        if num == 98:
            st[1] = val
            return None
        if num in (6, 38) and st[0] is not None and st[1] is not None:
            param = (st[0] << 7) | st[1]
            key = ("nrpn", ch, param)
            if num == 6:
                st[2] = val
                value = val << 7
            else:
                value = ((st[2] or 0) << 7) | val
            return self._incoming(key, value, src)
        return self._incoming(("cc", ch, num), val, src)

    def _incoming(self, key, value, src):
        if self.pickup is not None and _match(src, self.pickup_sources) and key[0] == "cc":
            if not self.pickup.accept(key, value):
                return None                      # the hub holds this back too: the parameter stays put
        if self.glide is not None:
            self.glide.cancel(key)               # a hand took this value over
        self.values[key] = value
        self.source[key] = src
        return key

    # -- outgoing --
    def _messages(self, key, value):
        kind, ch, num = key
        st = 0xB0 | ((ch - 1) & 0x0F)
        if kind == "cc":
            return [[st, num, value & 0x7F]]
        return [[st, 99, (num >> 7) & 0x7F], [st, 98, num & 0x7F],
                [st, 6, (value >> 7) & 0x7F], [st, 38, value & 0x7F]]

    def send(self, key, value):
        self.values[key] = value
        msgs = self._messages(key, value)
        for m in msgs:
            self.out(m)
        src = self.source.get(key)
        if self.pickup is not None:
            self.pickup.update_target(key, value)
        if self.feedback is not None and src and _match(src, self.feedback_sources):
            for m in msgs:
                self.feedback(src, m)

    # -- snapshot --
    def capture(self):
        if not self.values:
            return None
        return {"values": [[k[0], k[1], k[2], v, self.source.get(k, "")]
                           for k, v in sorted(self.values.items())]}

    def restore(self, data, glide, duration, now, jump_at):
        arm = []
        for entry in (data or {}).get("values", []):
            try:
                kind, ch, num, value, src = entry
                key = (str(kind), int(ch), int(num))
                value = int(value)
            except (TypeError, ValueError):
                continue
            if src:
                self.source.setdefault(key, src)
            start = self.values.get(key)
            glide.glide(key, start, value, duration, now, lambda v, k=key: self.send(k, v))
            if (self.pickup is not None and kind == "cc" and start is not None and start != value
                    and _match(self.source.get(key, ""), self.pickup_sources)):
                self.pickup.arm(key, self.values.get(key, value))
                arm.append([key[1], key[2], self.values.get(key, value)])
        if arm and self.arm_pickup is not None:
            self.arm_pickup(arm)

    def describe(self, data):
        n = len((data or {}).get("values", []))
        return f"{n} MIDI value{'s' if n != 1 else ''}"


# ---- DualSense script ----------------------------------------------------------------------------
CONTROL_PORT = 47122          # UDP on 127.0.0.1, answered by the DualSense script


class DualSenseClient:
    """Talks to state/snapshot_io.py inside the DualSense script."""

    def __init__(self, host="127.0.0.1", port=CONTROL_PORT, timeout=0.25):
        self.addr = (host, port)
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(timeout)

    def request(self, payload):
        try:
            while True:                                  # drop late answers of earlier requests
                self.sock.setblocking(False)
                try:
                    self.sock.recv(65536)
                except (BlockingIOError, OSError):
                    break
            self.sock.settimeout(self.timeout)
            self.sock.sendto(json.dumps(payload).encode(), self.addr)
            data = self.sock.recv(65536)
            return json.loads(data.decode())
        except (OSError, ValueError):
            return None

    def send(self, payload):
        try:
            self.sock.sendto(json.dumps(payload).encode(), self.addr)
        except OSError:
            pass


class DualSenseState(Provider):
    name = "dualsense"

    def __init__(self, client):
        self.client = client

    def capture(self):
        r = self.client.request({"cmd": "capture"})
        if not r or not r.get("ok"):
            return None
        return r.get("state")

    def restore(self, data, glide, duration, now, jump_at):
        if not data:
            return
        glide.jump(("dualsense",), data, duration, now,
                   lambda d: self.client.send({"cmd": "apply", "state": d}), at=jump_at)

    def describe(self, data):
        if not data:
            return ""
        parts = [f"slice {data.get('slice', '?')}"]
        frozen = [k for k in ("left", "right") if (data.get("freeze") or {}).get(k)]
        if frozen:
            parts.append("frozen " + "+".join(frozen))
        return " · ".join(parts)


# ---- Teletype (right half of the grid) -----------------------------------------------------------
class TeletypeLeds(Provider):
    """Keeps what Teletype showed on the right half. Read-only for now."""
    name = "teletype"

    def __init__(self):
        self.levels = None     # 64 levels of the right quad, from the grid proxy

    def update(self, levels):
        self.levels = list(levels)

    def capture(self):
        if self.levels is None:
            return None
        return {"leds": self.levels}

    def describe(self, data):
        leds = (data or {}).get("leds") or []
        return f"{sum(1 for v in leds if v)} lit" if leds else ""
