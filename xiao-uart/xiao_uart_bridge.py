#!/usr/bin/env python3
"""XIAO UART bridge: Pi hardware UART <-> remote XIAO (xiao-midi-bridge firmware) <-> USB-MIDI host.

Replaces the Pi-side XIAO of each host-to-host pair. For every link in links.json it creates
ALSA MIDI ports with the link's name (e.g. "XIAO-SSP"), so midi_passthrough.py finds them
exactly like the old USB XIAOs:
  - an ALSA *output* named <name>  (write to it -> bytes go out the UART -> remote XIAO -> USB host)
  - an ALSA *input*  named <name>  (MIDI from the USB host -> remote XIAO -> UART -> appears here)

Wire format = 4-byte USB-MIDI event packets (cable 0), same validation as the firmware;
invalid bytes are dropped one at a time so framing resyncs by itself.

Health: the firmware sends 1F A5 (80|state) (80|attempts) once a second (cable 1, never MIDI):
state bit 0 = its USB host configured it, bit 1 = host assigned an address, bit 2 = gave up
self-resetting. Together with the TX gate's power-on times this goes to
/run/xiao-uart/status.json for Kontrolleur (via er301_routes_tool export).
"""
import json, os, sys, threading, time
import serial, rtmidi

HERE = os.path.dirname(os.path.abspath(__file__))
BYTE_GAP_S = 0.001
STATUS_PATH = "/run/xiao-uart/status.json"
HB_MAGIC = 0xA5

def is_heartbeat(p):
    return p[0] == 0x1F and p[1] == HB_MAGIC and p[2] >= 0x80 and p[3] >= 0x80

def d7(b): return b < 0x80
def sx_head(b): return b == 0xF0 or d7(b)

def valid_packet(p):
    if p[0] >> 4: return False
    cin = p[0] & 0x0F
    if cin in (0x8, 0x9, 0xA, 0xB, 0xE): return (p[1] >> 4) == cin and d7(p[2]) and d7(p[3])
    if cin in (0xC, 0xD): return (p[1] >> 4) == cin and d7(p[2])
    if cin == 0x4: return sx_head(p[1]) and d7(p[2]) and d7(p[3])
    if cin == 0x5: return p[1] in (0xF7, 0xF6)
    if cin == 0x6: return sx_head(p[1]) and p[2] == 0xF7
    if cin == 0x7: return sx_head(p[1]) and d7(p[2]) and p[3] == 0xF7
    if cin == 0x2: return p[1] in (0xF1, 0xF3) and d7(p[2])
    if cin == 0x3: return p[1] == 0xF2 and d7(p[2]) and d7(p[3])
    if cin == 0xF: return p[1] in (0xF8, 0xFA, 0xFB, 0xFC, 0xFE)
    return False

PAYLOAD_LEN = {0x2: 2, 0x3: 3, 0x4: 3, 0x5: 1, 0x6: 2, 0x7: 3, 0x8: 3, 0x9: 3, 0xA: 3,
               0xB: 3, 0xC: 2, 0xD: 2, 0xE: 3, 0xF: 1}

def msg_to_packets(msg):
    """Full MIDI message (list of ints) -> list of 4-byte USB-MIDI packets."""
    if not msg: return []
    s = msg[0]
    if 0x80 <= s <= 0xEF:
        b = (msg + [0, 0])[:3]
        return [bytes([s >> 4, b[0], b[1], b[2]])]
    if s == 0xF0:                                   # SysEx, chunked in 3s
        out, i = [], 0
        while len(msg) - i > 3:
            out.append(bytes([0x4, msg[i], msg[i + 1], msg[i + 2]])); i += 3
        rest = msg[i:]
        cin = {1: 0x5, 2: 0x6, 3: 0x7}[len(rest)]
        out.append(bytes([cin] + rest + [0] * (3 - len(rest))))
        return out
    if s in (0xF1, 0xF3): return [bytes([0x2, s, msg[1] if len(msg) > 1 else 0, 0])]
    if s == 0xF2: return [bytes([0x3, s] + (msg[1:3] + [0, 0])[:2])]
    if s == 0xF6: return [bytes([0x5, s, 0, 0])]
    if s >= 0xF8: return [bytes([0xF, s, 0, 0])]
    return []

class Link:
    def __init__(self, dev, name, baud):
        self.dev, self.name, self.baud = dev, name, baud
        self.ser = None
        self.lock = threading.Lock()
        self.midi_out = rtmidi.MidiOut(rtmidi.API_LINUX_ALSA, name=name)
        self.midi_out.open_virtual_port(name)       # data from the UART appears here
        self.midi_in = rtmidi.MidiIn(rtmidi.API_LINUX_ALSA, name=name)
        self.midi_in.ignore_types(sysex=False, timing=False, active_sense=False)
        self.midi_in.open_virtual_port(name)        # others write here -> UART
        self.midi_in.set_callback(self.on_alsa)
        self.sysex = []
        self.hb = None                              # (monotonic time, state bits, attempts)
        self.gate = None

    def open_serial(self):
        while True:
            try:
                self.ser = serial.Serial(self.dev, self.baud, timeout=0.05)
                self.ser.reset_input_buffer()
                print(f"[{self.name}] {self.dev} @ {self.baud}", flush=True)
                return
            except Exception as e:
                print(f"[{self.name}] cannot open {self.dev}: {e}; retry in 2 s", flush=True)
                time.sleep(2)

    def on_alsa(self, event, _data=None):
        msg, _dt = event
        pk = b"".join(msg_to_packets(list(msg)))
        if not pk or self.ser is None: return
        try:
            with self.lock:
                self.ser.write(pk)
        except Exception as e:
            print(f"[{self.name}] write error: {e}", flush=True)

    def deliver(self, p):
        cin = p[0] & 0x0F
        data = list(p[1:1 + PAYLOAD_LEN[cin]])
        if cin in (0x4, 0x5, 0x6, 0x7) and (data[0] == 0xF0 or self.sysex):
            self.sysex += data
            if cin != 0x4 and self.sysex and self.sysex[-1] == 0xF7:
                self.midi_out.send_message(self.sysex); self.sysex = []
            elif len(self.sysex) > 65536:
                self.sysex = []
            return
        if cin == 0x4: return                      # stray SysEx continuation
        self.midi_out.send_message(data)

    def rx_loop(self):
        self.open_serial()
        buf, last = bytearray(), 0.0
        while True:
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                print(f"[{self.name}] read error: {e}; reopening", flush=True)
                time.sleep(1); self.open_serial(); buf.clear(); continue
            now = time.monotonic()
            if buf and chunk and now - last > BYTE_GAP_S * 20:   # stale partial packet
                buf.clear()
            if chunk: last = now
            for b in chunk:
                buf.append(b)
                if len(buf) < 4: continue
                if is_heartbeat(buf):
                    self.hb = (now, buf[2] & 0x7F, buf[3] & 0x7F); buf.clear(); continue
                if valid_packet(buf):
                    self.deliver(bytes(buf)); buf.clear()
                else:
                    del buf[0]

    def health(self, now):
        """host: ok | setup (address but no configuration yet) | none (no host talking) |
        gave_up | unknown (old firmware / no heartbeat); restarts: power-ons in the last 2 min."""
        powered = None if self.gate is None else bool(self.gate.enabled)
        restarts = 0 if self.gate is None else sum(1 for t in self.gate.power_ons if now - t < 120)
        host, attempts = "unknown", None
        if self.hb and now - self.hb[0] < 3:
            st, attempts = self.hb[1], self.hb[2]
            host = "ok" if st & 1 else "gave_up" if st & 4 else "setup" if st & 2 else "none"
        elif powered is False:
            host = "off"
        state = ("off" if powered is False else
                 "ok" if host == "ok" else
                 "stuck" if host == "gave_up" else
                 "flapping" if restarts >= 2 else
                 "waiting" if host in ("none", "setup") else
                 "ok" if restarts == 0 and powered else "unknown")
        return {"powered": powered, "host": host, "attempts": attempts,
                "restarts_2min": restarts, "state": state}


def status_loop(links):
    os.makedirs(os.path.dirname(STATUS_PATH), exist_ok=True)
    last = None
    while True:
        now = time.monotonic()
        data = {"t": time.time(), "links": {l.name: l.health(now) for l in links}}
        for name, h in data["links"].items():
            if last and last.get(name, {}).get("state") != h["state"]:
                icon = {"ok": "✅", "off": "⚪", "flapping": "🔁", "stuck": "🛑", "waiting": "⏳"}.get(h["state"], "❔")
                print(f"{icon} [{name}] USB host: {h['state']} (host={h['host']}, "
                      f"restarts in 2 min={h['restarts_2min']}, attempts={h['attempts']})", flush=True)
        last = data["links"]
        try:
            tmp = STATUS_PATH + ".tmp"
            with open(tmp, "w") as f: json.dump(data, f)
            os.replace(tmp, STATUS_PATH)
        except Exception as e:
            print(f"⚠️  status: {e}", flush=True)
        time.sleep(1)


def main():
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "links.json")
    cfg = json.load(open(cfg_path))
    links = [Link(l["device"], l["name"], int(l.get("baud", 250000))) for l in cfg["links"] if l.get("enabled", True)]
    try:
        from gpio_gate import start_gates          # no back-powering of unpowered XIAOs
        gates = start_gates([(l.dev, l.name) for l in links], lambda m: print(m, flush=True))
        by_name = {g.name: g for g in gates}
        for l in links: l.gate = by_name.get(l.name)
    except Exception as e:
        print(f"⚠️  TX gating unavailable: {e}", flush=True)
    for l in links:
        threading.Thread(target=l.rx_loop, daemon=True, name=l.name).start()
    threading.Thread(target=status_loop, args=(links,), daemon=True, name="status").start()
    print("bridge up:", ", ".join(f"{l.name}={l.dev}" for l in links), flush=True)
    while True: time.sleep(3600)

if __name__ == "__main__":
    main()
