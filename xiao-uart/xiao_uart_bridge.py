#!/usr/bin/env python3
"""XIAO UART bridge: Pi hardware UART <-> remote XIAO (xiao-midi-bridge firmware) <-> USB-MIDI host.

Replaces the Pi-side XIAO of each host-to-host pair. For every link in links.json it creates
ALSA MIDI ports with the link's name (e.g. "XIAO-SSP"), so midi_passthrough.py finds them
exactly like the old USB XIAOs:
  - an ALSA *output* named <name>  (write to it -> bytes go out the UART -> remote XIAO -> USB host)
  - an ALSA *input*  named <name>  (MIDI from the USB host -> remote XIAO -> UART -> appears here)

Wire format = 4-byte USB-MIDI event packets (cable 0), same validation as the firmware;
invalid bytes are dropped one at a time so framing resyncs by itself.
"""
import json, os, sys, threading, time
import serial, rtmidi

HERE = os.path.dirname(os.path.abspath(__file__))
BYTE_GAP_S = 0.001

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
                if valid_packet(buf):
                    self.deliver(bytes(buf)); buf.clear()
                else:
                    del buf[0]

def main():
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "links.json")
    cfg = json.load(open(cfg_path))
    links = [Link(l["device"], l["name"], int(l.get("baud", 250000))) for l in cfg["links"] if l.get("enabled", True)]
    try:
        from gpio_gate import start_gates          # no back-powering of unpowered XIAOs
        start_gates([(l.dev, l.name) for l in links], lambda m: print(m, flush=True))
    except Exception as e:
        print(f"⚠️  TX gating unavailable: {e}", flush=True)
    for l in links:
        threading.Thread(target=l.rx_loop, daemon=True, name=l.name).start()
    print("bridge up:", ", ".join(f"{l.name}={l.dev}" for l in links), flush=True)
    while True: time.sleep(3600)

if __name__ == "__main__":
    main()
