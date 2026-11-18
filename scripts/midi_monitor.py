#!/usr/bin/env python3
"""Stream what arrives on one MIDI input as JSON lines, for Kontrolleur's live Teletype view.

    python3 midi_monitor.py [--port bus|NAME] [--seconds N]

--port bus (default; "XIAO-TT" means the same, for older callers) listens on every XIAO link at once —
the links are interchangeable, so the i2c2midi may be on any of them. Any other NAME: the first port
containing it.

Opens its own ALSA subscription to the port (the MIDI hub keeps its own, nothing is taken away from it)
and prints one line per message:
    {"m": [status, data1, data2], "t": seconds since start}
plus a heartbeat every second, so the reader knows the link is alive and whether the port is there:
    {"hb": true, "port": "XIAO-TT:XIAO-TT MIDI 1 28:0" | null, "t": …}
The port may come and go (bridge unplugged, i2c2midi off): it is looked for again every second.
Exits when stdout closes (ssh ended: the next heartbeat fails), on Ctrl+C, or after --seconds.
SysEx is left out (the ER-301 routing answers are long and of no use here)."""
from __future__ import annotations

import argparse
import json
import sys
import time


def _rtmidi():
    try:
        import rtmidi
        return rtmidi
    except ImportError:
        sys.path.insert(0, __file__.rsplit("/", 1)[0])
        import rtmidi_compat as rtmidi
        return rtmidi


def find(ports, name):
    return next((i for i, p in enumerate(ports) if name.lower() in p.lower()), None)


BUS_ALIASES = ("bus", "links", "xiao-tt")


def find_all(ports, name):
    """{port name: index} of the ports to listen on."""
    if name.lower() in BUS_ALIASES:
        sys.path.insert(0, __file__.rsplit("/", 2)[0] + "/src")
        from xiao_links import is_link_port
        return {p: i for i, p in enumerate(ports) if is_link_port(p)}
    i = find(ports, name)
    return {} if i is None else {ports[i]: i}


def emit(obj) -> bool:
    try:
        sys.stdout.write(json.dumps(obj, separators=(",", ":")) + "\n")
        sys.stdout.flush()
        return True
    except (BrokenPipeError, OSError):
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="bus")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until stopped)")
    a = ap.parse_args()
    rtmidi = _rtmidi()
    t0 = time.monotonic()
    inps = {}                                 # port name -> MidiIn
    last_hb = 0.0
    pending = []

    def cb(event, _data=None):
        msg, _dt = event
        if msg and msg[0] != 0xF0:
            pending.append((list(msg), round(time.monotonic() - t0, 3)))

    try:
        while True:
            now = time.monotonic()
            if a.seconds and now - t0 > a.seconds:
                break
            if now - last_hb >= 1.0:
                last_hb = now
                probe = rtmidi.MidiIn()
                want = find_all(probe.get_ports(), a.port)
                del probe
                for p in [p for p in inps if p not in want]:        # disappeared or renumbered
                    try:
                        inps.pop(p).close_port()
                    except Exception:
                        pass
                for p, idx in want.items():
                    if p not in inps:
                        i = rtmidi.MidiIn()
                        i.ignore_types(sysex=True, timing=True, active_sense=True)
                        i.open_port(idx)
                        i.set_callback(cb)
                        inps[p] = i
                name = " + ".join(sorted(inps)) or None
                if not emit({"hb": True, "port": name, "t": round(now - t0, 3)}):
                    break
            while pending:
                m, t = pending.pop(0)
                if not emit({"m": m, "t": t}):
                    return
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        for i in inps.values():
            try:
                i.close_port()
            except Exception:
                pass


if __name__ == "__main__":
    main()
