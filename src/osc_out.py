#!/usr/bin/env python3
"""
osc_out.py - OpenMW OSC tap for midi_passthrough (runs on the Pi).

Imported by midi_passthrough.py. Maps incoming MIDI (ANY channel, ANY CC/note)
to OpenMW OSC addresses and sends them to the Mac. Adds ZERO ALSA/MIDI clients
(UDP socket only) and never raises into the passthrough callback.

Mappings live in an editable text file (osc_map.conf, same folder), which is
HOT-RELOADED automatically within ~1s of being saved - NO service restart.
See osc_map.conf for the format.
"""

import os
import socket
import struct
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(_HERE, "osc_map.conf")

DEFAULT_HOST = "172.16.100.109"
DEFAULT_PORT = 9000


class Map:
    __slots__ = ("channel", "kind", "number", "address", "mode",
                 "invert", "in_lo", "in_hi", "in_dz", "_toggle", "_was_high")

    def __init__(self, channel, kind, number, address,
                 mode="cc", invert=False, in_lo=0, in_hi=127, in_dz=0, toggle=0.0):
        self._was_high = False
        self.channel = channel        # 1..16, or None for any
        self.kind = kind              # "cc" | "note"
        self.number = number
        self.address = address
        self.mode = mode              # cc | toggle | gate | velocity
        self.invert = invert
        self.in_lo = in_lo
        self.in_hi = in_hi
        self.in_dz = in_dz            # deadzone half-width (CC units) around centre
        self._toggle = toggle

    def key(self):
        return (self.channel, self.kind, self.number, self.address)


# ---- runtime state ----
_host = DEFAULT_HOST
_port = DEFAULT_PORT
_debug = False
_maps = []
_mtime = 0.0
_last_check = 0.0
_last_values = {}

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_sock.setblocking(False)
_addr = (_host, _port)


def _builtin_default():
    return [
        Map(16, "cc", 51, "/omw/fov"),
        Map(16, "cc", 52, "/omw/bloom"),
        Map(16, "cc", 53, "/omw/dist"),
        Map(16, "cc", 54, "/omw/timeofday"),
    ]


def _parse_line(line):
    s = line.split("#", 1)[0].strip()
    if not s:
        return None
    t = s.split()
    head = t[0].lower()
    if head == "host" and len(t) >= 2:
        return ("host", t[1])
    if head == "port" and len(t) >= 2:
        return ("port", int(t[1]))
    if head == "debug" and len(t) >= 2:
        return ("debug", t[1].lower() in ("on", "1", "true", "yes"))
    # mapping line:  CHANNEL TYPE NUMBER ADDRESS [MODE] [flags...]
    if len(t) < 4:
        return None
    ch = t[0].lower()
    channel = None if ch in ("*", "any", "all") else int(ch)
    kind = t[1].lower()
    if kind not in ("cc", "note"):
        return None
    number = int(t[2])
    address = t[3]
    mode, invert, in_lo, in_hi, in_dz = "cc", False, 0, 127, 0
    for e in (x.lower() for x in t[4:]):
        if e in ("cc", "toggle", "gate", "velocity", "dsstick"):
            mode = e
        elif e == "invert":
            invert = True
        elif e.startswith("lo="):
            in_lo = int(e[3:])
        elif e.startswith("hi="):
            in_hi = int(e[3:])
        elif e.startswith("dz="):
            in_dz = int(e[3:])
    return ("map", Map(channel, kind, number, address, mode, invert, in_lo, in_hi, in_dz))


def _load(preserve_toggles=True):
    global _host, _port, _debug, _maps, _addr
    try:
        with open(CONFIG_FILE) as f:
            lines = f.readlines()
    except FileNotFoundError:
        _maps = _builtin_default()
        return
    new_host, new_port, new_debug, new_maps = DEFAULT_HOST, DEFAULT_PORT, False, []
    for ln in lines:
        try:
            r = _parse_line(ln)
        except Exception:
            r = None
        if not r:
            continue
        if r[0] == "host":  new_host = r[1]
        elif r[0] == "port": new_port = r[1]
        elif r[0] == "debug": new_debug = r[1]
        elif r[0] == "map":  new_maps.append(r[1])
    if preserve_toggles and _maps:
        old = {m.key(): m._toggle for m in _maps}
        for m in new_maps:
            if m.key() in old:
                m._toggle = old[m.key()]
    _host, _port, _debug = new_host, new_port, new_debug
    _addr = (_host, _port)
    _maps = new_maps if new_maps else _builtin_default()
    sys.stderr.write("[osc_out] loaded %d mappings -> %s:%d (debug %s)\n"
                     % (len(_maps), _host, _port, "on" if _debug else "off"))
    sys.stderr.flush()


def _maybe_reload():
    global _mtime, _last_check
    now = time.monotonic()
    if now - _last_check < 1.0:        # check at most once per second
        return
    _last_check = now
    try:
        m = os.path.getmtime(CONFIG_FILE)
    except OSError:
        return
    if m != _mtime:
        _mtime = m
        _load()


# ---- OSC encoding ----
def _osc_string(s):
    b = s.encode("ascii") + b"\x00"
    return b + b"\x00" * ((4 - len(b) % 4) % 4)

def _osc_float(address, value):
    return _osc_string(address) + _osc_string(",f") + struct.pack(">f", value)

def _send(address, value):
    _last_values[address] = value
    try:
        _sock.sendto(_osc_float(address, value), _addr)
    except (BlockingIOError, OSError):
        pass

def _norm(value, lo, hi, invert):
    v = 0.0 if hi == lo else (value - lo) / (hi - lo)
    v = 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)
    return 1.0 - v if invert else v


def _norm_center(value, lo, hi, dz, invert):
    # bipolar axis with a dead centre: rest -> 0.5, extremes -> 0.0 / 1.0
    center = (lo + hi) / 2.0
    if abs(value - center) <= dz:
        out = 0.5
    elif value > center + dz:
        span = hi - (center + dz)
        out = 0.5 + 0.5 * (value - (center + dz)) / span if span > 0 else 1.0
    else:
        span = (center - dz) - lo
        out = 0.5 * (value - lo) / span if span > 0 else 0.0
    out = 0.0 if out < 0.0 else (1.0 if out > 1.0 else out)
    return 1.0 - out if invert else out


def _ds_axis(v, invert):
    # DualSense-firmware stick encoding -> centred 0..1 axis.
    # Their apply_deadzone emits: rest -> 0, one way -> 64..127, other way -> 63..0.
    if v == 0:
        out = 0.5                                   # rest (full-deflection also hits 0; accepted)
    elif v >= 64:
        out = 0.5 + 0.5 * (v - 64) / 63.0           # 64 -> 0.5, 127 -> 1.0
    else:
        out = 0.5 - 0.5 * (64 - v) / 63.0           # 63 -> ~0.5, 1 -> ~0.0
    out = 0.0 if out < 0.0 else (1.0 if out > 1.0 else out)
    return 1.0 - out if invert else out


def handle(midi_message):
    """Map one raw MIDI message (list of bytes) to OSC. Never raises."""
    try:
        _maybe_reload()
        if not midi_message:
            return
        status = midi_message[0]
        mtype = status & 0xF0
        channel = (status & 0x0F) + 1          # 1..16

        if mtype == 0xB0 and len(midi_message) >= 3:               # control change
            cc, val = midi_message[1], midi_message[2]
            if _debug:
                sys.stderr.write("[osc_out] ch%d cc %d = %d\n" % (channel, cc, val)); sys.stderr.flush()
            for m in _maps:
                if m.kind == "cc" and m.number == cc and (m.channel is None or m.channel == channel):
                    if m.mode == "toggle":
                        high = val >= 64
                        if high and not m._was_high:      # rising edge = a press
                            m._toggle = 1.0 - m._toggle
                            _send(m.address, m._toggle)
                        m._was_high = high
                    elif m.mode == "gate":
                        _send(m.address, 1.0 if val >= 64 else 0.0)
                    elif m.mode == "dsstick":             # DualSense eurorack-firmware stick
                        _send(m.address, _ds_axis(val, m.invert))
                    else:                                 # continuous knob/fader/stick
                        if m.in_dz > 0:
                            _send(m.address, _norm_center(val, m.in_lo, m.in_hi, m.in_dz, m.invert))
                        else:
                            _send(m.address, _norm(val, m.in_lo, m.in_hi, m.invert))

        elif mtype in (0x90, 0x80) and len(midi_message) >= 3:     # note on/off
            note, vel = midi_message[1], midi_message[2]
            on = (mtype == 0x90 and vel > 0)
            if _debug:
                sys.stderr.write("[osc_out] ch%d note %d %s vel%d\n" % (channel, note, "on" if on else "off", vel)); sys.stderr.flush()
            for m in _maps:
                if m.kind == "note" and m.number == note and (m.channel is None or m.channel == channel):
                    if m.mode == "velocity":
                        if on: _send(m.address, _norm(vel, m.in_lo, m.in_hi, m.invert))
                    elif m.mode == "toggle":
                        if on:
                            m._toggle = 1.0 - m._toggle
                            _send(m.address, m._toggle)
                    else:                                          # gate (and cc-on-note)
                        _send(m.address, 1.0 if on else 0.0)
    except Exception:
        pass


# ---- load at import ----
_load(preserve_toggles=False)
try:
    _mtime = os.path.getmtime(CONFIG_FILE)
except OSError:
    _mtime = 0.0


if __name__ == "__main__":
    print("config file:", CONFIG_FILE)
    print("target: %s:%d   debug: %s" % (_host, _port, _debug))
    for m in _maps:
        print("  ch%-3s %-4s %-3d -> %-18s [%s]" %
              (m.channel if m.channel else "*", m.kind, m.number, m.address, m.mode))


# ---- heartbeat: re-send latest value of every address every HEARTBEAT_S so a
# dropped one-shot packet (button release / a triggers final 0) self-heals fast
# instead of leaving a control stuck. UDP-only; adds no ALSA clients.
HEARTBEAT_S = 0.05
def _osc_bundle(items):
    out = _osc_string("#bundle") + struct.pack(">II", 0, 1)
    for _a, _v in items:
        msg = _osc_float(_a, _v)
        out += struct.pack(">i", len(msg)) + msg
    return out
def _heartbeat_loop():
    while True:
        time.sleep(HEARTBEAT_S)
        try:
            items = list(_last_values.items())
            if items:
                _sock.sendto(_osc_bundle(items), _addr)
        except (BlockingIOError, OSError):
            pass
        except Exception:
            pass
_hb_thread = threading.Thread(target=_heartbeat_loop, name="osc_heartbeat", daemon=True)
_hb_thread.start()
