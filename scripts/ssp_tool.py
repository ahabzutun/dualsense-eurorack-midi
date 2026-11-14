#!/usr/bin/env python3
"""
Percussa SSP presets: which MIDI each preset listens to. Runs ON the SSP (python3 3.9):

    ssh root@ssp python3 - scan < ssp_tool.py
    python3 ssp_tool.py scan --root <folder with presets/>      (offline, on a copy)

Prints one JSON line:
  {"ok": true, "root": "/media/BOOT", "presets": [...], "current": {...}, "devices": [...]}

.pbp ("v5"): "synthor\\0", 8 bytes version, 16 bytes name, 1 byte, then records of little-endian
int32 [size, tag, payload]. Tag 220 starts a module: [slot, built-in type, plugin code]. Plugins keep
a JUCE state blob (tag 219) whose XML holds MIDI learn: <MIDI MIDI_IN=.. MIDI_CH=..><Automation>
<P_k paramId=k midi.channel=.. midi.num=.. midi.type=../></Automation>. Built-in type 13 is the
MIDI module: tag 207 = CH1-16 toggles, tag 206 = CCA-CCH.
"""
import glob
import json
import os
import re
import struct
import sys

ROOTS = ["/media/BOOT", "/media/linaro/SYNTHOR"]
MIDI_MODULE = 13
BUILTIN = {5: "SAMP", 13: "MIDI"}


def records(b):
    o = 33
    while o + 8 <= len(b):
        size, tag = struct.unpack_from("<ii", b, o)
        if size < 8 or o + size > len(b):
            return
        yield tag, b[o + 8:o + size]
        o += size


def _ints(p):
    return struct.unpack_from("<%di" % (len(p) // 4), p) if len(p) >= 4 else ()


def _code(v):
    return struct.pack(">i", v).decode("latin1") if v else ""


def _attr(s, name):
    m = re.search(r'(?:^|\s)%s="([^"]*)"' % re.escape(name), s)
    return m.group(1) if m else ""


def _int(s, default=0):
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return default


def parse(path):
    b = open(path, "rb").read()
    if b[:8] != b"synthor\0":
        return {"error": "not a preset"}
    name = b[16:32].split(b"\0")[0].decode("latin1").rstrip("_ ")
    mods, cur, names = [], None, {}
    for tag, p in records(b):
        if tag == 220:
            slot, kind, code = _ints(p)[:3]
            cur = {"slot": slot, "type": kind, "code": _code(code) or BUILTIN.get(kind, "#%d" % kind),
                   "plugin": bool(code), "midi_in": None, "midi_ch": 0, "inputs": [], "_ch": {}, "_cc": {}}
            mods.append(cur)
        elif cur is None:
            continue
        elif tag == 219:
            x = p[16:].decode("latin1", "replace")
            mi = re.search(r"<MIDI\b([^>]*?)(/>|>(.*?)</MIDI>)", x, re.S)
            if mi:
                head = mi.group(1)
                cur["midi_in"] = _attr(head, "MIDI_IN")
                cur["midi_ch"] = _int(_attr(head, "MIDI_CH"))
                for a in re.finditer(r"<P_\d+\b([^>]*)/>", mi.group(3) or ""):
                    t = _int(_attr(a.group(1), "midi.type"), -1)
                    cur["inputs"].append({"ch": _int(_attr(a.group(1), "midi.channel")),
                                          "kind": "cc" if t == 0 else ("note" if t == 1 else "t%d" % t),
                                          "num": _int(_attr(a.group(1), "midi.num")),
                                          "param": _int(_attr(a.group(1), "paramId")),
                                          "target": "param %d" % _int(_attr(a.group(1), "paramId"))})
            if cur["code"] == "MTIN":                    # MIDI → CV: 8 CCs on one channel (+ notes)
                ch = cur["midi_ch"]
                for k in "abcdefgh":
                    v = re.search(r'<PARAM id="cv_%s" value="([^"]*)"' % k, x)
                    if v and ch:
                        cur["inputs"].append({"ch": ch, "kind": "cc", "num": _int(v.group(1)), "target": "CV " + k.upper()})
                if ch and 'MIDI_NOTE_IN="1"' in x:
                    cur["inputs"].append({"ch": ch, "kind": "notes", "num": -1, "target": "pitch / gate"})
            elif cur["code"] == "MTMO" and cur["midi_ch"]:
                cur["inputs"].append({"ch": cur["midi_ch"], "kind": "notes", "num": -1, "target": "notes"})
            elif cur["code"] == "Grid":
                g = re.search(r"<GRID_STATE\b[^>]*>", x)
                g = g.group() if g else ""
                cur["midi_in"] = _attr(g, "midiDevice") or _attr(g, "midiDeviceId")
                for i, n in ((_attr(g, "midiDeviceId"), _attr(g, "midiDevice")),
                             (_attr(g, "globalsMidiDeviceId"), _attr(g, "globalsMidiDevice"))):
                    if i and n:
                        names.setdefault(i, n)          # Grid keeps id and name: names for the ids in other modules
                base = _int(_attr(g, "padMuteCCBase"), -1)
                if base >= 0:
                    cur["inputs"].append({"ch": 0, "kind": "cc", "num": base, "target": "pad mutes from CC %d" % base})
                if _attr(g, "progChangeEnabled") == "1":
                    cur["inputs"].append({"ch": 0, "kind": "pc", "num": -1, "target": "program change"})
        elif cur["type"] == MIDI_MODULE and tag == 207:
            i = _ints(p)
            cur["_ch"][i[1] + 1] = i[2]
        elif cur["type"] == MIDI_MODULE and tag == 206:
            i = _ints(p)
            cur["_cc"][i[1]] = i[2]
    out = []
    for m in mods:
        if m["type"] == MIDI_MODULE:
            chans = [c for c, on in sorted(m["_ch"].items()) if on] or list(range(1, 17))
            for c in chans:
                m["inputs"].append({"ch": c, "kind": "notes", "num": -1, "target": "pitch / gate / velocity"})
                for k, cc in sorted(m["_cc"].items()):
                    m["inputs"].append({"ch": c, "kind": "cc", "num": cc, "target": "CC" + "ABCDEFGH"[k]})
            m["channels"] = chans
        del m["_ch"], m["_cc"]
        out.append(m)
    return {"name": name, "modules": out, "device_names": names}


PRESET_LOG = "/tmp/synthor-presets.log"          # written by /root/synthor_log.py (see S90synthor)


def _logging_on():
    try:
        s = open("/etc/init.d/S90synthor").read()
        return "synthor_log.py" in s or "logger -t synthor" in s
    except OSError:
        return False


def current_from_log(names):
    """The last preset SYNTHOR loaded, from its output (needs S90synthor to keep it, see synthor_log.py)."""
    lines = []
    for f in ("/var/log/messages.0", "/var/log/messages", PRESET_LOG):
        try:
            lines += [l for l in open(f, errors="replace") if "loadPreset" in l]
        except OSError:
            pass
    for l in reversed(lines):
        m = re.search(r"(\d{3})\.pbp", l)
        if m:
            return {"file": m.group(1) + ".pbp", "line": l.strip()[-160:], "logging": True}
        m = re.search(r"preset name is \[([^\]]*)\]", l)
        if m:
            n = m.group(1).rstrip("_ ")
            hits = [f for f, nm in names.items() if nm == n]
            return {"file": hits[0] if len(hits) == 1 else None, "name": n, "line": l.strip()[-160:], "logging": True}
    return {"file": None, "logging": _logging_on()}


def midi_devices():
    """MIDI devices plugged into the SSP, as its modules name them (JUCE on ALSA: "client-port"):
    [{"id": "24-0", "name": "XIAO-SSP MIDI 1"}]. Only hardware (kernel clients of sound cards)."""
    out, client = [], None
    try:
        for line in open("/proc/asound/seq/clients", errors="replace"):
            m = re.match(r'Client\s+(\d+)\s*:\s*"([^"]*)"\s*\[(\w+)', line)
            if m:
                client = int(m.group(1)) if m.group(3) == "Kernel" and int(m.group(1)) >= 16 else None
                continue
            m = re.match(r'\s+Port\s+(\d+)\s*:\s*"([^"]*)"', line)
            if m and client is not None:
                out.append({"id": "%d-%s" % (client, m.group(1)), "name": m.group(2).strip()})
    except OSError:
        pass
    return out


def scan(root):
    presets, names = [], {}
    for f in sorted(glob.glob(os.path.join(root, "presets", "*.pbp"))):
        base = os.path.basename(f)
        try:
            p = parse(f)
        except Exception as e:
            p = {"error": "%s: %s" % (type(e).__name__, e)}
        p.update(file=base, mtime=int(os.path.getmtime(f)), size=os.path.getsize(f))
        names[base] = p.get("name")
        presets.append(p)
    known = {}
    for p in presets:
        for i, n in (p.get("device_names") or {}).items():
            known.setdefault(i, n)
    return {"ok": True, "root": root, "presets": presets, "current": current_from_log(names), "devices": midi_devices(),
            "device_names": known}


def main():
    args = sys.argv[1:]
    if "--root" in args:
        root = args[args.index("--root") + 1]
    else:
        root = next((r for r in ROOTS if os.path.isdir(os.path.join(r, "presets"))), None)
    try:
        if not args or args[0] != "scan":
            raise RuntimeError("usage: ssp_tool.py scan [--root DIR]")
        if not root:
            raise RuntimeError("no presets folder on this SSP")
        print(json.dumps(scan(root), separators=(",", ":")))
    except Exception as e:
        print(json.dumps({"ok": False, "error": "%s: %s" % (type(e).__name__, e)}))


if __name__ == "__main__":
    main()
