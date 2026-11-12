#!/usr/bin/env python3
"""
Read / write the configuration of a 16n faderbank over USB MIDI SysEx (as JSON).
Used by the DualSense Mapper app; runs on the Mac (16n plugged in there) or on the Pi.

    fader16_tool.py export     -> JSON: device, firmware, device options, 16 faders
    fader16_tool.py install    <- JSON on stdin (same shape); writes, reads back, verifies

Faders: usb/trs channel (1-16) and CC, usb/trs 14-bit flag, curve (0-127, 64 = linear;
curves need the curve firmware fork, otherwise "curves_supported" is false and they are skipped).
Writes are split into the 16n's partial messages (0D / 0C / 0B / 0A): a Teensy LC only
receives SysEx up to 60 bytes, so the full-config message would be cut off.
"""
import json
import sys
import time

HDR = [0xF0, 0x7D, 0x00, 0x00]


def _rtmidi():
    """python-rtmidi, or the alsa-midi stand-in where it can't be installed (e.g. Fedora 44)."""
    try:
        import rtmidi
    except ImportError:
        import rtmidi_compat as rtmidi
    return rtmidi


def _ports():
    rtmidi = _rtmidi()
    i, o = rtmidi.MidiIn(), rtmidi.MidiOut()
    ii = [n for n, p in enumerate(i.get_ports()) if "16n" in p]
    oi = [n for n, p in enumerate(o.get_ports()) if "16n" in p]
    if not ii or not oi:
        raise RuntimeError("no 16n found (is the faderbank plugged in?)")
    name = o.get_ports()[oi[0]]
    got = []
    i.ignore_types(sysex=False, timing=True, active_sense=True)
    i.open_port(ii[0])
    i.set_callback(lambda m, _d: got.append(m[0]))
    o.open_port(oi[0])
    return i, o, got, name


def _ask(o, got, msg, cmd, wait=1.2):
    got.clear()
    o.send_message(msg)
    t0 = time.time()
    while time.time() - t0 < wait:
        for m in list(got):
            if m[0] == 0xF0 and len(m) > 4 and m[4] == cmd:
                return m
        time.sleep(0.02)
    return None


def _flags(b):          # 3 bytes -> 16-bit mask (7 + 7 + 2 bits)
    return (b[0] & 0x7F) | ((b[1] & 0x7F) << 7) | ((b[2] & 0x03) << 14)


def _flag_bytes(mask):
    return [mask & 0x7F, (mask >> 7) & 0x7F, (mask >> 14) & 0x03]


def read_state(o, got):
    info = _ask(o, got, HDR + [0x1F, 0xF7], 0x0F, wait=2.0)
    if info is None:
        raise RuntimeError("the 16n did not answer the info request")
    cfg = list(info[9:-1])
    if len(cfg) < 80:
        raise RuntimeError(f"unexpected config length {len(cfg)}")
    hires = len(cfg) >= 86
    if not hires:
        cfg += [0] * 6
    cur = _ask(o, got, HDR + [0x2F, 0xF7], 0x09, wait=0.8)
    curves = list(cur[5:-1]) if cur else [64] * 16
    usb_hi, trs_hi = _flags(cfg[80:83]), _flags(cfg[83:86])
    faders = []
    for k in range(16):
        faders.append({
            "usb_channel": cfg[16 + k], "usb_cc": cfg[48 + k], "usb_14bit": bool((usb_hi >> k) & 1),
            "trs_channel": cfg[32 + k], "trs_cc": cfg[64 + k], "trs_14bit": bool((trs_hi >> k) & 1),
            "curve": curves[k] if k < len(curves) else 64,
        })
    return {
        "ok": True,
        "device": "16nLC" if info[5] == 3 else "16n",
        "firmware": ".".join(str(x) for x in info[6:9]),
        "hires_supported": hires,
        "curves_supported": cur is not None,
        "options": {
            "led_on": bool(cfg[0]), "led_flash": bool(cfg[1]), "rotate": bool(cfg[2]),
            "i2c_leader": bool(cfg[3]), "midi_thru": bool(cfg[8]),
            "fader_min": (cfg[5] << 7) + cfg[4], "fader_max": (cfg[7] << 7) + cfg[6],
        },
        "faders": faders,
        "_raw_flags": cfg[0:16],
    }


def _check(cond, msg):
    if not cond:
        raise ValueError(msg)


def install(want, o, got):
    before = read_state(o, got)
    faders = want["faders"]
    _check(len(faders) == 16, "need 16 faders")
    for k, f in enumerate(faders):
        for key in ("usb_channel", "trs_channel"):
            _check(1 <= int(f[key]) <= 16, f"fader {k + 1}: {key} must be 1-16")
        for key in ("usb_cc", "trs_cc"):
            _check(0 <= int(f[key]) <= 127, f"fader {k + 1}: {key} must be 0-127")
        _check(0 <= int(f.get("curve", 64)) <= 127, f"fader {k + 1}: curve must be 0-127")
        if f.get("usb_14bit"):
            _check(int(f["usb_cc"]) < 96, f"fader {k + 1}: 14-bit needs a CC below 96 (LSB = CC + 32)")
        if f.get("trs_14bit"):
            _check(int(f["trs_cc"]) < 96, f"fader {k + 1}: 14-bit needs a CC below 96 (LSB = CC + 32)")

    opts = want.get("options", {})
    flags = list(before["_raw_flags"])
    for idx, key in ((0, "led_on"), (1, "led_flash"), (2, "rotate"), (3, "i2c_leader"), (8, "midi_thru")):
        if key in opts:
            flags[idx] = 1 if opts[key] else 0
    for (lo, hi), key in (((4, 5), "fader_min"), ((6, 7), "fader_max")):
        if key in opts:
            v = int(opts[key])
            _check(0 <= v <= 16383, f"{key} must be 0-16383")
            flags[lo], flags[hi] = v & 0x7F, v >> 7

    usb_mask = sum(1 << k for k, f in enumerate(faders) if f.get("usb_14bit"))
    trs_mask = sum(1 << k for k, f in enumerate(faders) if f.get("trs_14bit"))
    usb = [int(f["usb_channel"]) for f in faders] + [int(f["usb_cc"]) for f in faders]
    trs = [int(f["trs_channel"]) for f in faders] + [int(f["trs_cc"]) for f in faders]
    msgs = [HDR + [0x0D] + flags + [0xF7]]
    if before["hires_supported"]:
        msgs += [HDR + [0x0C] + usb + _flag_bytes(usb_mask) + [0xF7],
                 HDR + [0x0B] + trs + _flag_bytes(trs_mask) + [0xF7]]
    else:
        msgs += [HDR + [0x0C] + usb + [0xF7], HDR + [0x0B] + trs + [0xF7]]
    if before["curves_supported"]:
        msgs.append(HDR + [0x0A] + [int(f.get("curve", 64)) for f in faders] + [0xF7])
    for m in msgs:
        assert len(m) <= 60
        o.send_message(m)
        time.sleep(0.35)

    after = read_state(o, got)
    keys = ["usb_channel", "usb_cc", "trs_channel", "trs_cc"]
    if before["hires_supported"]:
        keys += ["usb_14bit", "trs_14bit"]
    if before["curves_supported"]:
        keys += ["curve"]
    mismatch = [f"fader {k + 1} {key}" for k in range(16) for key in keys
                if after["faders"][k][key] != (bool(faders[k].get(key)) if key.endswith("14bit") else int(faders[k][key]))]
    after.pop("_raw_flags", None)
    return {"ok": not mismatch, "verified": not mismatch, "mismatch": mismatch[:8], "state": after,
            "skipped": [s for s, sup in (("14-bit", before["hires_supported"]), ("curves", before["curves_supported"])) if not sup]}


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    try:
        if cmd == "export":
            i, o, got, name = _ports()
            st = read_state(o, got)
            st.pop("_raw_flags", None)
            st["port"] = name
            print(json.dumps(st))
        elif cmd == "install":
            want = json.loads(sys.stdin.read())
            i, o, got, name = _ports()
            print(json.dumps(install(want, o, got)))
        else:
            print(__doc__)
            sys.exit(2)
    except Exception as e:
        print(json.dumps({"ok": False, "error": str(e)}))


if __name__ == "__main__":
    main()
