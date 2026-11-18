#!/usr/bin/env python3
"""
Helper for the DualSense Mapper app on the Mac (talks JSON over ssh).

    er301_routes_tool.py export            -> ER-301 routing, button functions and control names as JSON
    er301_routes_tool.py install           <- new er301_routes.py on stdin; checks it, keeps a backup,
                                              installs it and waits for the i2c2midi to confirm it
    er301_routes_tool.py install-controls  <- new controls.py on stdin; same, waits for the DualSense
                                              script to reload it
    er301_routes_tool.py shared-get NAME   -> a device dump kept on the Pi for every computer (NAME: ec4)
    er301_routes_tool.py shared-put NAME   <- the same, JSON on stdin
    … install / install-controls --expect SHA: only if the file on the Pi is still the one with that
      fingerprint (from export "sha"): nobody else changed it since it was loaded.

The live files are in ~/.config/dualsense-eurorack-midi (src/live_config.py).
"""
import glob
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
sys.path.insert(0, SRC)
import er301_config  # noqa: E402

try:
    import live_config  # noqa: E402
except Exception:        # older checkout
    live_config = None

try:
    import controls_config  # noqa: E402
except Exception:            # older checkout without button functions
    controls_config = None

ROUTES = er301_config.ROUTES_PATH
KEEP_BACKUPS = 10


def _entries(cfg, key, default_ch):
    out = []
    for e in getattr(cfg, key, []) or []:
        ch, ctl, port = (default_ch, *e) if len(e) == 2 else e
        out.append({"channel": ch, "control": ctl, "port": port})
    return out


# what this Pi understands, so Kontrolleur only writes what it can read:
# off_kept = PITCH / LFO may be dicts with enabled=False (off, settings kept)
# expect = install --expect SHA; status = the i2c2midi's answer (export "i2c2midi", install "i2c2midi")
# shared = shared-get / shared-put: device dumps both computers see (the EC4 can only be read on demand)
FEATURES = ["off_kept", "expect", "status", "shared"]
SHARED = ("ec4",)


def sha(path) -> str:
    """Fingerprint of a file ('' when it doesn't exist)."""
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:16]
    except OSError:
        return ""


def i2c2midi_status():
    return live_config.read_json(live_config.STATUS) if live_config is not None else None


def export():
    result = {"ok": True, "features": FEATURES, "sha": {"routes": sha(ROUTES)}, "i2c2midi": i2c2midi_status(),
              "paths": {"routes": ROUTES}}
    if controls_config is not None:
        result["sha"]["controls"] = sha(controls_config.CONTROLS_PATH)
        result["paths"]["controls"] = controls_config.CONTROLS_PATH
    try:
        maps = er301_config._load(er301_config.MAPPINGS_PATH, "mapper_mappings")
        result["cc_names"] = dict(getattr(maps, "CC_MAP", {}))
        result["nrpn_names"] = dict(getattr(maps, "NRPN_MAP", {}))
        result["long_press"] = float(getattr(maps, "LONG_PRESS_DURATION", 1.0))
        if controls_config is not None:
            try:
                c = controls_config.load()
                result["controls"] = {"buttons": c["buttons"], "chords": [list(x) for x in c["chords"]],
                                      "dpad_horizontal": c["dpad_horizontal"], "left_stick_x": c["left_stick_x"],
                                      "axes": c.get("axes"),
                                      "midi_inputs": [list(x) for x in c.get("midi_inputs", [])]}
            except controls_config.ControlsError as e:
                result["controls_error"] = str(e)
        cfg = er301_config._load(ROUTES, "mapper_routes")
        ch = getattr(cfg, "CHANNEL", 2)
        result.update({
            "channel": ch,
            "cv": _entries(cfg, "CV", ch),
            "nrpn_cv": _entries(cfg, "NRPN_CV", ch),
            "triggers": _entries(cfg, "TRIGGERS", ch),
            "note_triggers": _entries(cfg, "NOTE_TRIGGERS", ch),
            "pitch": getattr(cfg, "PITCH", None),
            "lfo": getattr(cfg, "LFO", None),
            "triggers_as_cv": bool(getattr(cfg, "TRIGGERS_AS_CV", True)),
        })
        er301_config.build()                     # report problems in the current file too
    except er301_config.ConfigError as e:
        result.update({"ok": False, "error": str(e)})
    except Exception as e:
        result.update({"ok": False, "error": f"{type(e).__name__}: {e}"})
    try:                                          # the MIDI bus's XIAO links (xiao-uart-bridge)
        with open("/home/ahab/xiao-uart/links.json") as f:
            result["links"] = [{"name": l["name"], "device": l.get("device"), "enabled": l.get("enabled", True)}
                               for l in json.load(f)["links"] if l.get("enabled", True)]
    except Exception:
        pass
    try:                                          # loglady, Teletype's own XIAO (loglady-teletype)
        with open("/run/loglady/status.json") as f:
            result["loglady"] = json.load(f)
    except Exception:
        pass
    print(json.dumps(result))


def _journal_line_since(unit, needle, stamp):
    try:
        out = subprocess.run(["journalctl", "-u", unit, "--since", stamp, "--no-pager", "-o", "cat"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    lines = [l for l in out.splitlines() if any(n in l for n in needle)]
    return lines[-1].strip() if lines else None


def _expected() -> str:
    a = sys.argv[2:]
    return a[a.index("--expect") + 1] if "--expect" in a and a.index("--expect") + 1 < len(a) else None


def _install(target, validate, unit, needle, wait_status=False):
    text = sys.stdin.read()
    want = _expected()
    if want is not None and sha(target) != want:
        print(json.dumps({"ok": False, "conflict": True, "sha": sha(target),
                          "error": f"{os.path.basename(target)} on the Pi changed since it was loaded "
                                   "(edited by hand or from another computer)"}))
        return
    new = target[:-3] + ".incoming.py"
    with open(new, "w") as f:
        f.write(text)
    try:
        summary = validate(new)
    except Exception as e:
        os.remove(new)
        print(json.dumps({"ok": False, "error": str(e)}))
        return
    backup = None
    if os.path.exists(target):
        backup = target + ".bak.app-" + time.strftime("%Y%m%d-%H%M%S")
        shutil.copy2(target, backup)             # a copy: the file itself is never missing, not even briefly
        for old in sorted(glob.glob(target + ".bak.app-*"))[:-KEEP_BACKUPS]:
            os.remove(old)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    os.replace(new, target)
    mt = os.stat(target).st_mtime
    line, status = None, None
    for _ in range(40):                          # the services check their file once per second; the
        time.sleep(0.2)                          # i2c2midi answers within ~3 s (or doesn't: rack off)
        if wait_status:
            st = i2c2midi_status()
            if st and st.get("file_mtime") == mt and st.get("state") not in ("waiting", None):
                status = st
                break
        line = _journal_line_since(unit, needle, stamp)
        if line and not wait_status:
            break
        if line and wait_status and i2c2midi_status() is None and _ > 25:
            break                                # a hub without the status file (not restarted yet)
    print(json.dumps({"ok": True, "summary": summary, "backup": os.path.basename(backup) if backup else None,
                      "hub": line, "i2c2midi": status, "sha": sha(target)}))


def install():
    def validate(path):
        try:
            return er301_config.build(routes_path=path)[1]
        except er301_config.ConfigError as e:
            raise Exception(str(e))
    _install(ROUTES, validate, "midi-passthrough", ["ER-301 routing"], wait_status=True)


def install_controls():
    if controls_config is None:
        print(json.dumps({"ok": False, "error": "this Pi has no controls_config.py yet"}))
        return
    def validate(path):
        try:
            c = controls_config.load(path)
        except controls_config.ControlsError as e:
            raise Exception(str(e))
        return f"{len(c['chords'])} chords, D-pad ↔ {c['dpad_horizontal']}, left stick X {c['left_stick_x']}"
    _install(controls_config.CONTROLS_PATH, validate, "dualsense-midi", ["Controls reloaded", "controls.py NOT applied"])


def shared_path(name):
    if name not in SHARED or live_config is None:
        return None
    return os.path.join(live_config.DIR, f"shared-{name}.json")


def shared_get(name):
    p = shared_path(name)
    if p is None:
        print(json.dumps({"ok": False, "error": f"unknown shared item {name!r}"}))
        return
    data = live_config.read_json(p)
    print(json.dumps({"ok": True, "data": data, "time": os.stat(p).st_mtime if data is not None else None}))


def shared_put(name):
    p = shared_path(name)
    if p is None:
        print(json.dumps({"ok": False, "error": f"unknown shared item {name!r}"}))
        return
    try:
        data = json.loads(sys.stdin.read())
    except ValueError as e:
        print(json.dumps({"ok": False, "error": f"not JSON: {e}"}))
        return
    live_config.write_json(p, data)
    print(json.dumps({"ok": True, "time": os.stat(p).st_mtime}))


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "export":
        export()
    elif cmd == "install":
        install()
    elif cmd == "install-controls":
        install_controls()
    elif cmd == "shared-get" and len(sys.argv) > 2:
        shared_get(sys.argv[2])
    elif cmd == "shared-put" and len(sys.argv) > 2:
        shared_put(sys.argv[2])
    else:
        print(__doc__)
        sys.exit(2)
