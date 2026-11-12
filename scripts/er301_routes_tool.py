#!/usr/bin/env python3
"""
Helper for the DualSense Mapper app on the Mac (talks JSON over ssh).

    er301_routes_tool.py export            -> ER-301 routing, button functions and control names as JSON
    er301_routes_tool.py install           <- new er301_routes.py on stdin; checks it, keeps a backup,
                                              installs it and waits for the MIDI hub to send it
    er301_routes_tool.py install-controls  <- new controls.py on stdin; same, waits for the DualSense
                                              script to reload it
"""
import glob
import json
import os
import subprocess
import sys
import time

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
sys.path.insert(0, SRC)
import er301_config  # noqa: E402

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


def export():
    result = {"ok": True}
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
                                      "axes": c.get("axes")}
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
    print(json.dumps(result))


def _journal_line_since(unit, needle, stamp):
    try:
        out = subprocess.run(["journalctl", "-u", unit, "--since", stamp, "--no-pager", "-o", "cat"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    lines = [l for l in out.splitlines() if any(n in l for n in needle)]
    return lines[-1].strip() if lines else None


def _install(target, validate, unit, needle):
    text = sys.stdin.read()
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
        os.replace(target, backup)
        for old in sorted(glob.glob(target + ".bak.app-*"))[:-KEEP_BACKUPS]:
            os.remove(old)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    os.replace(new, target)
    line = None
    for _ in range(25):                          # both services check their file once per second
        time.sleep(0.2)
        line = _journal_line_since(unit, needle, stamp)
        if line:
            break
    print(json.dumps({"ok": True, "summary": summary, "backup": os.path.basename(backup) if backup else None,
                      "hub": line}))


def install():
    def validate(path):
        try:
            return er301_config.build(routes_path=path)[1]
        except er301_config.ConfigError as e:
            raise Exception(str(e))
    _install(ROUTES, validate, "midi-passthrough", ["ER-301 routing"])


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


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "export":
        export()
    elif cmd == "install":
        install()
    elif cmd == "install-controls":
        install_controls()
    else:
        print(__doc__)
        sys.exit(2)
