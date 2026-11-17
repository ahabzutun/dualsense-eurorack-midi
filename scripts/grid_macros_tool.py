#!/usr/bin/env python3
"""
Helper for Kontrolleur's Macros tab (talks JSON over ssh, like er301_routes_tool.py).

    grid_macros_tool.py export                  -> grid_macros.json (cleaned up), its fingerprint, the service's state
    grid_macros_tool.py install [--expect SHA]  <- a new grid_macros.json on stdin; cleaned up, a backup kept,
                                                   written; with --expect only if the file is still the one with
                                                   that fingerprint (nobody else changed it since it was loaded)
    grid_macros_tool.py audition BANK SLOT      -> the running service loads that slot (1-based), as from the grid
    grid_macros_tool.py status                  -> the running service's state only

The file lives in ~/.config/dualsense-eurorack-midi/ (src/live_config.py); the service re-reads it when it changes.
Every command prints one JSON object as its last line.
"""
import hashlib
import json
import os
import shutil
import socket
import sys
import time

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
sys.path.insert(0, SRC)
import live_config  # noqa: E402
from macros.store import MacroStore  # noqa: E402

PATH = os.path.join(live_config.DIR, "grid_macros.json")
CONTROL_PORT = 47123
KEEP_BACKUPS = 10


def sha(path) -> str:
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()[:16]
    except OSError:
        return ""


def ask_service(payload, timeout=1.0):
    """The running grid-macros service's answer, or None when it isn't running."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(payload).encode(), ("127.0.0.1", int(os.environ.get("GRID_MACROS_PORT", CONTROL_PORT))))
        return json.loads(s.recv(65536).decode())
    except (OSError, ValueError):
        return None
    finally:
        s.close()


def out(obj):
    print(json.dumps(obj))


def export():
    exists = os.path.exists(PATH)
    store = MacroStore(PATH if exists else None)
    out({"ok": True, "path": PATH, "exists": exists, "sha": sha(PATH), "data": store.data,
         "service": ask_service({"cmd": "status"})})


def install(expect=None):
    try:
        raw = json.loads(sys.stdin.read())
    except ValueError as e:
        out({"ok": False, "error": f"not JSON: {e}"})
        return
    if expect is not None and sha(PATH) != expect:
        out({"ok": False, "conflict": True, "sha": sha(PATH),
             "error": "grid_macros.json on the Pi changed since it was loaded (saved from the grid?)"})
        return
    data = MacroStore.normalise(raw)
    if os.path.exists(PATH):
        bdir = os.path.join(live_config.DIR, "backups")
        os.makedirs(bdir, exist_ok=True)
        shutil.copy2(PATH, os.path.join(bdir, time.strftime("grid_macros-%Y%m%d-%H%M%S.json")))
        old = sorted(f for f in os.listdir(bdir) if f.startswith("grid_macros-"))
        for f in old[:-KEEP_BACKUPS]:
            try:
                os.remove(os.path.join(bdir, f))
            except OSError:
                pass
    store = MacroStore()
    store.path = PATH
    store.data = data
    store.save()
    out({"ok": True, "sha": sha(PATH), "data": data, "service": ask_service({"cmd": "status"})})


def audition(bank, slot):
    r = ask_service({"cmd": "load", "bank": bank - 1, "slot": slot - 1}, timeout=2.0)
    if r is None:
        out({"ok": False, "error": "the grid-macros service isn't running on the Pi"})
    else:
        out(r)


if __name__ == "__main__":
    a = sys.argv[1:]
    cmd = a[0] if a else ""
    if cmd == "export":
        export()
    elif cmd == "install":
        install(a[a.index("--expect") + 1] if "--expect" in a and a.index("--expect") + 1 < len(a) else None)
    elif cmd == "audition" and len(a) == 3:
        audition(int(a[1]), int(a[2]))
    elif cmd == "status":
        r = ask_service({"cmd": "status"})
        out(r if r is not None else {"ok": False, "error": "the grid-macros service isn't running on the Pi"})
    else:
        print(__doc__)
        sys.exit(2)
