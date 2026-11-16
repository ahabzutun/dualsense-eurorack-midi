"""
Where the live configuration lives: ~/.config/dualsense-eurorack-midi/ (controls.py, er301_routes.py and
the i2c2midi status), outside the git checkout, so `git pull` never touches or conflicts with it.

The files in src/config/ are the defaults of a fresh install. The first time a file is asked for, the copy
in src/config/ is moved over as it is (same text, same modification time: nothing reloads or re-sends
because of the move). Set DSEM_CONFIG_DIR to use another folder (tests).
"""
import json
import os
import shutil
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DIR = os.environ.get("DSEM_CONFIG_DIR") or os.path.join(os.path.expanduser("~"), ".config", "dualsense-eurorack-midi")
STATUS = os.path.join(DIR, "i2c2midi_status.json")


def path(name: str) -> str:
    live = os.path.join(DIR, name)
    if os.path.exists(live):
        return live
    src = os.path.join(HERE, "config", name)
    try:
        os.makedirs(DIR, exist_ok=True)
        if not os.path.exists(src):
            return live                          # nothing to move: the file will be created there
        tmp = f"{live}.moving-{os.getpid()}"
        shutil.copyfile(src, tmp)
        st = os.stat(src)
        os.utime(tmp, ns=(st.st_atime_ns, st.st_mtime_ns))     # same time: no reload just for the move
        os.chmod(tmp, 0o644)
        os.replace(tmp, live)
        return live
    except OSError:
        return src                               # home not writable: keep using the checkout


def write_json(p: str, data: dict):
    """Atomically (a reader never sees half a file)."""
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = f"{p}.tmp-{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(data, f)
    os.replace(tmp, p)


def read_json(p: str):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def now() -> float:
    return time.time()
