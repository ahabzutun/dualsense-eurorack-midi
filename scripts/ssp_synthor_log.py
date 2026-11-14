#!/usr/bin/env python3
"""
Runs SYNTHOR's start script in a pseudo-terminal, so SYNTHOR writes its output line by line
(into a pipe it would buffer it), and keeps the lines about presets in /tmp/synthor-presets.log.
Kontrolleur (scripts/ssp_tool.py) reads that file to know which preset is loaded.

Installed on the SSP as /root/synthor_log.py; /etc/init.d/S90synthor starts it instead of
/root/synthor.sh (falls back to synthor.sh when python3 or this file is missing).
"""
import os
import pty
import sys

LOG = "/tmp/synthor-presets.log"
MAX = 64 * 1024
CMD = sys.argv[1:] or ["/root/synthor.sh"]
pending = b""


def keep(lines):
    try:
        with open(LOG, "ab") as f:
            f.write(b"".join(l.rstrip(b"\r") + b"\n" for l in lines))
        if os.path.getsize(LOG) > MAX:                  # stay small: keep the newer half
            with open(LOG, "rb") as f:
                data = f.read()[-MAX // 2:]
            with open(LOG, "wb") as f:
                f.write(data[data.find(b"\n") + 1:])
    except OSError:
        pass


def read(fd):
    global pending
    data = os.read(fd, 4096)
    try:
        pending += data
        *lines, pending = pending.split(b"\n")
        pending = pending[-4096:]
        hits = [l for l in lines if b"preset" in l.lower()]
        if hits:
            keep(hits)
    except Exception:
        pass
    return data


pty.spawn(CMD, read)
