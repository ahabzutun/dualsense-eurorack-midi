"""Which ALSA MIDI ports are XIAO links (the MIDI bus members that xiao-uart-bridge provides).

The XIAO links are interchangeable: whatever is plugged into mysteryman, jumpingman, nadine (SSP,
i2c2midi, a friend's device …) gets the MIDI flow of the bus and can send into it. loglady is not a MIDI
link (she is Teletype's grid + keyboard, see loglady-teletype.service) and never appears here.
The names come from ~/xiao-uart/links.json (enabled links), so renaming a link there is enough.
"""
import json
import os
import re
import time

LINKS_JSON = os.path.expanduser("~ahab/xiao-uart/links.json")
_cache = {"t": 0.0, "names": set()}


def link_names(max_age=2.0):
    """Names of the enabled XIAO links (= their ALSA client names). Cached for max_age seconds."""
    now = time.monotonic()
    if now - _cache["t"] > max_age:
        try:
            with open(LINKS_JSON) as f:
                _cache["names"] = {l["name"] for l in json.load(f)["links"] if l.get("enabled", True)}
        except Exception:
            pass                                    # keep the last known set
        _cache["t"] = now
    return _cache["names"]


def client_of(port_name):
    """'16n:16n Port 1 28:0' -> ('16n', 28). Unknown format -> (port_name, 999)."""
    m = re.match(r"^(.*?):.*\s(\d+):\d+$", port_name)
    return (m.group(1), int(m.group(2))) if m else (port_name, 999)


def is_link_port(port_name):
    return client_of(port_name)[0] in link_names()
