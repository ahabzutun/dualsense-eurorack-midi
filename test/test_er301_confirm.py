#!/usr/bin/env python3
"""The MIDI hub sends the ER-301 routing to the i2c2midi and makes sure it arrived (midi_passthrough._er301_tick),
against a simulated i2c2midi and clock. Run: python3 test/test_er301_confirm.py"""
import os
import sys
import tempfile
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
tmp = tempfile.mkdtemp()
os.environ["DSEM_CONFIG_DIR"] = tmp                 # live files + status in a scratch folder
sys.modules.setdefault("rtmidi", types.SimpleNamespace(MidiIn=object, MidiOut=object))

import er301_config        # noqa: E402
import live_config         # noqa: E402
import midi_passthrough as MP  # noqa: E402
MP.HUB = MP.MIDIHub

ROUTES = live_config.path("er301_routes.py")
with open(ROUTES, "w") as f:
    f.write("CHANNEL = 2\nCV = [(2, 20, 4), (2, 21, 5)]\nTRIGGERS = [(2, 22, 6)]\nPITCH = None\nLFO = None\n")
now = [1000.0]
MP.time = types.SimpleNamespace(time=lambda: now[0], sleep=lambda s: None)


class FakeI2c2midi:
    """Answers like the firmware: commit → (status, counts), query → what it has."""
    def __init__(self, hub):
        self.hub, self.on, self.has, self.source, self.reject = hub, True, None, 1, False
        self.sent = 0

    def send_message(self, msg):
        self.sent += 1
        if not self.on:
            return
        if msg == er301_config.query_message():
            self.reply(2, 0)
        elif list(msg[:5]) == er301_config.HDR + [0x7F]:
            if self.reject:
                return self.reply(1, 3)
            self.has, self.source = dict(self.pending), 2
            self.reply(1, 0)
        elif list(msg[:5]) == er301_config.HDR + [0x01]:
            self.pending = {k: 0 for k in er301_config.RECORD_KIND.values()}
        elif len(msg) > 5 and msg[4] in er301_config.RECORD_KIND:
            self.pending[er301_config.RECORD_KIND[msg[4]]] += 1

    def reply(self, kind, status):
        c = self.has or {k: 0 for k in er301_config.RECORD_KIND.values()}
        r = er301_config.HDR + [0x7E, kind, status, self.source, c["cv"], c["nrpn"], c["triggers"], c["note_triggers"], 1, 0xF7]
        self.hub._er301_reply = (now[0], er301_config.parse_reply(r))


def hub():
    h = MP.HUB.__new__(MP.HUB)
    h.lock = threading.Lock()
    h.outputs = {}
    # (the real __init__ opens MIDI ports: set only what _er301_tick needs)
    h.ER301_ANSWER_WITHIN, h.ER301_RETRY, h.ER301_CHECK_EVERY = 3.0, (5, 10, 20, 40, 60), 60.0
    h._er301_mtime, h._er301_push_pending, h._er301_reply = None, False, None
    h._er301 = {"state": "unknown", "expected": None, "summary": "", "sent_at": 0.0, "kind": None, "tries": 0,
                "next": 0.0, "file_mtime": None, "reply": None, "checked": 0.0}
    dev = FakeI2c2midi(h)
    h.outputs["Teletype"] = dev
    return h, dev


def run(h, seconds):
    for _ in range(int(seconds)):
        h._er301_tick()
        now[0] += 1.0
    h._er301_tick()


def status():
    return live_config.read_json(live_config.STATUS)["state"]


errors = []


def check(what, got, want):
    if got != want:
        errors.append(f"{what}: {got!r}, expected {want!r}")
    print(("✅" if got == want else "❌"), what, "→", got)


h, dev = hub()
run(h, 2)
check("rack on: confirmed", status(), "confirmed")
check("rack on: i2c2midi has the routing", dev.has, {"cv": 2, "nrpn": 0, "triggers": 1, "note_triggers": 0})

h, dev = hub()
dev.on = False
run(h, 5)
check("rack off: no answer", status(), "no answer")
n = dev.sent
run(h, 30)
check("rack off: sent again meanwhile", dev.sent > n, True)
dev.on = True
run(h, 70)
check("rack switched on later: confirmed", status(), "confirmed")

h, dev = hub()
dev.reject = True
run(h, 2)
check("invalid routing: rejected", status(), "rejected")
n = dev.sent
run(h, 120)
check("rejected: not sent over and over", dev.sent - n <= 4, True)   # only the minute queries

h, dev = hub()
run(h, 2)
dev.has, dev.source = None, 0                        # power cycle with an empty EEPROM: firmware defaults
run(h, 65)
check("rack lost the routing: sent again and confirmed", (status(), dev.has["cv"] if dev.has else None), ("confirmed", 2))

h, dev = hub()
run(h, 2)
with open(ROUTES, "w") as f:
    f.write("CHANNEL = 2\nCV = [(2, 20, 4)]\nTRIGGERS = []\nPITCH = None\nLFO = None\n")
os.utime(ROUTES, (now[0] + 5, now[0] + 5))
run(h, 3)
check("file changed: new routing confirmed", (status(), dev.has["cv"], dev.has["triggers"]), ("confirmed", 1, 0))

print("ERRORS:", len(errors))
sys.exit(1 if errors else 0)
