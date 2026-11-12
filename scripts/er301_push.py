#!/usr/bin/env python3
"""
Send the ER-301 routing (src/config/er301_routes.py) to the i2c2midi and show its answer.

    python3 scripts/er301_push.py            send routing, wait for the answer
    python3 scripts/er301_push.py --query    only ask what the i2c2midi is using
    python3 scripts/er301_push.py --dry-run  only check the file and print the SysEx

Works while the MIDI hub is running (it opens its own ALSA client on the XIAO-TT port).
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
import er301_config  # noqa: E402


def find_port(ports):
    for i, name in enumerate(ports):
        if "XIAO" in name and "XIAO-SSP" not in name:
            return i, name
    return None, None


def main():
    args = sys.argv[1:]
    dry = "--dry-run" in args
    query_only = "--query" in args

    if not query_only:
        try:
            msgs, summary = er301_config.build()
        except er301_config.ConfigError as e:
            print(f"❌ er301_routes.py: {e}")
            return 2
        print(f"Routing: {summary}")
        if dry:
            for m in msgs:
                print("  " + " ".join(f"{b:02X}" for b in m))
            return 0
    else:
        msgs = [er301_config.query_message()]

    import rtmidi
    out, inp = rtmidi.MidiOut(), rtmidi.MidiIn()
    oi, oname = find_port(out.get_ports())
    ii, _ = find_port(inp.get_ports())
    if oi is None or ii is None:
        print("❌ No XIAO-TT MIDI port found (is the XIAO bridge to the i2c2midi plugged in?)")
        return 1
    replies = []
    inp.ignore_types(sysex=False, timing=True, active_sense=True)
    inp.open_port(ii)
    inp.set_callback(lambda m, _d: replies.append(m[0]))
    out.open_port(oi)

    er301_config.send(out, msgs)
    t0 = time.time()
    reply = None
    while time.time() - t0 < 2.0 and reply is None:
        for m in list(replies):
            reply = er301_config.parse_reply(m) or reply
        time.sleep(0.02)
    inp.cancel_callback()
    inp.close_port()
    out.close_port()

    if reply is None:
        print(f"⚠️  Sent to {oname}, but no answer from the i2c2midi within 2 s.")
        print("   (Rack off, old firmware, or the XIAO link from the Pi to the i2c2midi is down.)")
        return 1
    ok = reply["status"] == 0
    head = "✅ i2c2midi accepted the routing" if reply["kind"] == "commit" and ok else \
           ("ℹ️  i2c2midi status" if reply["kind"] == "query" else f"❌ i2c2midi rejected it: {reply['status_text']}")
    print(head)
    print(f"   in use: {reply['source']} — {reply['cv']} CV, {reply['nrpn']} NRPN, "
          f"{reply['triggers']} trigger, {reply['note_triggers']} note trigger routes")
    print(f"   ER-301 on its bus: {'yes' if reply['er301_present'] else 'NOT found'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
