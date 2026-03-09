#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The CH345 chip gets urb status -32 (EPIPE) on first enumeration — only a
# full power cycle clears it. Hub 1-1 (2109:3431) supports per-port power
# switching, so uhubctl can cut and restore 5V to port 1 in software.
#
# LOCK: The power-off/on triggers another udev ADD event. Lock prevents re-entry.

LOCKFILE="/tmp/reset-doremidi.lock"
if [ -f "$LOCKFILE" ]; then
    age=$(( $(date +%s) - $(stat -c %Y "$LOCKFILE" 2>/dev/null || echo 0) ))
    if [ "$age" -lt 30 ]; then
        exit 0
    fi
fi
touch "$LOCKFILE"

sleep 2

echo "reset-doremidi: power-cycling hub 1-1 port 1 via uhubctl" | systemd-cat -t reset-doremidi
uhubctl -l 1-1 -p 1 -a cycle -d 2 2>&1 | systemd-cat -t reset-doremidi

# Poll aconnect until DOREMiDi ALSA port appears (up to 15 seconds)
FOUND=0
for i in $(seq 1 15); do
    sleep 1
    if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
        FOUND=1
        break
    fi
done

if [ "$FOUND" = "1" ]; then
    sleep 2  # let ALSA client stabilize before passthrough rescans
    echo "reset-doremidi: ✅ done (${i}s) — restarting midi-passthrough" | systemd-cat -t reset-doremidi
    systemctl restart midi-passthrough.service
else
    echo "reset-doremidi: ⚠️  MIDI port missing after 15s" | systemd-cat -t reset-doremidi
fi
