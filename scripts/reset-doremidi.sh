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
    if [ "$age" -lt 60 ]; then
        exit 0
    fi
fi
touch "$LOCKFILE"

sleep 2

# Record the current ALSA client ID before power cycle (may be stale/absent)
OLD_CLIENT=$(aconnect -l 2>/dev/null | grep -i "doremidi" | grep -o "client [0-9]*" | grep -o "[0-9]*" | head -1)

echo "reset-doremidi: power-cycling hub 1-1 port 1 (old ALSA client: ${OLD_CLIENT:-none})" | systemd-cat -t reset-doremidi

# Power off for 5 seconds — CH345 needs time to fully discharge
uhubctl -l 1-1 -p 1 -a off 2>&1 | systemd-cat -t reset-doremidi
sleep 5
uhubctl -l 1-1 -p 1 -a on 2>&1 | systemd-cat -t reset-doremidi

# Wait for OLD client to disappear first (up to 5s)
if [ -n "$OLD_CLIENT" ]; then
    for i in $(seq 1 5); do
        sleep 1
        if ! aconnect -l 2>/dev/null | grep -q "client $OLD_CLIENT:"; then
            break
        fi
    done
fi

# Now poll for NEW working ALSA client (up to 15s)
FOUND=0
for i in $(seq 1 15); do
    sleep 1
    if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
        FOUND=1
        break
    fi
done

if [ "$FOUND" = "1" ]; then
    sleep 2
    echo "reset-doremidi: ✅ done (${i}s) — restarting midi-passthrough" | systemd-cat -t reset-doremidi
    systemctl restart midi-passthrough.service
else
    echo "reset-doremidi: ⚠️  MIDI port missing after 15s" | systemd-cat -t reset-doremidi
fi
