#!/bin/bash
# reset-doremidi.sh
#
# Run once at boot by doremidi-reset.service (after midi-passthrough.service).
# The CH345 chip gets urb status -32 (EPIPE) on first enumeration — only a
# full power cycle clears it. Hub 1-1 (2109:3431) supports per-port power
# switching on port 1 where DOREMiDi is connected.
#
# After the power cycle the passthrough hot-plug scanner picks up the fresh
# ALSA client automatically — no passthrough restart needed.

echo "reset-doremidi: powering off hub 1-1 port 1 for 10s" | systemd-cat -t reset-doremidi

uhubctl -l 1-1 -p 1 -a off 2>&1 | systemd-cat -t reset-doremidi
sleep 10
uhubctl -l 1-1 -p 1 -a on 2>&1 | systemd-cat -t reset-doremidi

# Poll for DOREMiDi ALSA client (up to 15s)
for i in $(seq 1 15); do
    sleep 1
    if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
        echo "reset-doremidi: ✅ done (${i}s after power-on)" | systemd-cat -t reset-doremidi
        exit 0
    fi
done

echo "reset-doremidi: ⚠️  MIDI port missing after 15s" | systemd-cat -t reset-doremidi
exit 1
