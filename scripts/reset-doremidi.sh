#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# On first boot the device gets urb status -32 (EPIPE) and never creates a
# MIDI port. A USB unbind/rebind cycle fixes this by re-triggering driver probe.
#
# KEY BEHAVIOUR: only resets if the MIDI port did NOT appear within 3 seconds.
# This prevents the script from breaking a healthy device on manual replug.

sleep 3

# Check if the DOREMiDi MIDI port already exists in ALSA
if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: MIDI port already present, no reset needed" | systemd-cat -t reset-doremidi
    exit 0
fi

echo "reset-doremidi: MIDI port missing after 3s, triggering unbind/rebind" | systemd-cat -t reset-doremidi

# Find sysfs device path by VID/PID
DEVICE_PATH=$(grep -rl "1a86" /sys/bus/usb/devices/*/idVendor 2>/dev/null | while read f; do
    dir=$(dirname "$f")
    pid=$(cat "$dir/idProduct" 2>/dev/null)
    if [ "$pid" = "752d" ]; then
        echo "$dir"
        break
    fi
done)

if [ -z "$DEVICE_PATH" ]; then
    echo "reset-doremidi: device not found in sysfs" | systemd-cat -t reset-doremidi
    exit 1
fi

DEVNAME=$(basename "$DEVICE_PATH")
echo "reset-doremidi: unbind/rebind $DEVNAME" | systemd-cat -t reset-doremidi

echo "$DEVNAME" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null
sleep 1
echo "$DEVNAME" > /sys/bus/usb/drivers/usb/bind 2>/dev/null
sleep 2

# Confirm MIDI port appeared after rebind
if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: ✅ MIDI port appeared after rebind" | systemd-cat -t reset-doremidi
else
    echo "reset-doremidi: ⚠️  MIDI port still missing after rebind" | systemd-cat -t reset-doremidi
fi
