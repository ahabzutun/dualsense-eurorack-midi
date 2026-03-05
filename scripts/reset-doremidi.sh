#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The device gets urb status -32 (EPIPE) on first boot enumeration, leaving
# the endpoint stalled — it appears in aconnect but sends no MIDI data.
# A USB unbind/rebind cycle re-triggers driver probe and clears the stall.
#
# We always rebind on connect. The rebind takes ~3 seconds total and is
# harmless on manual replugs — the device simply re-enumerates cleanly.
# The passthrough service's hot-plug detection handles the brief disconnect.

sleep 1

# Find the sysfs device path by VID/PID
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

if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: ✅ MIDI port present after rebind" | systemd-cat -t reset-doremidi
else
    echo "reset-doremidi: ⚠️  MIDI port missing after rebind" | systemd-cat -t reset-doremidi
fi
