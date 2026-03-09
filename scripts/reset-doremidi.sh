#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The CH345 chip gets urb status -32 (EPIPE) on first enumeration.
# An unbind/rebind cycle re-triggers snd-usb-midi probe and clears the stall.

sleep 2

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
sleep 3

if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: ✅ done" | systemd-cat -t reset-doremidi
else
    echo "reset-doremidi: ⚠️  MIDI port missing after reset" | systemd-cat -t reset-doremidi
fi
