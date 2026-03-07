#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The device gets urb status -32 (EPIPE) on boot — the endpoint is stalled.
#
# Two-step fix:
#   1. unbind/rebind  → re-triggers driver probe, creates ALSA MIDI port
#   2. USBDEVFS_RESET → clears the stalled endpoint, data flows again
#
# Both steps are needed: rebind alone creates the port but leaves the stall.
# Reset alone clears the stall but the driver may not rebind cleanly.

sleep 1

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
BUS=$(cat "$DEVICE_PATH/busnum" 2>/dev/null)
DEV=$(cat "$DEVICE_PATH/devnum" 2>/dev/null)

echo "reset-doremidi: step 1 — unbind/rebind $DEVNAME" | systemd-cat -t reset-doremidi

# Step 1: unbind/rebind to re-create the ALSA MIDI port
echo "$DEVNAME" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null
sleep 1
echo "$DEVNAME" > /sys/bus/usb/drivers/usb/bind 2>/dev/null
sleep 2
<<<<<<< Updated upstream

# Re-read device number — it changes after rebind
DEV=$(cat "$DEVICE_PATH/devnum" 2>/dev/null)
DEVNODE="/dev/bus/usb/$(printf '%03d' $BUS)/$(printf '%03d' $DEV)"

echo "reset-doremidi: step 2 — USBDEVFS_RESET $DEVNODE" | systemd-cat -t reset-doremidi

# Step 2: USB reset to clear stalled endpoint
python3 -c "
import fcntl, sys
USBDEVFS_RESET = 0x5514
try:
    with open('$DEVNODE', 'wb') as f:
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
except Exception as e:
    print(f'reset failed: {e}', file=sys.stderr)
    sys.exit(1)
" 2>&1 | systemd-cat -t reset-doremidi

sleep 2

=======
echo "1a86 752d" > /sys/bus/usb/drivers/snd-usb-audio/new_id 2>/dev/null
sleep 1
>>>>>>> Stashed changes
if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: ✅ done" | systemd-cat -t reset-doremidi
else
    echo "reset-doremidi: ⚠️  MIDI port missing after reset" | systemd-cat -t reset-doremidi
fi
