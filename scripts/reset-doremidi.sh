#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The device consistently has urb status -32 (EPIPE) on first enumeration,
# causing it to drop off USB. A USB reset via USBDEVFS_RESET ioctl clears
# the stall and brings it up cleanly without needing a physical replug.
#
# Run as root by udev. The 2-second sleep gives the kernel time to finish
# initial enumeration before we reset — too fast and the reset races with
# the driver bind.

sleep 2

# Find the sysfs path for this device by VID/PID
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

BUS=$(cat "$DEVICE_PATH/busnum" 2>/dev/null)
DEV=$(cat "$DEVICE_PATH/devnum" 2>/dev/null)
DEVNODE="/dev/bus/usb/$(printf '%03d' $BUS)/$(printf '%03d' $DEV)"

echo "reset-doremidi: resetting $DEVNODE" | systemd-cat -t reset-doremidi

python3 -c "
import fcntl, sys
USBDEVFS_RESET = 0x5514
try:
    with open('$DEVNODE', 'wb') as f:
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
    sys.exit(0)
except Exception as e:
    print(f'reset failed: {e}')
    sys.exit(1)
" | systemd-cat -t reset-doremidi
