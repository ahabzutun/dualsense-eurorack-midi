#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The CH345 chip gets urb status -32 (EPIPE) on first enumeration — the MIDI
# endpoint is stalled. A single USBDEVFS_RESET clears it without causing
# re-enumeration (unlike unbind/rebind which triggers a second stall cycle).
#
# Wait long enough for snd-usb-audio probe attempts to finish before resetting,
# otherwise the reset races with driver binding.

sleep 3

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

BUS=$(cat "$DEVICE_PATH/busnum" 2>/dev/null)
DEV=$(cat "$DEVICE_PATH/devnum" 2>/dev/null)
DEVNODE="/dev/bus/usb/$(printf '%03d' $BUS)/$(printf '%03d' $DEV)"

echo "reset-doremidi: USBDEVFS_RESET $DEVNODE" | systemd-cat -t reset-doremidi

python3 -c "
import fcntl, sys
USBDEVFS_RESET = 0x5514
try:
    with open('$DEVNODE', 'wb') as f:
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
    print('reset ok')
except Exception as e:
    print(f'reset failed: {e}', file=sys.stderr)
    sys.exit(1)
" 2>&1 | systemd-cat -t reset-doremidi

sleep 2

if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: ✅ done" | systemd-cat -t reset-doremidi
else
    echo "reset-doremidi: ⚠️  MIDI port missing after reset" | systemd-cat -t reset-doremidi
fi
