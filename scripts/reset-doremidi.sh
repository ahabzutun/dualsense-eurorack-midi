#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The CH345 chip gets urb status -32 (EPIPE) on first enumeration — endpoint
# 0x81 is stalled.
#
# Fix sequence:
#   1. unbind/rebind the USB device → snd-usb-midi re-probes, ALSA port created
#   2. USBDEVFS_CLEAR_HALT on EP 0x81 → clears the endpoint stall flag
#      WITHOUT causing re-enumeration (unlike USBDEVFS_RESET)

sleep 2

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

echo "reset-doremidi: step 1 — unbind/rebind $DEVNAME" | systemd-cat -t reset-doremidi

echo "$DEVNAME" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null
sleep 1
echo "$DEVNAME" > /sys/bus/usb/drivers/usb/bind 2>/dev/null
sleep 3

# Re-read device number after rebind (it changes)
DEV=$(cat "$DEVICE_PATH/devnum" 2>/dev/null)
DEVNODE="/dev/bus/usb/$(printf '%03d' $BUS)/$(printf '%03d' $DEV)"

echo "reset-doremidi: step 2 — CLEAR_HALT EP 0x81 on $DEVNODE" | systemd-cat -t reset-doremidi

# USBDEVFS_CLEAR_HALT clears the endpoint halt flag without re-enumeration
python3 -c "
import fcntl, sys, struct
USBDEVFS_CLEAR_HALT = 0x5515
EP_IN = 0x81
try:
    with open('$DEVNODE', 'wb') as f:
        fcntl.ioctl(f, USBDEVFS_CLEAR_HALT, struct.pack('I', EP_IN))
    print('clear_halt ok')
except Exception as e:
    print(f'clear_halt failed: {e}', file=sys.stderr)
" 2>&1 | systemd-cat -t reset-doremidi

sleep 2

if aconnect -l 2>/dev/null | grep -qi "doremidi"; then
    echo "reset-doremidi: ✅ done" | systemd-cat -t reset-doremidi
else
    echo "reset-doremidi: ⚠️  MIDI port missing after reset" | systemd-cat -t reset-doremidi
fi
