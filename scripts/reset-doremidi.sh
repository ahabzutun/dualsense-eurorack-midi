#!/bin/bash
# reset-doremidi.sh
#
# Called by udev when the DOREMiDi MPC-20-30C4 (VID 1a86 PID 752d) connects.
# The device gets urb status -32 (EPIPE) on first enumeration. A USB reset
# alone is not enough — after USBDEVFS_RESET the kernel MIDI driver doesn't
# automatically rebind. We must unbind and rebind the driver explicitly.
#
# Run as root by udev.

sleep 2

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

# Get the device's kernel name (e.g. "1-1.3.3")
DEVNAME=$(basename "$DEVICE_PATH")
BUS=$(cat "$DEVICE_PATH/busnum" 2>/dev/null)
DEV=$(cat "$DEVICE_PATH/devnum" 2>/dev/null)
DEVNODE="/dev/bus/usb/$(printf '%03d' $BUS)/$(printf '%03d' $DEV)"

echo "reset-doremidi: unbind/rebind $DEVNAME ($DEVNODE)" | systemd-cat -t reset-doremidi

# Unbind the USB device from its current driver
echo "$DEVNAME" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null

sleep 1

# Rebind — this re-triggers driver probe for all interfaces (MIDI driver rebinds)
echo "$DEVNAME" > /sys/bus/usb/drivers/usb/bind 2>/dev/null

sleep 1

echo "reset-doremidi: done" | systemd-cat -t reset-doremidi
