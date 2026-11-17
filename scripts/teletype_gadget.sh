#!/usr/bin/env bash
# The Pi's USB-C port as Teletype's USB device (configfs gadget): a grid, a keyboard, or both at once.
#
#   sudo scripts/teletype_gadget.sh [both|grid|keyboard|off|status]      (default: both)
#
#   both      one composite device: CDC serial (the grid, /dev/ttyGS0) + boot keyboard (/dev/hidg0)
#             Teletype's USB host hands every interface to its drivers (libavr32 uhc.c), and its keyboard
#             and grid handlers are independent, so this *should* give grid and keyboard together.
#             Untested on hardware so far — if Teletype only takes one of them, use grid / keyboard.
#   grid      only the grid (serial): what grid-macros.service needs
#   keyboard  only the keyboard (keyboard-forward.service types into it)
#   off       unbind (Teletype sees the device unplugged)
#
# Switching modes unbinds and rebinds: Teletype sees an unplug and a new device, like swapping cables.
#
# Needs (install-pi.sh --grid-macros sets these up, then a reboot):
#   /boot/firmware/config.txt:  dtoverlay=dwc2,dr_mode=peripheral   (only the USB-C port changes;
#                               the four USB-A ports are a separate controller and stay hosts)
#   the modules dwc2 and libcomposite (loaded here if needed)
# Cabling: the Pi's USB-C to Teletype through the power/data splitter — power from the Pi's supply, data
# to Teletype. No VBUS from the Pi towards Teletype (see learnings: no backfeed).
set -euo pipefail

MODE="${1:-both}"
G=/sys/kernel/config/usb_gadget/teletype

need_root() { [ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }; }

status() {
    if [ ! -d "$G" ]; then echo "gadget: not set up"; return; fi
    local udc funcs
    udc="$(cat "$G/UDC" 2>/dev/null || true)"
    funcs="$(find "$G/configs/c.1" -maxdepth 1 -type l -printf '%f ' 2>/dev/null || true)"
    echo "gadget: ${udc:-unbound} · functions: ${funcs:-none}"
    [ -e /dev/ttyGS0 ] && echo "  grid serial:  /dev/ttyGS0"
    [ -e /dev/hidg0 ] && echo "  keyboard:     /dev/hidg0"
    return 0
}

if [ "$MODE" = "status" ]; then status; exit 0; fi
need_root

case "$MODE" in both|grid|keyboard|off) ;; *) echo "mode: both | grid | keyboard | off | status" >&2; exit 2 ;; esac

modprobe libcomposite
if ! ls /sys/class/udc >/dev/null 2>&1 || [ -z "$(ls /sys/class/udc 2>/dev/null)" ]; then
    modprobe dwc2 2>/dev/null || true
    sleep 0.5
fi
UDC_NAME="$(ls /sys/class/udc 2>/dev/null | head -n1 || true)"
if [ -z "$UDC_NAME" ]; then
    echo "No USB device controller. Is 'dtoverlay=dwc2,dr_mode=peripheral' in /boot/firmware/config.txt" >&2
    echo "(and did the Pi reboot since)?" >&2
    exit 1
fi
mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config

# ---- build the gadget once ---------------------------------------------------------------------------
if [ ! -d "$G" ]; then
    mkdir -p "$G"
    echo 0x1d6b > "$G/idVendor"            # Linux Foundation
    echo 0x0104 > "$G/idProduct"           # Multifunction Composite Gadget
    echo 0x0100 > "$G/bcdDevice"
    echo 0x0200 > "$G/bcdUSB"
    echo 0xEF > "$G/bDeviceClass"          # composite with interface association (needed for CDC)
    echo 0x02 > "$G/bDeviceSubClass"
    echo 0x01 > "$G/bDeviceProtocol"
    mkdir -p "$G/strings/0x409"
    echo "ahab" > "$G/strings/0x409/manufacturer"
    echo "Teletype grid + keyboard" > "$G/strings/0x409/product"
    echo "ahab-tt-0001" > "$G/strings/0x409/serialnumber"

    mkdir -p "$G/configs/c.1/strings/0x409"
    echo "grid + keyboard" > "$G/configs/c.1/strings/0x409/configuration"
    echo 0xC0 > "$G/configs/c.1/bmAttributes"   # self-powered: Teletype supplies nothing
    echo 2 > "$G/configs/c.1/MaxPower"

    # the grid: CDC ACM serial (Teletype accepts any CDC data interface as a grid)
    mkdir -p "$G/functions/acm.grid"

    # the keyboard: standard boot keyboard, 8-byte reports (Teletype reads modifiers + 6 keys)
    mkdir -p "$G/functions/hid.kbd"
    echo 1 > "$G/functions/hid.kbd/protocol"
    echo 1 > "$G/functions/hid.kbd/subclass"
    echo 8 > "$G/functions/hid.kbd/report_length"
    printf '%b' '\x05\x01\x09\x06\xa1\x01\x05\x07\x19\xe0\x29\xe7\x15\x00\x25\x01\x75\x01\x95\x08\x81\x02\x95\x01\x75\x08\x81\x03\x95\x05\x75\x01\x05\x08\x19\x01\x29\x05\x91\x02\x95\x01\x75\x03\x91\x03\x95\x06\x75\x08\x15\x00\x25\x65\x05\x07\x19\x00\x29\x65\x81\x00\xc0' \
        > "$G/functions/hid.kbd/report_desc"
fi

# ---- (re)bind with the functions of this mode --------------------------------------------------------
if [ -n "$(cat "$G/UDC" 2>/dev/null || true)" ]; then
    echo "" > "$G/UDC"
    sleep 0.3                               # long enough for Teletype to see the unplug
fi
for l in "$G"/configs/c.1/*.*; do
    [ -L "$l" ] && rm "$l"
done

# Keyboard FIRST: Teletype's CDC driver (libavr32 uhi_cdc_install) only accepts the device when the *last*
# interface in the configuration is the CDC data interface — tested: grid first = no grid on Teletype.
case "$MODE" in
    both)     ln -s "$G/functions/hid.kbd" "$G/configs/c.1/"; ln -s "$G/functions/acm.grid" "$G/configs/c.1/" ;;
    grid)     ln -s "$G/functions/acm.grid" "$G/configs/c.1/" ;;
    keyboard) ln -s "$G/functions/hid.kbd" "$G/configs/c.1/" ;;
    off)      status; exit 0 ;;
esac
echo "$UDC_NAME" > "$G/UDC"

# let the service user open the device files (group dialout for ttyGS0 already; hidg0 is root-only)
sleep 0.3
[ -e /dev/hidg0 ] && chgrp input /dev/hidg0 && chmod 0660 /dev/hidg0
status
