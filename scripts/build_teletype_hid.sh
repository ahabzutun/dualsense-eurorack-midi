#!/usr/bin/env bash
# Builds and installs a patched usb_f_hid (the kernel's USB gadget keyboard) for the running kernel,
# so a keyboard on the Pi can type into Teletype through the USB gadget.
#
#   sudo scripts/build_teletype_hid.sh            # build + install if missing for this kernel
#   sudo scripts/build_teletype_hid.sh --force    # build + install again
#   scripts/build_teletype_hid.sh --check         # exit 0 when this kernel has it installed
#
# Why (found on the hardware, 2026-11-17): with the stock module Teletype collects the first key report
# and then nothing more. Two things together:
#   1. The interrupt endpoint's interval (bInterval 10 / 4): the Pi's dwc2 controller then schedules
#      each report for a frame parity that Teletype's host (AVR32 USBB) polling doesn't hit.  → 1
#   2. A zero-length packet after every report (req->zero = 1, kernel fix from 2025): Teletype's host
#      doesn't re-arm cleanly after a ZLP / short packet.                                       → 0
# Either change alone is not enough; both together: every report collected.
#
# Installed to /lib/modules/<kernel>/updates/ (wins over the stock module); a kernel update needs a new
# build — teletype-hid-module.service runs this script at boot when the module is missing.
# Needs: the kernel headers (linux-headers-rpi-v8, installed with the kernel), gcc, make, internet.
set -euo pipefail

KVER="$(uname -r)"
DEST="/lib/modules/$KVER/updates/usb_f_hid.ko"
MARK="/lib/modules/$KVER/updates/usb_f_hid.teletype"

if [ "${1:-}" = "--check" ]; then
    [ -f "$DEST" ] && [ -f "$MARK" ]
    exit $?
fi
if [ "${1:-}" != "--force" ] && [ -f "$DEST" ] && [ -f "$MARK" ]; then
    echo "patched usb_f_hid already installed for $KVER"
    exit 0
fi
[ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }

BUILD="/lib/modules/$KVER/build"
[ -d "$BUILD" ] || { echo "no kernel headers for $KVER ($BUILD) — apt install linux-headers-rpi-v8" >&2; exit 1; }

# the Raspberry Pi kernel branch for this kernel series, e.g. 6.12.62+rpt-rpi-v8 -> rpi-6.12.y
SERIES="$(echo "$KVER" | sed -E 's/^([0-9]+\.[0-9]+).*/\1/')"
SRC="https://raw.githubusercontent.com/raspberrypi/linux/rpi-$SERIES.y/drivers/usb/gadget/function"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
for f in f_hid.c u_hid.h; do
    curl -sfSL --retry 12 --retry-delay 5 --retry-all-errors -o "$f" "$SRC/$f" || { echo "could not download $f for rpi-$SERIES.y" >&2; exit 1; }
done

# 1. interval 1 on every endpoint descriptor (full / high / super speed)
n_int=$(grep -cE '\.bInterval\s*=\s*(4|10),' f_hid.c || true)
sed -i -E 's/(\.bInterval\s*=\s*)(4|10),/\11,/' f_hid.c
# 2. no zero-length packet after a report: only the IN write path (f_hidg_write)
n_zero=$(grep -cE '^\s*req->zero\s*=\s*1;' f_hid.c || true)
python3 - <<'EOF'
import re, sys
s = open("f_hid.c").read()
i = s.find("static ssize_t f_hidg_write(")
j = s.find("\n}\n", i)
if i < 0 or j < 0:
    sys.exit("f_hidg_write not found — the kernel source changed, check the patch")
body, n = re.subn(r"(\n\s*req->zero\s*=\s*)1;", r"\g<1>0; /* teletype: no ZLP */", s[i:j], count=1)
if n != 1:
    sys.exit("req->zero = 1 not found in f_hidg_write — the kernel source changed, check the patch")
open("f_hid.c", "w").write(s[:i] + body + s[j:])
EOF
if [ "$n_int" -lt 2 ]; then
    echo "expected bInterval lines not found ($n_int) — the kernel source changed, check the patch" >&2
    exit 1
fi
echo "patched: $n_int bInterval lines → 1, IN writes without ZLP (was: $n_zero 'req->zero = 1')"

printf 'obj-m := usb_f_hid.o\nusb_f_hid-y := f_hid.o\n' > Makefile
make -s -C "$BUILD" M="$WORK" modules >/dev/null
install -D -m 0644 usb_f_hid.ko "$DEST"
date -Is > "$MARK"
depmod -a "$KVER"
echo "installed $DEST"
echo "active from the next boot (or: rebuild the gadget, see docs/grid-macros.md)"
