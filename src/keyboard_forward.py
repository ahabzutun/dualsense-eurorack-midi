#!/usr/bin/env python3
"""
Keyboard forwarder (keyboard-forward.service): a USB keyboard plugged into the Pi types into Teletype.

The keyboard is read with evdev (and grabbed, so it doesn't also type into the Pi's console); every change
becomes an 8-byte boot-keyboard report on /dev/hidg0, the keyboard half of the Pi's USB gadget
(scripts/teletype_gadget.sh). Teletype reads modifiers from byte 0 and up to six keys from bytes 2–7.

Needs the patched usb_f_hid (scripts/build_teletype_hid.sh): with the kernel's own, Teletype takes the first
key and then nothing more.

Keyboards can come and go; so can /dev/hidg0 (gadget switched to grid only, or Teletype unplugged).
    python3 src/keyboard_forward.py [--hid /dev/hidg0]
"""
import argparse
import os
import select
import sys
import time

# Linux input key codes → USB HID usage IDs (keyboard page 0x07)
KEY_TO_HID = {
    30: 0x04, 48: 0x05, 46: 0x06, 32: 0x07, 18: 0x08, 33: 0x09, 34: 0x0A, 35: 0x0B, 23: 0x0C,
    36: 0x0D, 37: 0x0E, 38: 0x0F, 50: 0x10, 49: 0x11, 24: 0x12, 25: 0x13, 16: 0x14, 19: 0x15,
    31: 0x16, 20: 0x17, 22: 0x18, 47: 0x19, 17: 0x1A, 45: 0x1B, 21: 0x1C, 44: 0x1D,          # a–z
    2: 0x1E, 3: 0x1F, 4: 0x20, 5: 0x21, 6: 0x22, 7: 0x23, 8: 0x24, 9: 0x25, 10: 0x26, 11: 0x27,  # 1–0
    28: 0x28, 1: 0x29, 14: 0x2A, 15: 0x2B, 57: 0x2C,           # enter esc backspace tab space
    12: 0x2D, 13: 0x2E, 26: 0x2F, 27: 0x30, 43: 0x31,          # - = [ ] backslash
    39: 0x33, 40: 0x34, 41: 0x35, 51: 0x36, 52: 0x37, 53: 0x38,  # ; ' ` , . /
    58: 0x39,                                                  # caps lock
    59: 0x3A, 60: 0x3B, 61: 0x3C, 62: 0x3D, 63: 0x3E, 64: 0x3F,  # F1–F6
    65: 0x40, 66: 0x41, 67: 0x42, 68: 0x43, 87: 0x44, 88: 0x45,  # F7–F12
    99: 0x46, 70: 0x47, 119: 0x48,                             # print scroll pause
    110: 0x49, 102: 0x4A, 104: 0x4B, 111: 0x4C, 107: 0x4D, 109: 0x4E,  # ins home pgup del end pgdn
    106: 0x4F, 105: 0x50, 108: 0x51, 103: 0x52,                # right left down up
    69: 0x53, 98: 0x54, 55: 0x55, 74: 0x56, 78: 0x57, 96: 0x58,  # numlock kp/ kp* kp- kp+ kp-enter
    79: 0x59, 80: 0x5A, 81: 0x5B, 75: 0x5C, 76: 0x5D, 77: 0x5E, 71: 0x5F, 72: 0x60, 73: 0x61,
    82: 0x62, 83: 0x63,                                        # kp1–kp9 kp0 kp.
    86: 0x64, 127: 0x65,                                       # the key next to left shift (ISO), menu
}
# modifier key codes → bit in report byte 0
MODIFIERS = {29: 0x01, 42: 0x02, 56: 0x04, 125: 0x08,      # left ctrl shift alt meta
             97: 0x10, 54: 0x20, 100: 0x40, 126: 0x80}     # right ctrl shift alt meta


class ReportState:
    """Pressed keys → boot-keyboard reports. Pure logic, tested without hardware."""

    def __init__(self):
        self.mods = 0
        self.keys = []          # HID usages, in press order

    def event(self, code, value):
        """value 1 = down, 0 = up, 2 = autorepeat. Returns a report when it changed, else None."""
        if value == 2:
            return None         # the host repeats by itself (Teletype does)
        before = (self.mods, tuple(self.keys))
        if code in MODIFIERS:
            if value:
                self.mods |= MODIFIERS[code]
            else:
                self.mods &= ~MODIFIERS[code]
        elif code in KEY_TO_HID:
            usage = KEY_TO_HID[code]
            if value and usage not in self.keys:
                self.keys.append(usage)
            elif not value and usage in self.keys:
                self.keys.remove(usage)
        else:
            return None
        if (self.mods, tuple(self.keys)) == before:
            return None
        return self.report()

    def report(self) -> bytes:
        keys = self.keys[-6:] if len(self.keys) <= 6 else [0x01] * 6      # 0x01 = roll-over error
        return bytes([self.mods, 0] + keys + [0] * (6 - len(keys)))

    def release_all(self) -> bytes:
        self.mods = 0
        self.keys = []
        return bytes(8)


def is_keyboard(dev, ecodes):
    if "DualSense" in dev.name or "Wireless Controller" in dev.name:
        return False
    keys = dev.capabilities().get(ecodes.EV_KEY, [])
    return ecodes.KEY_A in keys and ecodes.KEY_SPACE in keys and ecodes.KEY_ENTER in keys


class HidWriter:
    """Hands reports to the gadget. Teletype collects one report at a time (every 1 ms with the patched
    usb_f_hid), so when keys change faster, wait for the previous one instead of dropping this one —
    a dropped key release would leave the key stuck."""

    def __init__(self, path, timeout=0.1):
        self.path, self.timeout = path, timeout
        self.fd = None
        self.sent = self.waited = self.dropped = 0

    def write(self, report) -> bool:
        if self.fd is None:
            try:
                self.fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                return False
        deadline = time.monotonic() + self.timeout
        waited = False
        while True:
            try:
                os.write(self.fd, report)
                self.sent += 1
                self.waited += waited
                return True
            except BlockingIOError:
                left = deadline - time.monotonic()
                if left <= 0:
                    self.dropped += 1
                    print("⚠️  Teletype isn't collecting keys (is the keyboard part of the gadget active?)",
                          flush=True)
                    return False
                waited = True
                select.select([], [self.fd], [], min(left, 0.01))
            except OSError:
                try:
                    os.close(self.fd)
                except OSError:
                    pass
                self.fd = None      # gadget switched or Teletype gone: reopen on the next report
                return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hid", default="/dev/hidg0")
    args = ap.parse_args(argv)
    import evdev
    from evdev import ecodes

    state = ReportState()
    keyboards = {}              # fd -> device
    next_scan = 0.0

    hid = HidWriter(args.hid)
    write = hid.write

    print("⌨️  keyboard forwarder running", flush=True)
    while True:
        now = time.monotonic()
        if now >= next_scan:
            next_scan = now + 2.0
            known = {d.path for d in keyboards.values()}
            for path in evdev.list_devices():
                if path in known:
                    continue
                try:
                    dev = evdev.InputDevice(path)
                except OSError:
                    continue
                if not is_keyboard(dev, ecodes):
                    dev.close()
                    continue
                try:
                    dev.grab()
                except OSError:
                    pass
                keyboards[dev.fd] = dev
                print(f"✅ keyboard: {dev.name}", flush=True)
        if not keyboards:
            time.sleep(0.5)
            continue
        r = select.select(list(keyboards), [], [], 0.5)[0]
        for fd in r:
            dev = keyboards[fd]
            try:
                for ev in dev.read():
                    if ev.type == ecodes.EV_KEY:
                        rep = state.event(ev.code, ev.value)
                        if rep is not None:
                            write(rep)
            except OSError:
                print(f"❌ keyboard gone: {dev.name}", flush=True)
                keyboards.pop(fd, None)
                write(state.release_all())


if __name__ == "__main__":
    sys.exit(main())
