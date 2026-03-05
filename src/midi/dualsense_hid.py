"""
Minimal DualSense HID output driver.

Replaces pydualsense entirely for our use case: we only need to write
LED colour, player indicator dots, and haptic motor intensity to the
controller. All input comes through evdev — we never read pydualsense's
parsed state — so the full pydualsense library was pure overhead.

Why this exists
---------------
pydualsense.sendReport() is a tight while loop with no sleep that both
reads the full HID report (250 Hz) AND writes the output report every
cycle. It creates two fresh Python lists per iteration (inReport +
outReport) = 500+ allocations/sec. Measured at ~245 KB/10s = ~88 MB/hr
of pymalloc arena growth that could not be stopped without breaking LED
and haptic timing.

This class opens /dev/hidraw0 directly and writes a pre-built 64-byte
USB output report from a background thread. The thread only writes when
state has changed (dirty flag), sleeping 20 ms between polls (50 Hz max
write rate). Since we never read from hidraw0, there is zero per-cycle
allocation — the same bytearray is reused on every write.

Public interface (drop-in replacement for the pydualsense ds object)
--------------------------------------------------------------------
    ds = DualSenseHID()
    ds.init()
    ds.setLeftMotor(intensity)      # 0-255
    ds.setRightMotor(intensity)     # 0-255
    ds.light.setColorI(r, g, b)     # 0-255 each
    ds.light.setPlayerID(player)    # PlayerID enum or plain int
    ds.close()

USB HID output report layout (64 bytes, report ID 0x02)
-------------------------------------------------------
    [0]     0x02        report ID (USB)
    [1]     0xFF        enable flags byte 1 (all features)
    [2]     0x57        enable flags byte 2:
                          0x01 main motors
                          0x02 main motors (both bits required)
                          0x04 trigger motors (unused but kept)
                          0x10 player indicator LEDs
                          0x40 LED brightness control
    [3]     rightMotor  0-255
    [4]     leftMotor   0-255
    [5-38]  0x00        audio / trigger effects (unused)
    [39]    0x03        ledOption = LedOptions.Both (0x01|0x02)
    [40-41] 0x00
    [42]    0x00        pulseOptions = PulseOptions.Off
    [43]    0x00        brightness = Brightness.high
    [44]    playerID    PlayerID value (0/4/10/21/27/31)
    [45]    R           touchpad LED red   0-255
    [46]    G           touchpad LED green 0-255
    [47]    B           touchpad LED blue  0-255
    [48-63] 0x00        unused
"""

import os
import glob
import threading
import time


class PlayerID:
    """PlayerID constants — mirrors pydualsense.enums.PlayerID."""
    PLAYER_1 = 4
    PLAYER_2 = 10
    PLAYER_3 = 21
    PLAYER_4 = 27
    ALL      = 31

    def __init__(self, value: int = 0):
        self.value = value

    def __int__(self):
        return self.value


class _DSLight:
    """Holds LED + player dot state. Mirrors pydualsense.DSLight interface."""

    def __init__(self, hid: "DualSenseHID"):
        self._hid = hid

    def setColorI(self, r: int, g: int, b: int) -> None:
        with self._hid._lock:
            self._hid._r = max(0, min(255, int(r)))
            self._hid._g = max(0, min(255, int(g)))
            self._hid._b = max(0, min(255, int(b)))
            self._hid._dirty = True

    def setPlayerID(self, player) -> None:
        value = int(player) if not isinstance(player, int) else player
        with self._hid._lock:
            self._hid._player_id = value
            self._hid._dirty = True


class DualSenseHID:
    """
    Minimal DualSense output driver — LED, player dots, haptic motors.
    Drop-in replacement for the pydualsense top-level object.
    """

    _REPORT_ID  = 0x02
    _FLAGS1     = 0xFF
    _FLAGS2     = 0x01 | 0x02 | 0x04 | 0x10 | 0x40  # motors + player LED + brightness
    _LED_OPTION = 0x03   # LedOptions.Both (0x01|0x02)
    _IDLE_SLEEP = 0.02   # 50 Hz poll interval

    def __init__(self):
        self._fd          = None
        self._right_motor = 0
        self._left_motor  = 0
        self._r           = 0
        self._g           = 0
        self._b           = 0
        self._player_id   = 0
        self._dirty       = True
        self._running     = False
        self._lock        = threading.Lock()
        self._thread      = None
        self._report      = bytearray(64)  # pre-allocated, reused every write
        self.light        = _DSLight(self)

    @staticmethod
    def _find_hidraw() -> str:
        """Return /dev/hidrawN path for the connected DualSense (VID 054C, PID 0CE6)."""
        for hidraw in sorted(glob.glob("/dev/hidraw*")):
            try:
                uevent = f"/sys/class/hidraw/{os.path.basename(hidraw)}/device/uevent"
                with open(uevent) as f:
                    content = f.read().upper()
                if "054C" in content and "0CE6" in content:
                    return hidraw
            except OSError:
                continue
        return "/dev/hidraw0"  # fallback

    def init(self) -> None:
        """Open the HID device and start the background write thread."""
        path = self._find_hidraw()
        self._fd = os.open(path, os.O_WRONLY)
        print(f"[DualSenseHID] ✅ Opened {path} for output")
        self._running = True
        self._dirty   = True
        self._thread  = threading.Thread(
            target=self._write_loop,
            daemon=True,
            name="DualSenseHID-writer"
        )
        self._thread.start()

    def close(self) -> None:
        """Stop the write thread and close the HID device."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        print("[DualSenseHID] Closed")

    def setLeftMotor(self, intensity: int) -> None:
        with self._lock:
            self._left_motor = max(0, min(255, int(intensity)))
            self._dirty = True

    def setRightMotor(self, intensity: int) -> None:
        with self._lock:
            self._right_motor = max(0, min(255, int(intensity)))
            self._dirty = True

    def _build_report(self) -> None:
        """Fill self._report from current state. Must be called with lock held."""
        r = self._report
        r[0]  = self._REPORT_ID
        r[1]  = self._FLAGS1
        r[2]  = self._FLAGS2
        r[3]  = self._right_motor
        r[4]  = self._left_motor
        r[39] = self._LED_OPTION
        r[42] = 0x00  # PulseOptions.Off
        r[43] = 0x00  # Brightness.high
        r[44] = self._player_id
        r[45] = self._r
        r[46] = self._g
        r[47] = self._b

    def _write_loop(self) -> None:
        """
        Background thread: write the HID report whenever state changes.
        Sleeps 20ms between polls — zero I/O and zero allocation when idle.
        """
        while self._running:
            with self._lock:
                if self._dirty:
                    self._build_report()
                    self._dirty = False
                    data = bytes(self._report)
                else:
                    data = None

            if data is not None and self._fd is not None:
                try:
                    os.write(self._fd, data)
                except OSError as e:
                    print(f"[DualSenseHID] Write error: {e}")

            time.sleep(self._IDLE_SLEEP)
