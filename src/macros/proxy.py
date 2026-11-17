"""
The grid proxy: the Pi sits between the real grid and Teletype and splits the grid in two.

    real grid ⇄ (/dev/ttyACM*)  Pi  (/dev/ttyGS0, USB gadget) ⇄ Teletype

  - keys with x < 8 go to the macro engine, keys with x ≥ 8 go on to Teletype unchanged
  - Teletype's LED quad for x = 8 goes on to the grid; its quad for x = 0 is dropped, the Pi draws that one
  - Teletype's size query (0x05) is answered by the Pi at once — Teletype waits only ~0.5 ms for it
    (libavr32 monome_setup_mext) and the real grid may not even be there yet
  - nothing else reaches Teletype: its parser stops at the first byte it doesn't know

GridSplitter is the pure logic (bytes in, bytes out) and is tested without any hardware.
"""
from . import mext


class GridSplitter:
    def __init__(self, on_macro_key, split_x=8, cols=16, rows=8):
        self.on_macro_key = on_macro_key      # (x, y, down)
        self.split_x = split_x
        self.cols = cols
        self.rows = rows
        self.from_grid_parser = mext.StreamParser(mext.DEVICE_LEN)
        self.from_tt_parser = mext.StreamParser(mext.HOST_LEN)
        self.tt_quads = {}                    # (qx, qy) -> last level map bytes Teletype sent for its half
        self.on_teletype_leds = None          # (qx, qy, levels) — mirror for the teletype provider
        self.grid_answered_size = None

    # grid → Pi
    def from_grid(self, data: bytes) -> bytes:
        """Bytes from the real grid. Returns the bytes to pass on to Teletype."""
        out = bytearray()
        for msg in self.from_grid_parser.feed(data):
            cmd = msg[0]
            if cmd in (0x20, 0x21):
                x, y = msg[1], msg[2]
                if x < self.split_x:
                    self.on_macro_key(x, y, cmd == 0x21)
                else:
                    out += msg
            elif cmd == 0x03:
                self.grid_answered_size = (msg[1], msg[2])
            # everything else (ids, versions, tilt …) stays with the Pi
        return bytes(out)

    # Teletype → Pi
    def from_teletype(self, data: bytes):
        """Bytes from Teletype. Returns (answer to Teletype, bytes for the grid)."""
        to_tt = bytearray()
        to_grid = bytearray()
        for msg in self.from_tt_parser.feed(data):
            cmd = msg[0]
            if cmd == 0x05:
                to_tt += mext.size_answer(self.cols, self.rows)
            elif cmd == 0x00:
                to_tt += bytes((0x00, 0x01, 0x01))            # one grid section
            elif cmd == 0x01:
                to_tt += bytes((0x01,)) + b"m-pi-grid-proxy".ljust(32, b"\0")
            elif cmd == 0x1A:
                qx, qy = msg[1], msg[2]
                if qx >= self.split_x:
                    self.tt_quads[(qx, qy)] = msg
                    to_grid += msg
                    if self.on_teletype_leds is not None:
                        self.on_teletype_leds(*mext.unpack_level_map(msg))
            elif cmd in (0x10, 0x11, 0x18):
                if msg[1] >= self.split_x:                    # single LEDs on Teletype's half
                    to_grid += msg
            # the rest (all on/off, intensity, rows/cols spanning both halves) is not something
            # Teletype sends; dropped so it can't paint over the macro half
        return bytes(to_tt), bytes(to_grid)

    def macro_frame(self, levels) -> bytes:
        """The Pi's own half as one level map for the grid."""
        return mext.level_map(0, 0, levels)

    def resend_teletype_half(self) -> bytes:
        """After the grid (re)connects: what Teletype last drew on its half."""
        return b"".join(self.tt_quads.values())
