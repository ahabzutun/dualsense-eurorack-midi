"""
The monome "mext" serial protocol, as far as the grid ⇄ Teletype link needs it.

Checked against the Teletype firmware (monome/libavr32, src/monome.c):
  - Teletype → grid sends only  0x05 (size query, once on connect)  and
    0x1A x y d[32] (a whole 8×8 quad of levels, two 4-bit levels per byte, row by row).
  - Teletype reads only  0x20 x y (key up),  0x21 x y (key down)  and the 0x03 cols rows size answer.
    Its parser stops at the first byte it doesn't know, so nothing else may reach it.

The grid side can say a bit more (query / id / version answers); those are parsed so they can be skipped.
"""

# host → device commands and their total lengths (command byte included)
HOST_LEN = {
    0x00: 1,  # system query
    0x01: 1,  # system id
    0x02: 33,  # write id
    0x03: 1,  # grid offset get
    0x04: 4,  # grid offset set (n x y)
    0x05: 1,  # grid size get
    0x06: 3,  # grid size set
    0x07: 1,  # addr get
    0x08: 3,  # addr set
    0x0F: 1,  # firmware version
    0x10: 3, 0x11: 3,  # led off / on x y
    0x12: 1, 0x13: 1,  # all off / on
    0x14: 11,  # led map x y d[8]
    0x15: 4, 0x16: 4,  # row / col x y d
    0x17: 2,  # intensity
    0x18: 4,  # level set x y l
    0x19: 2,  # level all l
    0x1A: 35,  # level map x y d[32]
    0x1B: 7, 0x1C: 7,  # level row / col x y d[4]
}

# device → host messages and their total lengths
DEVICE_LEN = {
    0x00: 3,  # query answer (section, count)
    0x01: 33,  # id string
    0x02: 3,  # grid offset
    0x03: 3,  # grid size (cols, rows)
    0x04: 3,  # addr
    0x0F: 9,  # firmware version
    0x20: 3, 0x21: 3,  # key up / down
    0x50: 3,  # encoder delta (arc)
    0x51: 2, 0x52: 2,  # encoder key up / down
    0x60: 3,  # analog in
    0x80: 2,  # tilt active
    0x81: 8,  # tilt
}

QUAD = 8


class StreamParser:
    """Turns a byte stream into complete messages (tuples of ints). Unknown command bytes are dropped
    one by one, so the parser resynchronises after a glitch instead of getting stuck."""

    def __init__(self, lengths):
        self.lengths = lengths
        self.buf = bytearray()
        self.dropped = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while self.buf:
            n = self.lengths.get(self.buf[0])
            if n is None:
                del self.buf[0]
                self.dropped += 1
                continue
            if len(self.buf) < n:
                break
            out.append(bytes(self.buf[:n]))
            del self.buf[:n]
        return out


def key(x: int, y: int, down: bool) -> bytes:
    return bytes((0x21 if down else 0x20, x & 0xFF, y & 0xFF))


def size_answer(cols: int, rows: int) -> bytes:
    return bytes((0x03, cols, rows))


def level_map(qx: int, qy: int, levels) -> bytes:
    """levels: 64 values 0-15, row by row (8 per row)."""
    out = bytearray((0x1A, qx, qy))
    for i in range(0, 64, 2):
        out.append(((levels[i] & 0x0F) << 4) | (levels[i + 1] & 0x0F))
    return bytes(out)


def unpack_level_map(msg: bytes):
    """0x1A message → (qx, qy, 64 levels)."""
    levels = []
    for b in msg[3:35]:
        levels.append(b >> 4)
        levels.append(b & 0x0F)
    return msg[1], msg[2], levels
