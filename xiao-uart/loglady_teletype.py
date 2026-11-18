#!/usr/bin/env python3
"""loglady-teletype: the Pi side of loglady, the XIAO that is the Pi's exclusive link to Teletype.

Replaces the Pi's USB gadget (dwc2 + configfs) and the power/data splitter:
  /run/loglady/grid   pseudo-terminal = the grid serial towards Teletype   (grid_macros.py --teletype)
  /run/loglady/kbd    FIFO, 8-byte boot-keyboard reports                    (keyboard_forward.py --hid)
  /run/loglady/ctl    FIFO, text commands: "replug" (Teletype sees an unplug + new device, re-queries
                      the grid — what `teletype_gadget.sh rebind` did), "status"

UART to loglady (/dev/ttyAMA5, 1 Mbaud): COBS frames, payload = type, data..., crc8. See the firmware
(firmware/loglady-teletype/src/main.cpp). Exclusive: nothing is forwarded until the board on the UART
has answered the hello with "loglady teletype".

    python3 loglady_teletype.py [--dev /dev/ttyAMA5] [--baud 1000000]
    python3 loglady_teletype.py --replug        # ask the running service to replug loglady
"""
import argparse, json, os, select, stat, sys, termios, time, tty

RUN = "/run/loglady"
IDENT_PREFIX = b"loglady teletype"

T_GRID_OUT, T_GRID_IN, T_KBD = 0x01, 0x02, 0x03
T_HELLO, T_REPLUG, T_PING = 0x10, 0x11, 0x12
T_IDENT, T_STATUS = 0x20, 0x21


def log(*a):
    print(*a, flush=True)


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def cobs_encode(raw: bytes) -> bytes:
    out, block = bytearray(), bytearray()
    for b in raw:
        if b == 0:
            out.append(len(block) + 1); out += block; block.clear()
        else:
            block.append(b)
            if len(block) == 254:
                out.append(255); out += block; block.clear()
    out.append(len(block) + 1); out += block
    return bytes(out)


def cobs_decode(enc: bytes) -> bytes:
    out, i, n = bytearray(), 0, len(enc)
    while i < n:
        code = enc[i]; i += 1
        if code == 0 or i + code - 1 > n:
            raise ValueError("bad cobs")
        out += enc[i:i + code - 1]; i += code - 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def frame(t: int, data: bytes = b"") -> bytes:
    raw = bytes([t]) + data
    return cobs_encode(raw + bytes([crc8(raw)])) + b"\x00"


def parse(enc: bytes):
    raw = cobs_decode(enc)
    if len(raw) < 2 or crc8(raw[:-1]) != raw[-1]:
        raise ValueError("bad crc")
    return raw[0], raw[1:-1]


def write_status(**kw):
    """/run/loglady/status.json for Kontrolleur (via the export): ident, teletype, grid_line, t."""
    try:
        tmp = os.path.join(RUN, "status.json.new")
        with open(tmp, "w") as f:
            json.dump(dict(kw, t=time.time()), f)
        os.replace(tmp, os.path.join(RUN, "status.json"))
    except OSError:
        pass


def make_fifo(path):
    if os.path.exists(path) and not stat.S_ISFIFO(os.stat(path).st_mode):
        os.unlink(path)
    if not os.path.exists(path):
        os.mkfifo(path, 0o660)
    return os.open(path, os.O_RDWR | os.O_NONBLOCK)   # RDWR: never EOF, writers never get ENXIO


def setup_pty():
    master, slave = os.openpty()
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    os.set_blocking(master, False)
    link = os.path.join(RUN, "grid")
    tmp = link + ".new"
    try:
        os.unlink(tmp)
    except FileNotFoundError:
        pass
    os.symlink(os.ttyname(slave), tmp)
    os.replace(tmp, link)
    return master, slave        # we keep the slave open: grid_macros can come and go


class Loglady:
    def __init__(self, dev, baud):
        import serial
        self.serial_mod = serial
        self.dev, self.baud = dev, baud
        self.ser = None
        self.ident = None
        self.rx = bytearray()
        self.last_hello = 0.0
        self.status = None
        self.bad = 0

    def open(self):
        while True:
            try:
                self.ser = self.serial_mod.Serial(self.dev, self.baud, timeout=0)
                self.ser.reset_input_buffer()
                log(f"✅ UART {self.dev} @ {self.baud}")
                return
            except Exception as e:
                log(f"⚠️  {self.dev}: {e}; retry in 2 s")
                time.sleep(2)

    def send(self, t, data=b""):
        if self.ser is None:
            return
        try:
            self.ser.write(frame(t, data))
        except Exception as e:
            log(f"⚠️  UART write: {e}")
            self.ser = None

    def frames(self):
        """Read what's there; yield (type, data) of complete, valid frames."""
        try:
            chunk = self.ser.read(4096)
        except Exception as e:
            log(f"⚠️  UART read: {e}")
            self.ser = None
            return
        self.rx += chunk
        while True:
            i = self.rx.find(0)
            if i < 0:
                if len(self.rx) > 4096:
                    self.rx.clear()
                return
            enc, self.rx = bytes(self.rx[:i]), self.rx[i + 1:]
            if not enc:
                continue
            try:
                yield parse(enc)
            except ValueError:
                self.bad += 1


class Pacer:
    """Token bucket for grid bytes towards Teletype: loglady passes USB data on at roughly half the
    UART rate, so long bursts would overflow her UART buffer. Small bursts (key presses, query answers)
    go straight through; long streams are held to RATE bytes/s."""
    RATE, BURST = 40000, 768

    def __init__(self):
        self.tokens, self.t = float(self.BURST), time.monotonic()

    def wait(self, n):
        now = time.monotonic()
        self.tokens = min(self.BURST, self.tokens + (now - self.t) * self.RATE)
        self.t = now
        if self.tokens < n:
            time.sleep((n - self.tokens) / self.RATE)
            self.tokens, self.t = float(n), time.monotonic()
        self.tokens -= n


def run(args):
    os.makedirs(RUN, exist_ok=True)
    grid_m, _grid_s = setup_pty()
    kbd = make_fifo(os.path.join(RUN, "kbd"))
    ctl = make_fifo(os.path.join(RUN, "ctl"))
    log(f"grid: {os.path.join(RUN, 'grid')} -> {os.readlink(os.path.join(RUN, 'grid'))}")
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from gpio_gate import start_gates          # no back-powering while Teletype is off
        start_gates([(args.dev, "loglady")], log)
    except Exception as e:
        log(f"⚠️  TX gating unavailable: {e}")
    ll = Loglady(args.dev, args.baud)
    ll.open()
    kbd_pending = b""
    pace = Pacer()
    replug_after_ident = True
    last_ping = 0.0

    while True:
        if ll.ser is None:
            ll.ident = None
            time.sleep(1)
            ll.open()
        now = time.monotonic()
        if ll.ident is None and now - ll.last_hello > 1.0:
            ll.send(T_HELLO)
            ll.last_hello = now
        if ll.ident is not None and now - last_ping > 5.0:
            ll.send(T_PING)
            last_ping = now

        r, _, _ = select.select([ll.ser.fileno(), grid_m, kbd, ctl], [], [], 0.2)

        if ll.ser.fileno() in r:
            for t, d in ll.frames():
                if t == T_IDENT:
                    if d.startswith(IDENT_PREFIX):
                        if ll.ident != d:
                            log(f"🪵 {d.decode(errors='replace')} on {args.dev}")
                        ll.ident = d
                        if replug_after_ident:
                            replug_after_ident = False
                            ll.send(T_REPLUG)
                            log("🔌 replug: Teletype re-reads the grid with the Pi listening")
                    else:
                        log(f"⛔ {args.dev}: '{d.decode(errors='replace')}' is not loglady — ignored")
                        ll.ident = None
                elif ll.ident is None:
                    continue                      # exclusive: nothing before loglady identified
                elif t == T_GRID_IN:
                    try:
                        os.write(grid_m, d)
                    except BlockingIOError:
                        pass                      # nobody reads the grid side: drop
                elif t == T_STATUS and len(d) >= 8:
                    st = (d[0], d[1])
                    if st != ll.status:
                        log(f"ℹ️  Teletype USB: {'connected' if d[0] else 'not connected'}"
                            f", grid line {'open' if d[1] else 'closed'}"
                            f" (bad frames {d[4] | d[5] << 8}, keys dropped {d[6] | d[7] << 8})")
                        ll.status = st
                        write_status(ident=(ll.ident or b"").decode(errors="replace"), teletype=bool(d[0]),
                                     grid_line=bool(d[1]))

        if grid_m in r:
            try:
                data = os.read(grid_m, 4096)
            except (BlockingIOError, OSError):
                data = b""
            if data and ll.ident is not None:
                for i in range(0, len(data), 240):
                    pace.wait(len(data[i:i + 240]))
                    ll.send(T_GRID_OUT, data[i:i + 240])

        if kbd in r:
            try:
                kbd_pending += os.read(kbd, 4096)
            except BlockingIOError:
                pass
            while len(kbd_pending) >= 8:
                rep, kbd_pending = kbd_pending[:8], kbd_pending[8:]
                if ll.ident is not None:
                    ll.send(T_KBD, rep)

        if ctl in r:
            try:
                cmds = os.read(ctl, 4096).decode(errors="ignore").split()
            except BlockingIOError:
                cmds = []
            for c in cmds:
                if c == "replug":
                    if ll.ident is not None:
                        ll.send(T_REPLUG)
                        log("🔌 replug (requested)")
                    else:
                        replug_after_ident = True
                elif c == "status":
                    log(f"status: ident={ll.ident}, teletype={ll.status}, bad frames from loglady={ll.bad}")
                    ll.send(T_PING)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttyAMA5")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--replug", action="store_true", help="ask the running service to replug loglady")
    args = ap.parse_args(argv)
    if args.replug:
        fd = os.open(os.path.join(RUN, "ctl"), os.O_WRONLY | os.O_NONBLOCK)
        os.write(fd, b"replug\n")
        os.close(fd)
        return
    run(args)


if __name__ == "__main__":
    main()
