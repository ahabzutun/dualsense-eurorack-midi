"""TX gating for the XIAO UART links: never feed an unpowered XIAO through its RX pin.

A Pi UART TX line idles high (3.3 V). Into an unpowered XIAO that current flows through the SAMD21's
protection diodes into its supply: the XIAO sits half-powered (faint green LED) and doesn't get a clean
power-on reset when its USB host powers it later — it doesn't boot.

So each link watches its RX line: a powered XIAO holds its TX (= our RX) high when idle. With a
pull-down on our RX, an unpowered XIAO reads low. Low for longer than OFF_AFTER → our TX pin becomes an
input with pull-down (no current into the XIAO). High again → TX back to its UART function.
Pin levels are read from the GPIO registers (/dev/gpiomem), which doesn't disturb the UART.
BCM2711 (Pi 4) register layout.
"""
import mmap, os, struct, threading, time

GPFSEL0, GPLEV0, PUPPDN0 = 0x00, 0x34, 0xE4
F_INPUT, F_ALT0, F_ALT4 = 0b000, 0b100, 0b011
PULL_NONE, PULL_UP, PULL_DOWN = 0, 1, 2

# device -> (tx gpio, rx gpio, uart function)
UART_PINS = {
    "/dev/ttyAMA0": (14, 15, F_ALT0),
    "/dev/ttyAMA2": (0, 1, F_ALT4),
    "/dev/ttyAMA3": (4, 5, F_ALT4),
    "/dev/ttyAMA4": (8, 9, F_ALT4),
    "/dev/ttyAMA5": (12, 13, F_ALT4),
}


class Gpio:
    _lock = threading.Lock()

    def __init__(self, path="/dev/gpiomem"):
        fd = os.open(path, os.O_RDWR | os.O_SYNC)
        try:
            self.m = mmap.mmap(fd, 4096, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        finally:
            os.close(fd)

    def _r(self, off):
        return struct.unpack_from("<I", self.m, off)[0]

    def _w(self, off, v):
        struct.pack_into("<I", self.m, off, v)

    def level(self, pin):
        return (self._r(GPLEV0) >> pin) & 1

    def func(self, pin):
        return (self._r(GPFSEL0 + (pin // 10) * 4) >> ((pin % 10) * 3)) & 7

    def set_func(self, pin, f):
        off, sh = GPFSEL0 + (pin // 10) * 4, (pin % 10) * 3
        with self._lock:
            self._w(off, (self._r(off) & ~(7 << sh)) | (f << sh))

    def set_pull(self, pin, p):
        off, sh = PUPPDN0 + (pin // 16) * 4, (pin % 16) * 2
        with self._lock:
            self._w(off, (self._r(off) & ~(3 << sh)) | (p << sh))


class TxGate:
    """Keeps one link's TX pin off while its XIAO is unpowered."""
    POLL, OFF_AFTER = 0.05, 0.3

    def __init__(self, gpio, device, name, log=print):
        self.gpio, self.name, self.log = gpio, name, log
        self.tx, self.rx, self.alt = UART_PINS[device]
        self.low_since = None
        self.enabled = None
        gpio.set_pull(self.rx, PULL_DOWN)       # unpowered XIAO reads low, and gets no current from us
        gpio.set_pull(self.tx, PULL_DOWN)       # when TX is an input, hold it low too
        self.alive = threading.Event()
        self.power_ons = []                     # monotonic times the XIAO came (back) up

    def poll(self, now):
        if self.gpio.level(self.rx):
            self.low_since = None
            want = True
        else:
            if self.low_since is None:
                self.low_since = now
            if now - self.low_since > self.OFF_AFTER or self.enabled is None:
                want = False                    # gone (or unknown at start): TX off
            else:
                want = self.enabled             # a short low (data bits): keep as is
        if want != self.enabled:
            self.gpio.set_func(self.tx, self.alt if want else F_INPUT)
            self.log(f"{'🟢' if want else '⚪'} [{self.name}] XIAO "
                     f"{'powered: TX on' if want else 'unpowered: TX off (no back-powering)'}")
            if want and self.enabled is not None:
                self.power_ons = [t for t in self.power_ons if now - t < 600] + [now]
            self.enabled = want
        if want:
            self.alive.set()
        else:
            self.alive.clear()


def start_gates(links, log=print):
    """links: iterable of (device, name). Starts one polling thread for all; returns the gates."""
    gpio = Gpio()
    gates = [TxGate(gpio, dev, name, log) for dev, name in links if dev in UART_PINS]

    def loop():
        while True:
            now = time.monotonic()
            for g in gates:
                try:
                    g.poll(now)
                except Exception as e:      # never let gating kill the bridge
                    log(f"⚠️  [{g.name}] gate: {e}")
            time.sleep(TxGate.POLL)

    threading.Thread(target=loop, daemon=True, name="tx-gates").start()
    return gates
