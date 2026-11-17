"""
Pickup inside the MIDI hub (midi_passthrough.py).

The grid macros tell the hub over UDP (127.0.0.1:47121) which controls have to pick up a value first:
    {"source": "16n", "pickup": [[channel, cc, value], …]}
From then on, CCs from an input whose name contains the source are held back until the fader passes the
parameter's value. The value follows the macros' own messages (port Grid_Macros), so a fader can also
catch a value that is still gliding. rtmidi calls the hub's callbacks from several threads, hence the lock.
"""
import json
import socket
import threading

from .pickup import Pickup

PORT = 47121
MACRO_PORT_NAME = "Grid_Macros"


class HubPickup:
    def __init__(self, port=PORT, window=2, start_thread=True):
        self.pickup = Pickup(window)
        self.sources = set()
        self.lock = threading.Lock()
        self.sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", port))
            self.sock = s
        except OSError as e:
            print(f"⚠️  Pickup socket not available ({e}); 16n faders won't pick up after a macro")
        if start_thread and self.sock is not None:
            threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
            except OSError:
                return
            self.handle(data)

    def handle(self, data: bytes):
        try:
            req = json.loads(data.decode())
            src = str(req.get("source") or "")
            entries = req.get("pickup") or []
        except (ValueError, AttributeError):
            return
        with self.lock:
            if src:
                self.sources.add(src.lower())
            for e in entries:
                try:
                    ch, cc, value = int(e[0]), int(e[1]), int(e[2])
                except (TypeError, ValueError, IndexError):
                    continue
                self.pickup.arm(("cc", ch, cc), value)

    def allow(self, port_name: str, msg) -> bool:
        """False: hold this message back (a fader that hasn't picked up its value yet)."""
        if len(msg) < 3 or (msg[0] & 0xF0) != 0xB0:
            return True
        name = port_name.lower()
        with self.lock:
            if not any(s in name for s in self.sources):
                return True
            return self.pickup.accept(("cc", (msg[0] & 0x0F) + 1, msg[1]), msg[2])

    def note_macro(self, msg):
        """A message from the macros' own port: the parameter's value moved."""
        if len(msg) >= 3 and (msg[0] & 0xF0) == 0xB0:
            with self.lock:
                self.pickup.update_target(("cc", (msg[0] & 0x0F) + 1, msg[1]), msg[2])
