"""
The bit of the python-rtmidi API the device tools use, on top of alsa-midi (Linux).

python-rtmidi has no builds for newer Pythons (e.g. Fedora 44's Python 3.14) and Fedora no longer
packages it; alsa-midi is maintained and has wheels. The tools do

    try:
        import rtmidi
    except ImportError:
        import rtmidi_compat as rtmidi

Only SysEx is delivered (the tools ignore everything else); chunked SysEx from the ALSA sequencer is
put back together. Port names look like python-rtmidi's on Linux: "16n:16n MIDI 1 24:0".
"""
import threading

from alsa_midi import PortCaps, PortType, SequencerClient, SysExEvent


def _name(p):
    return f"{p.client_name}:{p.name} {p.client_id}:{p.port_id}"


class _Base:
    _caps = PortCaps.READ | PortCaps.WRITE | PortCaps.SUBS_READ | PortCaps.SUBS_WRITE

    def __init__(self, name="kontrolleur-tool"):
        self.client = SequencerClient(name)
        self.port = self.client.create_port("io", caps=self._caps,
                                            type=PortType.MIDI_GENERIC | PortType.APPLICATION)
        self._target = None

    def _list(self):
        raise NotImplementedError

    def get_ports(self):
        return [_name(p) for p in self._list()]

    def close_port(self):
        try:
            self.client.close()
        except Exception:
            pass

    def __del__(self):
        self.close_port()


class MidiOut(_Base):
    def _list(self):
        return self.client.list_ports(output=True)

    def open_port(self, n):
        self._target = self._list()[n]
        self.port.connect_to(self._target)

    def send_message(self, msg):
        data = bytes(msg)
        # the sequencer takes SysEx in pieces; 256 bytes keeps every event small
        for i in range(0, len(data), 256):
            self.client.event_output(SysExEvent(data[i:i + 256]), port=self.port, dest=self._target)
        self.client.drain_output()


class MidiIn(_Base):
    def __init__(self, name="kontrolleur-tool"):
        super().__init__(name)
        self._queue = []
        self._buf = bytearray()
        self._callback = None
        self._thread = None
        self._stop = False

    def _list(self):
        return self.client.list_ports(input=True)

    def ignore_types(self, sysex=True, timing=True, active_sense=True):
        pass                                     # only SysEx is delivered anyway

    def open_port(self, n):
        self.port.connect_from(self._list()[n])

    def _poll(self, timeout):
        ev = self.client.event_input(timeout=timeout)
        if isinstance(ev, SysExEvent):
            d = bytes(ev.data)
            if d[:1] == b"\xF0":
                self._buf = bytearray()
            self._buf += d
            if self._buf[-1:] == b"\xF7":
                msg = (list(self._buf), 0.0)
                self._buf = bytearray()
                return msg
        return None

    def get_message(self):
        m = self._poll(0)
        return m

    def set_callback(self, fn, data=None):
        self._callback = fn

        def loop():
            while not self._stop:
                try:
                    m = self._poll(0.05)
                except Exception:
                    break
                if m is not None and self._callback:
                    self._callback(m, data)

        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def close_port(self):
        self._stop = True
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=0.5)
        super().close_port()
