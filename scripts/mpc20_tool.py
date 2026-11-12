#!/usr/bin/env python3
"""
Read and write the settings of a DOREMiDi MPC-20 pedal-to-MIDI box (used by DualSense Mapper).

The MPC-20 is configured over its USB serial (CDC) interface, not over MIDI. Protocol
(worked out from the Pedal Config Tool V4.0.3; firmware V2.1.2):

    hello   F0 00 21 5E 01 01 F7                      -> F0 00 21 5E 01 04 F7 ...
    model   AA CC 02 00 01 0D 0A                      -> AA CC 06 'MPC-20' 0D 0A
    version AA CC 02 00 03 0D 0A                      -> AA CC 06 'V2.1.2' 0D 0A
    read    F0 00 21 5E 04 04 <input> <slot> F7       -> F0 00 21 5E 10 04 <16-byte block> 00 F7
    write   F0 00 21 5E 10 03 <16-byte block> 00 F7   -> F0 00 21 5E 04 03 <slot> 00 F7

Inputs 0-3 are the jacks EXP1-EXP4 (4-7: the second switch of a dual-switch pedal). Each input
has 8 message slots; all slots that are switched on are sent together. The pedal type is
detected by the box: an empty jack reads as type 0 with all zeros.

Block (16 bytes):
    0 input   1 slot   2 pedal type (0 none, 1 expression, 2 switch, 3 hi-hat)
    3 channel (0-15)   4 message kind (see KINDS)
    5 number (CC / note / program)   6 (pitch bend: max >> 7)
    7 value at the start of travel   8 value at the end (pitch bend: low 7 bits; 5: min >> 7)
    9 switch mode   10 gain (50)   11 trigger level   12 press note   13 release note
    14 'ql' (curve parameter)   15 curve preview value
A slot is off when number, start and end are all 0 (that's how the box stores unused slots).

Usage:
    mpc20_tool.py export            -> JSON on stdout
    mpc20_tool.py install < x.json  -> writes changed slots, reads back, JSON result
    mpc20_tool.py backup FILE       -> raw dump of all inputs and slots
    mpc20_tool.py restore FILE      -> writes a raw dump back
On macOS it uses the /dev/cu.usbmodem… port of the box; on Linux (the Pi), where the kernel's
cdc_acm driver can't bind this box, it talks to the CDC data interface directly with pyusb.
"""
import glob, json, os, struct, subprocess, sys, time

HDR = bytes([0xF0, 0x00, 0x21, 0x5E])
KINDS = {0: 'cc', 1: 'pc', 2: 'note', 3: 'pitchbend', 4: 'pressure', 5: 'start', 6: 'continue', 7: 'stop', 8: 'tempo'}
KIND_IDS = {v: k for k, v in KINDS.items()}
TYPES = {0: 'none', 1: 'expression', 2: 'switch', 3: 'hihat'}
INPUTS, SLOTS = 8, 8
VID, PID = 0x1A86, 0x752D


class SerialLink:
    """macOS / any OS where the box shows up as a serial port."""
    def __init__(self, path):
        import termios
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self.where = path

    def write(self, b):
        os.write(self.fd, b)

    def read(self):
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b''

    def close(self):
        os.close(self.fd)


class UsbLink:
    """Linux: claim the CDC data interface with pyusb (cdc_acm fails on this box's descriptors).
    The MIDI interface stays with snd-usb-audio, so the MIDI hub keeps working."""
    def __init__(self):
        import usb.core, usb.util
        self.usb, self.util = usb.core, usb.util
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise RuntimeError('no MPC-20 found on USB (is it plugged in / switched on?)')
        cfg = dev.get_active_configuration()
        comm = next(i for i in cfg if i.bInterfaceClass == 0x02)
        data = next(i for i in cfg if i.bInterfaceClass == 0x0A)
        self.dev, self.ifc, self.comm = dev, data.bInterfaceNumber, comm.bInterfaceNumber
        try:
            if dev.is_kernel_driver_active(self.ifc):
                dev.detach_kernel_driver(self.ifc)
        except Exception:
            pass
        usb.util.claim_interface(dev, self.ifc)
        self.ep_out = next(e for e in data if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        self.ep_in = next(e for e in data if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        # 115200 8N1 + DTR/RTS, like the Pedal Config Tool's serial port
        try:
            dev.ctrl_transfer(0x21, 0x20, 0, self.comm, struct.pack('<IBBB', 115200, 0, 0, 8))
            dev.ctrl_transfer(0x21, 0x22, 0x03, self.comm, None)
        except Exception:
            pass
        self.where = f'USB {dev.bus}-{dev.address}'

    def write(self, b):
        self.ep_out.write(b, timeout=500)

    def read(self):
        try:
            return bytes(self.ep_in.read(64, timeout=15))
        except self.usb.USBTimeoutError:
            return b''
        except self.usb.USBError as e:
            if getattr(e, 'errno', None) in (110, None) or 'timed out' in str(e).lower():
                return b''
            raise

    def close(self):
        try:
            self.util.release_interface(self.dev, self.ifc)
            self.util.dispose_resources(self.dev)
        except Exception:
            pass


def find_mac_port():
    """The callout device whose USB parent is a DOREMiDi box."""
    try:
        out = subprocess.run(['ioreg', '-r', '-c', 'IOUSBHostDevice', '-l', '-w0'],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:
        out = ''
    for block in out.split('+-o ')[1:]:
        if 'DOREMiDi' in block or 'MPC-' in block:
            for line in block.splitlines():
                if '"IOCalloutDevice"' in line:
                    return line.split('=', 1)[1].strip().strip('"')
    cands = glob.glob('/dev/cu.usbmodemCDAB*')
    return cands[0] if cands else None


def open_link():
    if sys.platform == 'darwin':
        p = find_mac_port()
        if not p:
            raise RuntimeError('no MPC-20 found (is it plugged into this Mac?)')
        return SerialLink(p)
    try:
        return UsbLink()
    except ImportError:
        raise RuntimeError('pyusb is missing (pip install pyusb)')


class MPC20:
    def __init__(self):
        self.link = open_link()
        self.buf = b''

    def close(self):
        self.link.close()

    def _xfer(self, msg, want, timeout=0.6):
        """Send msg, return the first reply that starts with want (live pedal MIDI is skipped)."""
        end = time.time() + 0.03
        while time.time() < end:        # drop anything pending
            self.link.read()
        self.buf = b''
        self.link.write(bytes(msg))
        end = time.time() + timeout
        while time.time() < end:
            chunk = self.link.read()
            if not chunk:
                time.sleep(0.003)
                continue
            self.buf += chunk
            i = self.buf.find(want)
            if i >= 0:
                tail = self.buf[i:]
                j = tail.find(b'\xF7') if want[0] == 0xF0 else tail.find(b'\x0D\x0A')
                if j >= 0:
                    return tail[:j + (1 if want[0] == 0xF0 else 2)]
        return None

    def hello(self):
        return self._xfer(HDR + bytes([1, 1, 0xF7]), HDR + bytes([1]))

    def text(self, code):
        r = self._xfer(bytes([0xAA, 0xCC, 0x02, 0x00, code, 0x0D, 0x0A]), bytes([0xAA, 0xCC]))
        return r[3:-2].decode('ascii', 'replace') if r else None

    def read_block(self, inp, slot):
        for _ in range(3):
            r = self._xfer(HDR + bytes([4, 4, inp, slot, 0xF7]), HDR + bytes([0x10, 4, inp, slot]))
            if r and len(r) >= 24:
                return list(r[6:22])
        raise RuntimeError(f'no answer reading input {inp + 1} slot {slot + 1}')

    def write_block(self, blk):
        assert len(blk) == 16 and all(0 <= b < 128 for b in blk), blk
        # the acknowledgement is F0 00 21 5E 04 03 <slot> 00 F7; a missed one is retried
        for _ in range(3):
            if self._xfer(HDR + bytes([0x10, 3] + list(blk) + [0x00, 0xF7]), HDR + bytes([4, 3]), 1.0):
                return
            if self.read_block(blk[0], blk[1]) == list(blk):
                return
        raise RuntimeError(f'no acknowledgement writing input {blk[0] + 1} slot {blk[1] + 1}')


# ── block <-> JSON ─────────────────────────────────────────────────────────────

def decode(blk):
    kind = KINDS.get(blk[4], f'kind{blk[4]}')
    s = dict(slot=blk[1], kind=kind, channel=blk[3] + 1, number=blk[5], start=blk[7], end=blk[8],
             mode=blk[9], gain=blk[10], ql=blk[14], raw=bytes(blk).hex())
    if kind == 'pitchbend':
        s['start'] = (blk[5] << 7) | blk[7]
        s['end'] = (blk[6] << 7) | blk[8]
        s['number'] = 0
        s['on'] = s['start'] != s['end']
    elif kind in ('start', 'continue', 'stop', 'tempo'):
        s['on'] = any(blk[5:9]) or blk[9] != 0
    else:
        s['on'] = bool(blk[5] or blk[7] or blk[8])
    return s


def encode(slot, inp, ptype):
    """JSON slot -> 16-byte block, starting from its raw bytes so unknown fields survive."""
    blk = list(bytes.fromhex(slot['raw'])) if slot.get('raw') else [0] * 16
    blk[0], blk[1], blk[2] = inp, slot['slot'], ptype
    kind = slot.get('kind', 'cc')
    if kind not in KIND_IDS:
        raise ValueError(f'unknown message kind {kind!r}')
    blk[3] = max(0, min(15, int(slot.get('channel', 1)) - 1))
    blk[4] = KIND_IDS[kind]
    if not slot.get('on', True):
        blk[5] = blk[6] = blk[7] = blk[8] = 0
        if kind == 'pitchbend' or kind in ('start', 'continue', 'stop', 'tempo'):
            blk[9] = 0
        return blk
    if kind == 'pitchbend':
        a, b = int(slot['start']), int(slot['end'])
        a, b = max(0, min(16383, a)), max(0, min(16383, b))
        blk[5], blk[6], blk[7], blk[8] = a >> 7, b >> 7, a & 0x7F, b & 0x7F
    else:
        blk[5] = max(0, min(127, int(slot.get('number', 0))))
        blk[6] = 0
        blk[7] = max(0, min(127, int(slot.get('start', 0))))
        blk[8] = max(0, min(127, int(slot.get('end', 127))))
    if 'mode' in slot:
        blk[9] = max(0, min(127, int(slot['mode'])))
    if 'gain' in slot:
        blk[10] = max(0, min(127, int(slot['gain'])))
    if 'ql' in slot:
        blk[14] = max(0, min(127, int(slot['ql'])))
    return blk


def export(box):
    inputs = []
    for i in range(INPUTS):
        blocks = [box.read_block(i, s) for s in range(SLOTS)]
        ptype = blocks[0][2]
        inputs.append(dict(input=i, type=TYPES.get(ptype, f'type{ptype}'), type_id=ptype,
                           slots=[decode(b) for b in blocks]))
    return inputs


def cmd_export():
    box = MPC20()
    try:
        box.hello()
        model, fw = box.text(1), box.text(3)
        print(json.dumps(dict(ok=True, device=model or 'MPC-20', firmware=fw or '?',
                              port=box.link.where, inputs=export(box))))
    finally:
        box.close()


def cmd_install():
    want = json.load(sys.stdin)
    box = MPC20()
    try:
        box.hello()
        written, skipped, mismatch = [], [], []
        for inp in want.get('inputs', []):
            i = int(inp['input'])
            cur = [box.read_block(i, s) for s in range(SLOTS)]
            ptype = cur[0][2]
            if ptype == 0:
                if any(s.get('changed') for s in inp.get('slots', [])):
                    skipped.append(f'EXP{i + 1}' if i < 4 else f'input {i + 1}')
                continue
            for s in inp.get('slots', []):
                blk = encode(s, i, ptype)
                if blk != cur[s['slot']]:
                    box.write_block(blk)
                    written.append((i, s['slot'], blk))
        for i, s, blk in written:
            back = box.read_block(i, s)
            if back != blk:
                mismatch.append(f'EXP{i + 1} message {s + 1}: wrote {bytes(blk).hex()} read {bytes(back).hex()}')
        state = export(box)
        print(json.dumps(dict(ok=not mismatch, verified=not mismatch, written=len(written),
                              skipped=skipped, mismatch=mismatch, inputs=state)))
    finally:
        box.close()


def cmd_backup(path):
    box = MPC20()
    try:
        box.hello()
        dump = {f'{i},{s}': bytes(box.read_block(i, s)).hex() for i in range(INPUTS) for s in range(SLOTS)}
        json.dump(dict(device=box.text(1), firmware=box.text(3), time=time.strftime('%Y-%m-%d %H:%M:%S'),
                       blocks=dump), open(path, 'w'), indent=1)
        print(json.dumps(dict(ok=True, file=path)))
    finally:
        box.close()


def cmd_restore(path):
    dump = json.load(open(path))['blocks']
    box = MPC20()
    try:
        box.hello()
        n = 0
        for key, hx in dump.items():
            blk = list(bytes.fromhex(hx))
            if blk[2] == 0:
                continue                      # nothing was plugged in there: nothing to restore
            if box.read_block(blk[0], blk[1]) != blk:
                box.write_block(blk); n += 1
        print(json.dumps(dict(ok=True, written=n)))
    finally:
        box.close()


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'export'
    try:
        if cmd == 'export':
            cmd_export()
        elif cmd == 'install':
            cmd_install()
        elif cmd == 'backup':
            cmd_backup(sys.argv[2])
        elif cmd == 'restore':
            cmd_restore(sys.argv[2])
        else:
            raise SystemExit(__doc__)
    except Exception as e:
        print(json.dumps(dict(ok=False, error=str(e))))
        sys.exit(0)


if __name__ == '__main__':
    main()
