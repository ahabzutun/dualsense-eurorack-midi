#!/usr/bin/env python3
"""
Read and write the setups of a Faderfox EC4 (used by DualSense Mapper).

The EC4 can't be asked for its data: transfers are started on the device.
  receive: on the EC4 press Func → Setup → Send → "Send all setups" (hold the knob)
  send:    on the EC4 press Func → Setup → Receive ("Work in progress"), then send
Format as in the Faderfox web editor (github.com/privatepublic-de/faderfox-editor, MIT):
one SysEx  F0 00 00 00  41 <dev=0x0B>  42 <3>  43 <ver hi>  44 <ver lo>
  then per 64-byte page:  49 <addr hi> 4A <addr lo> 4D <byte> ×64  4B <crc hi> 4C <crc lo>  (30 × 00)
  4F <dev> F7         — every value as two bytes (0x20 | high nibble, 0x10 | low nibble)
Memory 0x0B00–0xFFFF: 16 setups × 16 groups × 16 encoders.

Usage:
    ec4_tool.py receive [SECONDS]      wait for "Send all setups", print JSON (with the raw memory)
    ec4_tool.py send < x.json          apply the JSON to its raw memory and send everything
    ec4_tool.py parse FILE.syx         .syx → JSON
    ec4_tool.py build FILE.syx < x.json   JSON → .syx (nothing is sent)
JSON: {ok, version, mem (base64), setups: [{name, groups: [{name, encoders: [...]}]}]}
"""
import base64, json, sys, time

MEMORY_SIZE, MEMORY_OFFSET = 0xF500, 0x0B00
DEVICE_ID = 0x0B
ADDR_KEY1 = 0x0B00 - MEMORY_OFFSET      # push-button number + mode (16 bytes per group)
ADDR_SETUP_NAMES = 0x1BC0 - MEMORY_OFFSET
ADDR_GROUP_NAMES = 0x1C00 - MEMORY_OFFSET
ADDR_PRESETS = 0x2000 - MEMORY_OFFSET   # 192 bytes per group
ADDR_KEY2 = 0xE000 - MEMORY_OFFSET      # push-button lower/upper/display/link (32 bytes per group)
FILE_SIZE = 229340

TYPES = ['cc_rel1', 'cc_rel2', 'cc', 'pc', 'cc14', 'pitchbend', 'aftertouch', 'note', 'nrpn']
MODES = ['div8', 'div4', 'div2', 'none', 'low', 'middle', 'max', 'step2', 'step4', 'step6']
SCALES = ['off', '0-127', '0-100', '0-1000', '-63..63', '-50..50', '-500..500', 'on/off', '0-9999']
PB_TYPES = ['off', 'note', 'cc', 'pc', 'pitchbend', 'aftertouch', 'group', 'setup', 'acc0', 'acc3', 'lsp6', 'min', 'max']
PB_MODES = ['key', 'toggle']


def hilo(v):
    return [0x20 | ((v >> 4) & 0x0F), 0x10 | (v & 0x0F)]


# ── SysEx <-> memory ───────────────────────────────────────────────────────────

def parse_syx(data):
    if len(data) < 8 or data[0] != 0xF0 or any(data[1:4]):
        raise ValueError('not a Faderfox SysEx dump')
    mem = bytearray(MEMORY_SIZE)
    ix, version, page, buf, crc, pages = 4, 0.0, 0, [], 0, 0
    end = len(data) - 1
    while True:
        while ix < end and data[ix] == 0:
            ix += 1
        if ix + 2 >= len(data):
            raise ValueError('dump is incomplete (no end marker)')
        cmd, val = data[ix], ((data[ix + 1] & 0x0F) << 4) | (data[ix + 2] & 0x0F)
        ix += 3
        if cmd == 0x41:
            if val != DEVICE_ID:
                raise ValueError(f'dump is from another Faderfox device (id {val:#x})')
        elif cmd == 0x42:
            if val != 3:
                raise ValueError(f'unexpected dump type {val}')
        elif cmd == 0x43:
            version += val
        elif cmd == 0x44:
            version += val / 10
        elif cmd == 0x49:
            page = val << 8
        elif cmd == 0x4A:
            page |= val
        elif cmd == 0x4D:
            buf.append(val); crc += val
        elif cmd == 0x4B:
            crc_in = val << 8
        elif cmd == 0x4C:
            if (crc_in | val) != (crc & 0xFFFF):
                raise ValueError(f'checksum error in block {page:#06x}')
            a = page - MEMORY_OFFSET
            if 0 <= a and a + len(buf) <= MEMORY_SIZE:
                mem[a:a + len(buf)] = bytes(buf)
            buf, crc, pages = [], 0, pages + 1
        elif cmd == 0x4F:
            break
    if version < 2:
        raise ValueError('this EC4 runs firmware 1.x; update it to 2.x first')
    return mem, version, pages


def build_syx(mem):
    out = [0xF0, 0, 0, 0, 0x41, *hilo(DEVICE_ID), 0x42, 0x20, 0x13, 0x43, 0x20, 0x12, 0x44, 0x20, 0x14]
    for p in range(MEMORY_SIZE // 64):
        pos = p * 64
        addr = pos + MEMORY_OFFSET
        out += [0x49, *hilo(addr >> 8), 0x4A, *hilo(addr & 0xFF)]
        for b in mem[pos:pos + 64]:
            out += [0x4D, *hilo(b)]
        crc = sum(mem[pos:pos + 64]) & 0xFFFF
        out += [0x4B, *hilo(crc >> 8), 0x4C, *hilo(crc & 0xFF)] + [0] * 30
    out += [0x4F, *hilo(DEVICE_ID), 0xF7]
    return bytes(out)


# ── memory <-> JSON ───────────────────────────────────────────────────────────

def _grp(s, g, length, start=ADDR_PRESETS):
    return start + (s * 16 + g) * length


def _name(mem, a):
    return bytes(mem[a:a + 4]).decode('latin-1').rstrip('\x00')


def _set_name(mem, a, name):
    b = (name or '')[:4].ljust(4).encode('latin-1', 'replace')
    mem[a:a + 4] = b


def decode(mem):
    setups = []
    for s in range(16):
        groups = []
        for g in range(16):
            base, k1, k2 = _grp(s, g, 192), _grp(s, g, 16, ADDR_KEY1), _grp(s, g, 32, ADDR_KEY2)
            encs = []
            for e in range(16):
                b0, b16, b32 = mem[base + e], mem[base + 16 + e], mem[base + 32 + e]
                lo, hi, b80, b96 = mem[base + 48 + e], mem[base + 64 + e], mem[base + 80 + e], mem[base + 96 + e]
                b112 = mem[base + 112 + e]
                t = b0 >> 4
                t = t if t < len(TYPES) else 2
                m = b80 >> 4
                m = m if m < len(MODES) else 3
                sc = b80 & 0x0F
                sc = sc if sc < len(SCALES) else 1
                highres = t in (4, 5, 8) and sc in (0, 3, 6, 8)
                p0, p1, p2 = mem[k1 + e], mem[k2 + e], mem[k2 + 16 + e]
                pt = b112 >> 4
                encs.append(dict(
                    name=_name(mem, base + 128 + e * 4),
                    type=TYPES[t], channel=(b0 & 0x0F) + 1, number=b16 & 0x7F, number_h=b32,
                    lower=lo | ((b96 & 0x0F) << 8 if highres else 0),
                    upper=hi | ((b96 >> 4) << 8 if highres else 0),
                    mode=MODES[m], scale=SCALES[sc], link=bool(b16 & 0x80),
                    push=dict(type=PB_TYPES[pt] if pt < len(PB_TYPES) else 'off', channel=(b112 & 0x0F) + 1,
                              number=p0 & 0x7F, mode=PB_MODES[p0 >> 7], display=bool(p1 & 0x80),
                              lower=p1 & 0x7F, upper=p2 & 0x7F, link=bool(p2 & 0x80))))
            groups.append(dict(name=_name(mem, ADDR_GROUP_NAMES + s * 64 + g * 4), encoders=encs))
        setups.append(dict(name=_name(mem, ADDR_SETUP_NAMES + s * 4), groups=groups))
    return setups


def encode(setups, mem):
    """Write JSON values into a copy of mem; fields that aren't given stay as they are."""
    mem = bytearray(mem)
    def idx(lst, v, default):
        return lst.index(v) if v in lst else default
    for s, su in enumerate(setups[:16]):
        if 'name' in su:
            _set_name(mem, ADDR_SETUP_NAMES + s * 4, su['name'])
        for g, gr in enumerate(su.get('groups', [])[:16]):
            if 'name' in gr:
                _set_name(mem, ADDR_GROUP_NAMES + s * 64 + g * 4, gr['name'])
            base, k1, k2 = _grp(s, g, 192), _grp(s, g, 16, ADDR_KEY1), _grp(s, g, 32, ADDR_KEY2)
            for e, en in enumerate(gr.get('encoders', [])[:16]):
                t = idx(TYPES, en.get('type'), mem[base + e] >> 4)
                ch = max(1, min(16, int(en.get('channel', (mem[base + e] & 15) + 1)))) - 1
                mem[base + e] = (t << 4) | ch
                num = max(0, min(31 if t == 4 else 127, int(en.get('number', mem[base + 16 + e] & 0x7F))))
                link = en.get('link', bool(mem[base + 16 + e] & 0x80))
                mem[base + 16 + e] = num | (0x80 if link else 0)
                if 'number_h' in en:
                    mem[base + 32 + e] = max(0, min(127, int(en['number_h'])))
                m = idx(MODES, en.get('mode'), mem[base + 80 + e] >> 4)
                sc = idx(SCALES, en.get('scale'), mem[base + 80 + e] & 15)
                mem[base + 80 + e] = (m << 4) | sc
                highres = t in (4, 5, 8) and sc in (0, 3, 6, 8)
                lo = int(en.get('lower', mem[base + 48 + e])); hi = int(en.get('upper', mem[base + 64 + e]))
                if highres:
                    lo, hi = max(0, min(4095, lo)), max(0, min(4095, hi))
                    mem[base + 96 + e] = ((hi >> 8) << 4) | (lo >> 8)
                else:
                    lo, hi = max(0, min(255, lo)), max(0, min(255, hi))
                mem[base + 48 + e], mem[base + 64 + e] = lo & 0xFF, hi & 0xFF
                if 'name' in en:
                    _set_name(mem, base + 128 + e * 4, en['name'])
                pb = en.get('push')
                if pb:
                    pt = idx(PB_TYPES, pb.get('type'), mem[base + 112 + e] >> 4)
                    pch = max(1, min(16, int(pb.get('channel', (mem[base + 112 + e] & 15) + 1)))) - 1
                    mem[base + 112 + e] = (pt << 4) | pch
                    pm = idx(PB_MODES, pb.get('mode'), mem[k1 + e] >> 7)
                    mem[k1 + e] = (pm << 7) | max(0, min(127, int(pb.get('number', mem[k1 + e] & 0x7F))))
                    disp = pb.get('display', bool(mem[k2 + e] & 0x80))
                    mem[k2 + e] = (0x80 if disp else 0) | max(0, min(127, int(pb.get('lower', mem[k2 + e] & 0x7F))))
                    plink = pb.get('link', bool(mem[k2 + 16 + e] & 0x80)) and pt <= 4
                    mem[k2 + 16 + e] = (0x80 if plink else 0) | max(0, min(127, int(pb.get('upper', mem[k2 + 16 + e] & 0x7F))))
    return mem


# ── MIDI ──────────────────────────────────────────────────────────────────────

def _port(lst):
    for i, p in enumerate(lst):
        if 'EC4' in p:
            return i, p
    raise RuntimeError('no Faderfox EC4 found (is it plugged in?)')


def receive(seconds):
    import rtmidi
    mi = rtmidi.MidiIn()
    mi.ignore_types(sysex=False, timing=True, active_sense=True)
    n, name = _port(mi.get_ports())
    mi.open_port(n)
    buf, end, last = bytearray(), time.time() + seconds, 0.0
    try:
        while time.time() < end:
            m = mi.get_message()
            if m:
                d = m[0]
                if (d and d[0] == 0xF0) or buf:
                    buf += bytes(d); last = time.time()
                    if buf[-1] == 0xF7 and len(buf) > 1000:
                        break
            else:
                if buf and time.time() - last > 3:
                    break
                time.sleep(0.002)
    finally:
        mi.close_port()
    if not buf:
        raise RuntimeError(f'nothing received within {seconds} s. On the EC4: Func → Setup → Send → "Send all setups" (hold the knob).')
    return bytes(buf), name


def _alsa_rawmidi():
    """/dev/snd/midiC<card>D0 of the EC4 (Linux)."""
    import re
    try:
        cards = open('/proc/asound/cards').read()
    except OSError:
        return None
    for line in cards.splitlines():
        m = re.match(r'\s*(\d+)\s+\[(.*?)\]\s*:\s*(.*)', line)
        if m and ('EC4' in m.group(2) or 'EC4' in m.group(3)):
            return f'/dev/snd/midiC{m.group(1)}D0', f'hw:{m.group(1)},0 ({m.group(2).strip()})'
    return None


def send(syx):
    """Send the whole dump as one SysEx."""
    if sys.platform == 'darwin':
        import rtmidi
        mo = rtmidi.MidiOut()
        n, name = _port(mo.get_ports())
        mo.open_port(n)
        try:
            mo.send_message(list(syx))           # CoreMIDI splits and paces long SysEx itself
            time.sleep(len(syx) / 3000 + 2)
        finally:
            mo.close_port()
        return name
    # Linux: the ALSA sequencer refuses a 229 KB event, so write to the raw MIDI device.
    # Writes block while the USB endpoint is busy, which paces the transfer for the EC4.
    found = _alsa_rawmidi()
    if not found:
        raise RuntimeError('no Faderfox EC4 found (is it plugged in?)')
    path, name = found
    import os
    try:
        fd = os.open(path, os.O_WRONLY)
    except OSError as e:
        raise RuntimeError(f"can't open {name}: {e.strerror}")
    try:
        view = memoryview(syx)
        for i in range(0, len(syx), 256):
            chunk = view[i:i + 256]
            while chunk:
                k = os.write(fd, chunk)
                chunk = chunk[k:]
        time.sleep(2)                            # let the last blocks leave before closing
    finally:
        os.close(fd)                             # closing drains the rest of the output buffer
    return name


def out(obj):
    print(json.dumps(obj))


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'receive'
    try:
        if cmd == 'receive':
            secs = float(sys.argv[2]) if len(sys.argv) > 2 else 120
            data, port = receive(secs)
            mem, version, pages = parse_syx(data)
            out(dict(ok=True, port=port, version=version, pages=pages,
                     mem=base64.b64encode(bytes(mem)).decode(), setups=decode(mem)))
        elif cmd == 'parse':
            mem, version, pages = parse_syx(open(sys.argv[2], 'rb').read())
            out(dict(ok=True, version=version, pages=pages, mem=base64.b64encode(bytes(mem)).decode(),
                     setups=decode(mem)))
        elif cmd in ('send', 'build'):
            j = json.load(sys.stdin)
            base = base64.b64decode(j['mem'])
            if len(base) != MEMORY_SIZE:
                raise ValueError('the JSON has no complete EC4 memory (receive first)')
            mem = encode(j.get('setups', []), base)
            syx = build_syx(mem)
            check, _, _ = parse_syx(syx)
            if bytes(check) != bytes(mem):
                raise RuntimeError('internal check failed: the built dump does not decode to the same data')
            changed = sum(1 for a, b in zip(mem, base) if a != b)
            if cmd == 'build':
                open(sys.argv[2], 'wb').write(syx)
                out(dict(ok=True, file=sys.argv[2], bytes=len(syx), changed_bytes=changed))
            else:
                port = send(syx)
                out(dict(ok=True, port=port, bytes=len(syx), changed_bytes=changed))
        else:
            raise SystemExit(__doc__)
    except Exception as e:
        out(dict(ok=False, error=str(e)))


if __name__ == '__main__':
    main()
