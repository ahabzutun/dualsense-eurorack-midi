"""
Builds the i2c2midi ER-301 routing SysEx from config/er301_routes.py.

Protocol (see er301_direct.h in the i2c2midi fork):
  F0 7D 45 33 <cmd> <payload> F7
  01 BEGIN · 10 CC · 11 NRPN · 12 GATE · 13 NOTEGATE · 20 PITCH · 21 LFO · 22 OPTIONS
  7F COMMIT <count MSB> <count LSB>        reply: 7E kind status source nCc nNrpn nGate nNote present
"""
import importlib.util
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES_PATH = os.path.join(HERE, "config", "er301_routes.py")
MAPPINGS_PATH = os.path.join(HERE, "config", "mappings.py")

HDR = [0xF0, 0x7D, 0x45, 0x33]
LIMITS = {"CV": 32, "NRPN_CV": 8, "TRIGGERS": 16, "NOTE_TRIGGERS": 16}
STATUS = {0: "ok", 1: "record count mismatch", 2: "no BEGIN received", 3: "invalid routing"}
SOURCE = {0: "firmware defaults", 1: "EEPROM", 2: "received from the Pi"}


class ConfigError(Exception):
    pass


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _resolve(ctl, names, what, maximum):
    if isinstance(ctl, str):
        if ctl not in names:
            raise ConfigError(f"unknown {what} '{ctl}' (known: {', '.join(sorted(names))})")
        num = names[ctl]
    elif isinstance(ctl, int):
        num = ctl
    else:
        raise ConfigError(f"{what} must be a name or a number, got {ctl!r}")
    if not 0 <= num <= maximum:
        raise ConfigError(f"{what} {num} out of range 0–{maximum}")
    return num


def _check(cond, msg):
    if not cond:
        raise ConfigError(msg)


def _routes(cfg, key, default_ch, names, what, maximum):
    out = []
    entries = getattr(cfg, key, []) or []
    _check(len(entries) <= LIMITS[key], f"{key}: at most {LIMITS[key]} entries")
    for e in entries:
        _check(isinstance(e, (tuple, list)) and len(e) in (2, 3),
               f"{key}: entry {e!r} must be (control, port) or (channel, control, port)")
        ch, ctl, port = (default_ch, *e) if len(e) == 2 else e
        _check(isinstance(ch, int) and 1 <= ch <= 16, f"{key}: channel {ch!r} must be 1–16")
        _check(isinstance(port, int) and 1 <= port <= 100, f"{key}: port {port!r} must be 1–100")
        out.append((ch, _resolve(ctl, names, what, maximum), port))
    return out


def build(routes_path=ROUTES_PATH, mappings_path=MAPPINGS_PATH):
    """Return (list of SysEx messages, one-line summary). Raises ConfigError."""
    try:
        cfg = _load(routes_path, "er301_routes_live")
    except ConfigError:
        raise
    except Exception as e:
        raise ConfigError(f"{os.path.basename(routes_path)}: {type(e).__name__}: {e}")
    try:
        maps = _load(mappings_path, "er301_mappings_live")
        cc_names = dict(getattr(maps, "CC_MAP", {}))
        nrpn_names = dict(getattr(maps, "NRPN_MAP", {}))
    except Exception:
        cc_names, nrpn_names = {}, {}

    ch = getattr(cfg, "CHANNEL", 2)
    _check(isinstance(ch, int) and 1 <= ch <= 16, f"CHANNEL {ch!r} must be 1–16")

    cv = _routes(cfg, "CV", ch, cc_names, "control", 127)
    nrpn = _routes(cfg, "NRPN_CV", ch, nrpn_names, "NRPN", 16383)
    gates = _routes(cfg, "TRIGGERS", ch, cc_names, "control", 127)
    notes = _routes(cfg, "NOTE_TRIGGERS", ch, {}, "note", 127)

    msgs = [HDR + [0x01, 0xF7]]
    records = []
    for c, n, p in cv:
        records.append([0x10, c, n, p])
    for c, n, p in nrpn:
        records.append([0x11, c, n >> 7, n & 0x7F, p])
    for c, n, p in gates:
        records.append([0x12, c, n, p])
    for c, n, p in notes:
        records.append([0x13, c, n, p])

    pitch = getattr(cfg, "PITCH", None)
    if pitch:
        port = pitch.get("port")
        _check(isinstance(port, int) and 1 <= port <= 100, f"PITCH port {port!r} must be 1–100")
        zero = pitch.get("zero_note", 60)
        _check(isinstance(zero, int) and 0 <= zero <= 127, "PITCH zero_note must be 0–127")
        octc = pitch.get("octave_control")
        octn = 0 if octc is None else _resolve(octc, cc_names, "control", 127)
        _check(octc is None or octn != 0, "PITCH octave_control cannot be CC 0 (0 means none)")
        semis = pitch.get("semitones_per_step", 12)
        _check(isinstance(semis, int) and 0 <= semis <= 127, "PITCH semitones_per_step must be 0–127")
        records.append([0x20, ch, port, zero, octn, semis])
    else:
        records.append([0x20, 0, 0, 0, 0, 0])

    lfo = getattr(cfg, "LFO", None)
    if lfo:
        port = lfo.get("port")
        _check(isinstance(port, int) and 1 <= port <= 100, f"LFO port {port!r} must be 1–100")
        rate = _resolve(lfo.get("rate_control"), cc_names, "control", 127)
        rst = lfo.get("reset_control")
        rstn = 0 if rst is None else _resolve(rst, cc_names, "control", 127)
        _check(rst is None or rstn != 0, "LFO reset_control cannot be CC 0 (0 means none)")
        lo = int(round(float(lfo.get("min_hz", 0.25)) * 100))
        hi = int(round(float(lfo.get("max_hz", 40.0)) * 100))
        _check(1 <= lo <= hi <= 16383, "LFO needs 0.01 ≤ min_hz ≤ max_hz ≤ 163.83")
        pulse = lfo.get("pulse_ms", 5)
        _check(isinstance(pulse, int) and 1 <= pulse <= 127, "LFO pulse_ms must be 1–127")
        records.append([0x21, ch, rate, rstn, port, lo >> 7, lo & 0x7F, hi >> 7, hi & 0x7F, pulse])
    else:
        records.append([0x21, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    records.append([0x22, 1 if getattr(cfg, "TRIGGERS_AS_CV", True) else 0])

    msgs += [HDR + r + [0xF7] for r in records]
    n = len(records)
    msgs.append(HDR + [0x7F, n >> 7, n & 0x7F, 0xF7])
    summary = (f"{len(cv)} CV, {len(nrpn)} NRPN, {len(gates)} trigger, {len(notes)} note trigger, "
               f"pitch {'→ ' + str(pitch['port']) if pitch else 'off'}, "
               f"LFO {'→ ' + str(lfo['port']) if lfo else 'off'}, "
               f"triggers as {'SC.CV' if getattr(cfg, 'TRIGGERS_AS_CV', True) else 'SC.TR'}")
    return msgs, summary


def query_message():
    return HDR + [0x02, 0xF7]


def send(midiout, msgs, gap=0.003):
    """Send the messages one by one (each send_message is one complete SysEx)."""
    for m in msgs:
        midiout.send_message(m)
        time.sleep(gap)


def parse_reply(msg):
    """Return a dict for an i2c2midi ER-301 reply, else None."""
    if len(msg) == 14 and list(msg[:5]) == HDR + [0x7E] and msg[-1] == 0xF7:
        kind, status, source, ncc, nnrpn, ngate, nnote, present = msg[5:13]
        return {"kind": "commit" if kind == 1 else "query", "status": status,
                "status_text": STATUS.get(status, f"unknown {status}"),
                "source": SOURCE.get(source, f"unknown {source}"),
                "cv": ncc, "nrpn": nnrpn, "triggers": ngate, "note_triggers": nnote,
                "er301_present": bool(present)}
    return None


def mtime(path=ROUTES_PATH):
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None
