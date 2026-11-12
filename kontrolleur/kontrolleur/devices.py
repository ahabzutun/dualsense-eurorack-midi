"""16n faderbank, DOREMiDi MPC-20 and Faderfox EC4: the data each device tab edits."""
from __future__ import annotations

import copy
import json
from typing import Dict, List, Optional

# ── 16n ─────────────────────────────────────────────────────────────────────────────────────────

FADER_DEFAULT = dict(usb_channel=16, usb_cc=80, usb_14bit=False, trs_channel=1, trs_cc=32, trs_14bit=False, curve=64)
OPTIONS_DEFAULT = dict(led_on=True, led_flash=True, rotate=False, i2c_leader=False, midi_thru=False,
                       fader_min=15, fader_max=8135)


def curve_exponent(v: int) -> float:
    """out = in ^ (2 ^ ((v − 64) / 16)) — same formula as the 16n firmware fork"""
    return 2.0 ** ((v - 64) / 16.0)


def curve_apply(x: float, v: int) -> float:
    return max(0.0, min(1.0, x)) ** curve_exponent(v)


def curve_name(v: int) -> str:
    if v == 64:
        return "Linear"
    k = curve_exponent(v)
    return f"Exp  x^{k:.1f}" if v > 64 else f"Log  x^{k:.2f}"


CURVE_PRESETS = [("Log+", 32), ("Log", 48), ("Lin", 64), ("Exp", 80), ("Exp+", 96)]


def _json(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


class Faderbank:
    def __init__(self):
        self.faders: List[dict] = [dict(FADER_DEFAULT, usb_cc=80 + k, trs_cc=32 + k) for k in range(16)]
        self.options: dict = dict(OPTIONS_DEFAULT)
        self.loaded = False
        self.device = ""
        self.firmware = ""
        self.curves_supported = False
        self.hires_supported = False
        self.port = ""

    def apply(self, d: dict):
        f = d.get("faders")
        if f and len(f) == 16:
            self.faders = [dict(FADER_DEFAULT, **x) for x in f]
        if d.get("options"):
            self.options = dict(OPTIONS_DEFAULT, **d["options"])
        self.device = d.get("device") or ""
        self.firmware = d.get("firmware") or ""
        self.curves_supported = bool(d.get("curves_supported"))
        self.hires_supported = bool(d.get("hires_supported"))
        self.port = d.get("port") or ""
        self.loaded = True

    @property
    def json(self) -> str:
        return _json({"faders": self.faders, "options": self.options})

    def problems(self) -> List[str]:
        p = []
        for k, f in enumerate(self.faders):
            if f["usb_14bit"] and f["usb_cc"] >= 96:
                p.append(f"Fader {k + 1}: 14-bit USB needs a CC below 96 (the LSB goes to CC + 32).")
            if f["trs_14bit"] and f["trs_cc"] >= 96:
                p.append(f"Fader {k + 1}: 14-bit TRS needs a CC below 96.")
        if self.options["fader_min"] >= self.options["fader_max"]:
            p.append("Fader min must be below fader max.")
        return p


# ── MPC-20 ──────────────────────────────────────────────────────────────────────────────────────

PEDAL_KINDS = ["cc", "note", "pc", "pitchbend", "pressure", "start", "continue", "stop"]
PEDAL_LABEL = {"cc": "CC", "note": "Note", "pc": "Program", "pitchbend": "Pitch bend", "pressure": "Ch. pressure",
               "start": "Start", "continue": "Continue", "stop": "Stop"}
PEDAL_SHORT = {"cc": "CC", "note": "N", "pc": "PC", "pitchbend": "PB", "pressure": "AT", "start": "START",
               "continue": "CONT", "stop": "STOP"}


def pedal_has_number(k: str) -> bool:
    return k in ("cc", "note", "pc")


def pedal_has_range(k: str) -> bool:
    return k in ("cc", "note", "pitchbend", "pressure")


def pedal_max(k: str) -> int:
    return 16383 if k == "pitchbend" else 127


def pedal_choices(t: str) -> List[str]:
    """What each pedal type can send (as the Pedal Config Tool offers it)."""
    if t == "switch":
        return ["cc", "note", "pc", "start", "continue", "stop"]
    if t == "hihat":
        return ["cc", "note"]
    return ["cc", "note", "pitchbend", "pressure"]


def pedal_name(i: int) -> str:
    return f"EXP{i + 1}" if i < 4 else f"EXP{i - 3} ring"


def hub_channel(ch: int, mapping) -> int:
    """Pedal messages on channel 1 follow the DualSense instrument channel in the MIDI hub."""
    return mapping.channel if ch == 1 else ch


class PedalBoard:
    def __init__(self):
        self.inputs: List[dict] = [
            {"input": i, "type": "none", "type_id": 0,
             "slots": [dict(slot=s, kind="cc", channel=1, number=0, start=0, end=0, mode=0, gain=50, ql=50, raw="", on=False)
                       for s in range(8)]}
            for i in range(8)]
        self.loaded = False
        self.device = ""
        self.firmware = ""
        self.port = ""

    def apply_inputs(self, lst: List[dict]):
        if len(lst) >= 4:
            self.inputs = copy.deepcopy(lst)
        self.loaded = True

    def apply(self, d: dict):
        if d.get("inputs") is not None:
            self.apply_inputs(d["inputs"])
        self.device = d.get("device") or ""
        self.firmware = d.get("firmware") or ""
        self.port = d.get("port") or ""

    @property
    def json(self) -> str:
        return _json({"inputs": self.inputs})

    def problems(self) -> List[str]:
        p = []
        for inp in self.inputs:
            if inp["type_id"] == 0:
                continue
            for s in inp["slots"]:
                if not s["on"]:
                    continue
                if not 1 <= s["channel"] <= 16:
                    p.append(f"{pedal_name(inp['input'])} message {s['slot'] + 1}: channel must be 1–16.")
                if pedal_has_range(s["kind"]) and s["start"] == s["end"]:
                    p.append(f"{pedal_name(inp['input'])} message {s['slot'] + 1}: start and end are the same value.")
        return p


# ── EC4 ─────────────────────────────────────────────────────────────────────────────────────────

EC4_TYPES = [("cc_rel1", "CC relative 1"), ("cc_rel2", "CC relative 2"), ("cc", "CC absolute"), ("pc", "Program change"),
             ("cc14", "CC 14-bit"), ("pitchbend", "Pitch bend"), ("aftertouch", "Aftertouch"), ("note", "Note"), ("nrpn", "NRPN")]
EC4_MODES = [("div8", "÷ 8 (fine)"), ("div4", "÷ 4"), ("div2", "÷ 2"), ("none", "no acceleration"), ("low", "low acceleration"),
             ("middle", "middle acceleration"), ("max", "max acceleration"), ("step2", "large step 2"), ("step4", "large step 4"),
             ("step6", "large step 6")]
EC4_SCALES = [("off", "display off"), ("0-127", "0 … 127"), ("0-100", "0 … 100"), ("0-1000", "0 … 1000"), ("-63..63", "−63 … +63"),
              ("-50..50", "−50 … +50"), ("-500..500", "−500 … +500"), ("on/off", "ON / OFF"), ("0-9999", "0 … 9999")]
EC4_PUSH_TYPES = [("off", "off"), ("note", "Note"), ("cc", "CC"), ("pc", "Program change"), ("pitchbend", "Pitch bend"),
                  ("aftertouch", "Aftertouch"), ("group", "select group"), ("setup", "select setup"), ("acc0", "acceleration 0"),
                  ("acc3", "acceleration 3"), ("lsp6", "large step 6"), ("min", "set to min"), ("max", "set to max")]


def ec4_short(t: str) -> str:
    return {"cc_rel1": "REL", "cc_rel2": "REL", "cc": "CC", "pc": "PC", "cc14": "CC14", "pitchbend": "PB",
            "aftertouch": "AT", "note": "N", "nrpn": "NRPN"}.get(t, t)


def ec4_highres(e: dict) -> bool:
    """14-bit ranges as in the Faderfox editor (CC 14-bit / PB / NRPN with a wide display)."""
    return e["type"] in ("cc14", "pitchbend", "nrpn") and e["scale"] in ("off", "0-1000", "-500..500", "0-9999")


class EC4Board:
    def __init__(self):
        self.setups: List[dict] = []
        self.mem = ""
        self.version = 0.0
        self.received_at = None   # datetime

    @property
    def loaded(self) -> bool:
        return bool(self.mem) and len(self.setups) == 16

    def apply(self, d: dict):
        s, m = d.get("setups"), d.get("mem")
        if not s or len(s) != 16 or not m:
            return
        self.setups = copy.deepcopy(s)
        self.mem = m
        self.version = float(d.get("version") or 0)

    @property
    def json(self) -> str:
        return _json({"mem": self.mem, "setups": self.setups})

    @property
    def fingerprint(self) -> str:
        """Only the editable part, for 'unsaved changes'."""
        return _json(self.setups)

    def problems(self) -> List[str]:
        p = []
        for s, su in enumerate(self.setups):
            for g, gr in enumerate(su["groups"]):
                for e, en in enumerate(gr["encoders"]):
                    if en["type"] == "cc14" and en["number"] > 31:
                        p.append(f"Setup {s + 1} group {g + 1} encoder {e + 1}: 14-bit CC needs a CC number 0–31.")
        return p
