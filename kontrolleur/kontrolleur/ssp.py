"""Percussa SSP: which MIDI the loaded patch listens to, and which of our controllers send it.

The SSP is reached over ssh (its own key / ~/.ssh/config); scripts/ssp_tool.py is piped to its python3
and answers with one JSON line (see that file for the .pbp format)."""
from __future__ import annotations

import json
import re
from typing import Dict, List, Optional, Tuple

from . import clients
from . import model as M
from . import store

DEFAULT_HOSTS = "ssp-eth ssp-wifi ssp ssp-studio"
DS_CHANNELS = (1, 2, 3)          # the DualSense switches between channels 1–3 (Create / Options)


class SSPState:
    def __init__(self):
        self.scan: Optional[dict] = None
        self.host = ""                # the address that answered
        self.error = ""
        self.loading = False
        self.pick = ""                # preset file chosen in the app; "" = follow the one loaded on the SSP
        try:
            self.scan = json.loads((store.data_dir() / "ssp-scan.json").read_text())
            self.stale = True         # from the last session, until a refresh
        except Exception:
            self.stale = False

    def save_cache(self):
        try:
            (store.data_dir() / "ssp-scan.json").write_text(json.dumps(self.scan))
        except OSError:
            pass

    @property
    def loaded(self) -> bool:
        return bool(self.scan and self.scan.get("ok"))

    @property
    def presets(self) -> List[dict]:
        return [p for p in (self.scan or {}).get("presets", []) if not p.get("error")]

    @property
    def current(self) -> dict:
        return (self.scan or {}).get("current") or {}

    @property
    def active_file(self) -> Optional[str]:
        files = {p["file"] for p in self.presets}
        if self.pick in files:
            return self.pick
        f = self.current.get("file")
        return f if f in files else None

    @property
    def active(self) -> Optional[dict]:
        f = self.active_file
        return next((p for p in self.presets if p["file"] == f), None)

    def devices(self) -> List[dict]:
        return (self.scan or {}).get("devices", [])

    def device_status(self, midi_in: Optional[str], preset: Optional[dict] = None) -> Tuple[str, bool]:
        """(text, ok) for a module's MIDI input device. Most modules save the device by its ALSA id
        ("20-0" = client 20 port 0), which depends on the order things were plugged in; the names
        for old ids come from modules that save both (GRID)."""
        if not midi_in:
            return "no MIDI device set", False
        names = {d["name"]: d for d in self.devices()}
        ids = {d["id"]: d for d in self.devices()}
        if midi_in in names:
            return midi_in, True
        if not re.fullmatch(r"\d+-\d+", midi_in):
            return f"{midi_in}, not connected", False
        known = ((preset or {}).get("device_names") or {}).get(midi_in) or \
            ((self.scan or {}).get("device_names") or {}).get(midi_in)
        if midi_in in ids:
            now = ids[midi_in]["name"]
            if known and known != now:
                return f"{now} (id {midi_in}; saved for {known})", True
            return now, True
        if known:
            if known in names:
                return f"{known}, saved as id {midi_in}; it's {names[known]['id']} now, so pick it again in the module", False
            return f"{known} (saved as id {midi_in}), not connected", False
        card = (int(midi_in.split("-")[0]) - 16) // 4
        return f"device id {midi_in} (the SSP's sound card {card} when the patch was saved), not connected", False


def label(p: dict) -> str:
    n = p["file"].replace(".pbp", "")
    return f"{n} · {p.get('name') or '—'}"


def midi_modules(p: Optional[dict]) -> List[dict]:
    return [m for m in (p or {}).get("modules", []) if m.get("inputs")]


def module_key(m: dict) -> str:
    return f"{m['code']}{m['slot']}"


def module_title(m: dict) -> str:
    return f"{'MIDI module' if m['code'] == 'MIDI' else m['code']} · {m['slot'] + 1}"


# Parameter names of plugin modules, in the order the plugin creates them (MIDI learn stores that index).
# From the plugins' source (github.com/TheTechnobear/SSP, technobear/*/Source/PluginProcessor.cpp).
def _pmix():
    sends = ["Level", "Cue send", "Aux 1", "Aux 2", "Pan", "Gain", "Mute", "Solo", "Cue", "AC"]
    return [f"In {n + 1} {x}" for n in range(8) for x in sends] + [f"Out {n + 1} {x}" for n in range(8) for x in sends]


PARAM_NAMES = {
    "PMIX": _pmix(),
    "ATTN": ["Slew"] + [f"Attn {c}" for c in "ABCDEFGHIJKLMNOP"],
    "PVCA": [f"VCA {v + 1} {x}" for v in range(4) for x in ("Gain", "Pan")],
    "LDRF": [f"{x} {f + 1}" for f in range(4) for x in ("Cutoff", "Res")],
    "MMX4": [f"VCA {o}{c}" for o in "1234" for c in "ABCD"],
    "COMP": ["Ratio", "Threshold", "Attack", "Release", "Makeup", "Auto makeup"],
    "MTIN": [f"CC {c}" for c in "ABCDEFGH"] + ["Slew CC", "PB range", "Clock", "Transport"],
    "MTOT": [f"CC {c}" for c in "ABCDEFGH"] + ["PB range"],
    "MTTR": [f"Note {c}" for c in "ABCDEFGH"],
    "MSW8": ["In select", "Out select", "Active", "Soft"],
    "CLDS": ["Position", "Size", "Density", "Texture", "Mix", "Spread", "Feedback", "Reverb", "Pitch", "Mode", "In gain", "Freeze"],
    "PLTS": ["Pitch", "Harmonics", "Timbre", "Morph", "Model", "Freq mod", "Timbre mod", "Morph mod", "LPG", "VCA", "Trig", "Level"],
    "RNGS": ["Pitch", "Structure", "Brightness", "Damping", "Position", "Polyphony", "Model", "Bypass", "Easter egg", "In gain", "In", "Strum"],
    "SRVB": ["LP freq", "Feedback", "Mix", "Freeze"],
}


def target_text(m: dict, i: dict) -> str:
    """'In 1 Level' for MIDI learn on a known plugin, else what the tool said ('param 10', 'CV A' …)."""
    names = PARAM_NAMES.get(m.get("code", "").upper())
    k = i.get("param")
    if names and k is not None and 0 <= k < len(names):
        return names[k]
    return i["target"]


def input_text(i: dict) -> str:
    ch = f"ch {i['ch']}" if i["ch"] else "any ch"
    if i["kind"] == "cc":
        return f"{ch} CC {i['num']}"
    if i["kind"] in ("note", "notes"):
        return f"{ch} notes" if i["num"] < 0 else f"{ch} note {i['num']}"
    if i["kind"] == "pc":
        return f"{ch} program change"
    return f"{ch} {i['kind']} {i['num']}"


# ── who sends what into the MIDI hub (→ SSP) ───────────────────────────────────────────────────

def senders(ctx) -> Dict[tuple, List[Tuple[str, str]]]:
    """(channel as the SSP sees it, 'cc' | 'note', number) → [(device, label)].
    The MIDI hub forwards the DualSense, 16n, pedals and EC4 to the SSP."""
    from .tabs.dualsense import short_name
    out: Dict[tuple, List[Tuple[str, str]]] = {}
    m = ctx.mapping

    def add(key, dev, lab):
        lst = out.setdefault(key, [])
        if (dev, lab) not in lst:
            lst.append((dev, lab))

    for i in M.ALL_INPUTS:
        if i.kind == "cc" and m.sends_midi(i):
            for ch in DS_CHANNELS:
                add((ch, "cc", m.number(i)), "dualsense", f"{short_name(i)} (ch {ch})")
    for k, f in enumerate(ctx.bank.faders):
        add((f["usb_channel"], "cc", f["usb_cc"]), "16n", f"Fader {k + 1}")
    from .devices import pedal_name
    for inp in ctx.pedals.inputs:
        for s in inp["slots"]:
            if s["on"] and s["kind"] in ("cc", "note"):
                add((m.hub_ch("pedal", s["channel"]), s["kind"], s["number"]), "pedal",
                    f"{pedal_name(inp['input'])} · message {s['slot'] + 1}")
    for si, su in enumerate(ctx.ec4.setups):
        for gi, g in enumerate(su["groups"]):
            for k, e in enumerate(g["encoders"]):
                key_e = (e["channel"], "cc", e["number"])
                if e["type"] == "cc" and key_e not in out:
                    add(key_e, "ec4", f"S{si + 1} G{gi + 1} encoder {k + 1}" + (f" “{e['name'].strip()}”" if e["name"].strip() else ""))
                p = e["push"]
                key_p = (p["channel"], p["type"], p["number"])
                if p["type"] in ("cc", "note") and key_p not in out:
                    add(key_p, "ec4", f"S{si + 1} G{gi + 1} push {k + 1}")
    return out


def matches(snd: Dict[tuple, list], i: dict) -> List[Tuple[str, str]]:
    """The controllers that send what this SSP input listens to."""
    if i["kind"] == "cc":
        if i["ch"]:
            return snd.get((i["ch"], "cc", i["num"]), [])
        return [s for (ch, k, n), lst in snd.items() if k == "cc" and n == i["num"] for s in lst]
    if i["kind"] in ("note", "notes"):
        return [s for (ch, k, n), lst in snd.items()
                if k == "note" and (not i["ch"] or ch == i["ch"]) and (i["num"] < 0 or n == i["num"]) for s in lst]
    return []


# ── loading ────────────────────────────────────────────────────────────────────────────────────

def scan_remote(hosts: str, identity: str) -> Tuple[str, dict]:
    """Try each host in turn; (host that answered, scan)."""
    src = (clients.SCRIPTS / "ssp_tool.py").read_text()
    errors = []
    for h in hosts.replace(",", " ").split():
        target = h if "@" in h else f"root@{h}"
        try:
            text = clients.run_ssh(target, identity, "python3 - scan", stdin=src, timeout=40, connect_timeout=4)
            return h, clients.last_json(text, h)
        except clients.ToolError as e:
            errors.append(f"{h}: {e}")
    raise clients.ToolError("SSP not reachable (" + "; ".join(errors) + ")" if errors else "no SSP host set")
