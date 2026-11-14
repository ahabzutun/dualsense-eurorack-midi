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

DEFAULT_HOSTS = "ssp-eth ssp ssp-studio"
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

    def device_status(self, midi_in: Optional[str]) -> Tuple[str, bool]:
        """(text, ok) for a module's MIDI input device."""
        if not midi_in:
            return "no MIDI device set", False
        names = {d["name"]: d for d in self.devices()}
        ids = {d["id"]: d for d in self.devices()}
        if midi_in in names:
            return midi_in, True
        if re.fullmatch(r"\d+-\d+", midi_in):
            if midi_in in ids:
                return f"{ids[midi_in]['name']} (id {midi_in})", True
            return f"device id {midi_in}, saved in an earlier session", False
        return f"{midi_in}, not connected", False


def label(p: dict) -> str:
    n = p["file"].replace(".pbp", "")
    return f"{n} · {p.get('name') or '—'}"


def midi_modules(p: Optional[dict]) -> List[dict]:
    return [m for m in (p or {}).get("modules", []) if m.get("inputs")]


def module_key(m: dict) -> str:
    return f"{m['code']}{m['slot']}"


def module_title(m: dict) -> str:
    return f"{'MIDI module' if m['code'] == 'MIDI' else m['code']} · {m['slot'] + 1}"


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
