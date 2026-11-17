"""Grid macros (the left half of the monome grid, run by the Pi): the data of grid_macros.json as Kontrolleur
edits it, and what its values mean in this rig (which control they came from, where they land).

The Pi side is src/macros/ and scripts/grid_macros_tool.py; docs/grid-macros.md describes the whole thing."""
from __future__ import annotations

import copy
import json
from typing import List, Optional, Tuple

BANKS, SLOTS, SLEW_STEPS = 8, 7, 6
PROVIDERS = [("dualsense", "DualSense script"), ("midi", "MIDI values"), ("teletype", "Teletype's LEDs")]
PROVIDER_NAMES = [p for p, _ in PROVIDERS]
# controllers the macros can remember (matched against the Pi's MIDI port names)
SOURCES = [("16n", "16n"), ("EC4", "EC4"), ("CH345", "Pedals (MPC-20)")]
CURVES = [("linear", "Linear"), ("smooth", "Smooth (ease in / out)"), ("exp", "Exponential (slow start)")]

DEFAULT_SETTINGS = {
    "glide_times": [0.0, 0.1, 0.5, 2.0, 8.0, 30.0], "curve": "linear", "hold_to_save": 0.5, "jump_at": "end",
    "tick_hz": 50, "providers": list(PROVIDER_NAMES), "bank_providers": {}, "midi_sources": ["16n", "EC4"],
    "pickup_sources": ["16n"], "pickup_window": 2, "feedback_sources": ["EC4"],
}


def default_data() -> dict:
    return {"version": 1, "settings": copy.deepcopy(DEFAULT_SETTINGS),
            "banks": [[None] * SLOTS for _ in range(BANKS)], "slew": [[0] * SLOTS for _ in range(BANKS)],
            "names": [[""] * SLOTS for _ in range(BANKS)], "bank_names": [""] * BANKS}


def _fill(data: dict) -> dict:
    """Older files (from before names) get the missing parts."""
    d = default_data()
    if not isinstance(data, dict):
        return d
    for k in d:
        if k in data and data[k] is not None:
            d[k] = data[k]
    s = dict(DEFAULT_SETTINGS)
    s.update(d.get("settings") or {})
    d["settings"] = s
    return d


def fmt_secs(v: float) -> str:
    if v <= 0:
        return "instant"
    return f"{v:g} s" if v < 60 else f"{v / 60:g} min"


class MacrosState:
    def __init__(self):
        self.data = default_data()
        self.loaded = False
        self.loading = False
        self.sha = ""               # fingerprint of grid_macros.json as loaded (the Pi refuses an upload if it changed)
        self.exists = False         # the file is on the Pi (the service has run at least once)
        self.service: Optional[dict] = None     # the running service's state when loaded (None: not running)
        self.path = ""
        self.error = ""
        self.synced: Optional[str] = None

    # ── loading / dirty ──
    def apply(self, export: dict):
        self.data = _fill(export.get("data"))
        self.sha = export.get("sha") or ""
        self.exists = bool(export.get("exists"))
        self.service = export.get("service")
        self.path = export.get("path") or ""
        self.loaded = True
        self.error = ""
        self.synced = self.fingerprint

    @property
    def fingerprint(self) -> str:
        return json.dumps(self.data, sort_keys=True)

    @property
    def dirty(self) -> bool:
        return self.synced is not None and self.synced != self.fingerprint

    # ── access ──
    @property
    def settings(self) -> dict:
        return self.data["settings"]

    def slot(self, b: int, s: int) -> Optional[dict]:
        return self.data["banks"][b][s]

    def name(self, b: int, s: int) -> str:
        return self.data["names"][b][s]

    def set_name(self, b: int, s: int, text: str):
        self.data["names"][b][s] = text[:40]

    def bank_name(self, b: int) -> str:
        return self.data["bank_names"][b]

    def set_bank_name(self, b: int, text: str):
        self.data["bank_names"][b] = text[:40]

    def slew(self, b: int, s: int) -> int:
        return int(self.data["slew"][b][s])

    def set_slew(self, b: int, s: int, step: int):
        self.data["slew"][b][s] = max(0, min(SLEW_STEPS - 1, int(step)))

    def glide_times(self) -> List[float]:
        t = list(self.settings.get("glide_times") or DEFAULT_SETTINGS["glide_times"])
        return (t + DEFAULT_SETTINGS["glide_times"][len(t):])[:SLEW_STEPS]

    def glide(self, b: int, s: int) -> float:
        return float(self.glide_times()[self.slew(b, s)])

    def bank_used(self, b: int) -> bool:
        return any(x is not None for x in self.data["banks"][b])

    def used_slots(self) -> int:
        return sum(1 for b in range(BANKS) for s in range(SLOTS) if self.slot(b, s) is not None)

    def active(self, b: int) -> Optional[int]:
        a = (self.service or {}).get("active") or {}
        v = a.get(str(b))
        return int(v) if v is not None else None

    def providers_for(self, b: int) -> List[str]:
        per = self.settings.get("bank_providers") or {}
        names = per.get(str(b))
        return list(names) if names is not None else list(self.settings.get("providers") or PROVIDER_NAMES)

    def set_bank_providers(self, b: int, names: Optional[List[str]]):
        per = dict(self.settings.get("bank_providers") or {})
        if names is None:
            per.pop(str(b), None)
        else:
            per[str(b)] = list(names)
        self.settings["bank_providers"] = per

    # ── editing slots ──
    def clear(self, b: int, s: int):
        self.data["banks"][b][s] = None
        self.data["names"][b][s] = ""

    def copy_slot(self, src: Tuple[int, int], dst: Tuple[int, int]):
        (b1, s1), (b2, s2) = src, dst
        self.data["banks"][b2][s2] = copy.deepcopy(self.data["banks"][b1][s1])
        self.data["slew"][b2][s2] = self.data["slew"][b1][s1]
        self.data["names"][b2][s2] = self.data["names"][b1][s1]

    def move_slot(self, src: Tuple[int, int], dst: Tuple[int, int]):
        """Swaps when the target holds something, so nothing is lost by a move."""
        (b1, s1), (b2, s2) = src, dst
        for k in ("banks", "slew", "names"):
            a = self.data[k]
            a[b1][s1], a[b2][s2] = a[b2][s2], a[b1][s1]

    def copy_bank(self, b1: int, b2: int):
        for s in range(SLOTS):
            self.copy_slot((b1, s), (b2, s))
        self.data["bank_names"][b2] = self.data["bank_names"][b1]
        per = self.settings.get("bank_providers") or {}
        self.set_bank_providers(b2, per.get(str(b1)))

    def remove_part(self, b: int, s: int, provider: str):
        snap = self.slot(b, s)
        if snap is not None:
            (snap.get("state") or {}).pop(provider, None)

    def remove_value(self, b: int, s: int, index: int):
        snap = self.slot(b, s)
        vals = ((snap or {}).get("state") or {}).get("midi", {}).get("values")
        if vals is not None and 0 <= index < len(vals):
            del vals[index]
            if not vals:
                self.remove_part(b, s, "midi")

    # ── backup / restore ──
    def export_set(self) -> dict:
        return {"kontrolleur": "grid-macros", "version": 1, "data": copy.deepcopy(self.data)}

    def import_set(self, obj: dict):
        data = obj.get("data") if isinstance(obj, dict) and "data" in obj else obj
        if not isinstance(data, dict) or "banks" not in data:
            raise ValueError("not a grid macros file")
        self.data = _fill(copy.deepcopy(data))


# ── what a snapshot holds, in this rig's words ───────────────────────────────────────────────────────

def midi_values(snap: Optional[dict]) -> List[Tuple[str, int, int, int, str]]:
    out = []
    for e in (((snap or {}).get("state") or {}).get("midi") or {}).get("values") or []:
        try:
            kind, ch, num, value, src = e
            out.append((str(kind), int(ch), int(num), int(value), str(src or "")))
        except (TypeError, ValueError):
            continue
    return out


def value_name(ctx, kind: str, ch: int, num: int, src: str, senders=None) -> str:
    """'16n Fader 3', 'EC4 S1 G1 encoder 2 “CUT”' — or the raw message when no known control sends it."""
    if kind == "cc" and senders is not None:
        names = {"16n": "16n", "ec4": "EC4", "pedal": "Pedal", "dualsense": "DualSense", "teletype": "Teletype"}
        for dev, lab in senders.get((ch, "cc", num), []):
            if dev != "teletype":
                return f"{names.get(dev, dev)} {lab}"
    raw = f"ch {ch} {'CC' if kind == 'cc' else 'NRPN'} {num}"
    return f"{src} · {raw}" if src else raw


def destinations(ctx, kind: str, ch: int, num: int) -> List[str]:
    """Where this value lands: ER-301 routes through the i2c2midi, inputs of the SSP patch in use."""
    out = []
    if kind == "cc":
        m = ctx.mapping
        e = m.ext_route(ch, num, False)
        if e is not None:
            out.append("ER-301 " + m.route_label(e))
        try:
            from . import ssp as S
            for mod, inp in S.listeners(ctx, ch, "cc", num):
                out.append("SSP " + S.target_text(mod, inp))
        except Exception:
            pass
    return out


def ds_summary(d: Optional[dict]) -> str:
    if not d:
        return ""
    parts = [f"slice {d.get('slice', '?')}"]
    fr = [k for k in ("left", "right") if (d.get("freeze") or {}).get(k)]
    if fr:
        parts.append("frozen " + " + ".join(fr))
    if d.get("motion"):
        parts.append("motion on")
    return " · ".join(parts)


def summary(snap: Optional[dict]) -> List[str]:
    if snap is None:
        return []
    st = snap.get("state") or {}
    out = []
    if st.get("dualsense"):
        out.append(ds_summary(st["dualsense"]))
    n = len(midi_values(snap))
    if n:
        out.append(f"{n} MIDI value{'s' if n != 1 else ''}")
    if st.get("teletype"):
        out.append("Teletype view")
    return out or ["nothing captured"]
