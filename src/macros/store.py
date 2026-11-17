"""
The macro store: 8 banks × 7 slots of snapshots, each slot's slew setting, and the settings of the grid
macros, in one JSON file (~/.config/dualsense-eurorack-midi/grid_macros.json). Kontrolleur can read and
write it like the other files there; the service notices outside changes and reloads.

A saved slot:  {"saved_at": <unix time>, "state": {"<provider>": <what that provider captured>, …}}
Names (set in Kontrolleur, not needed by the grid): "names" 8 × 7 slot names, "bank_names" 8 bank names.
"""
import copy
import json
import os
import time

BANKS = 8
SLOTS = 7
SLEW_STEPS = 6

DEFAULT_SETTINGS = {
    # seconds per slew fader step, from the step just above the load key (0) to the one below save (5)
    "glide_times": [0.0, 0.1, 0.5, 2.0, 8.0, 30.0],
    "curve": "linear",              # linear · smooth · exp
    "hold_to_save": 0.5,            # seconds the save key has to be held
    "jump_at": "end",               # values that can't glide (slice, freeze …) change at the "start" or "end"
    "tick_hz": 50,                  # glide steps per second
    "providers": ["dualsense", "midi", "teletype"],
    "bank_providers": {},           # optional per bank: {"2": ["midi"]} = bank 3 only touches MIDI values
    # MIDI values that are remembered: messages from inputs whose port name contains one of these
    "midi_sources": ["16n", "EC4"],
    # these sources get pickup (their faders have to pass the value before they take over again)
    "pickup_sources": ["16n"],
    "pickup_window": 2,
    # these sources get the new values sent back (endless encoders that keep their own value)
    "feedback_sources": ["EC4"],
}


def default_data():
    return {
        "version": 1,
        "settings": copy.deepcopy(DEFAULT_SETTINGS),
        "banks": [[None] * SLOTS for _ in range(BANKS)],
        "slew": [[0] * SLOTS for _ in range(BANKS)],
        "names": [[""] * SLOTS for _ in range(BANKS)],
        "bank_names": [""] * BANKS,
    }


class MacroStore:
    def __init__(self, path=None):
        self.path = path
        self.data = default_data()
        self._mtime = None
        if path:
            self.load()

    # ---- file ------------------------------------------------------------------------------------
    def load(self):
        try:
            with open(self.path) as f:
                raw = json.load(f)
            self._mtime = os.path.getmtime(self.path)
        except (OSError, ValueError):
            return False
        self.data = self._normalise(raw)
        return True

    def reload_if_changed(self) -> bool:
        if not self.path:
            return False
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            return False
        if m != self._mtime:
            return self.load()
        return False

    def save(self):
        if not self.path:
            return
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = f"{self.path}.tmp-{os.getpid()}"
        with open(tmp, "w") as f:
            json.dump(self.data, f, indent=1)
        os.replace(tmp, self.path)
        self._mtime = os.path.getmtime(self.path)

    @staticmethod
    def _normalise(raw):
        d = default_data()
        if not isinstance(raw, dict):
            return d
        s = raw.get("settings")
        if isinstance(s, dict):
            for k, v in s.items():
                if k in d["settings"]:
                    d["settings"][k] = v
        banks = raw.get("banks")
        if isinstance(banks, list):
            for b in range(min(BANKS, len(banks))):
                if isinstance(banks[b], list):
                    for s_ in range(min(SLOTS, len(banks[b]))):
                        slot = banks[b][s_]
                        d["banks"][b][s_] = slot if isinstance(slot, dict) else None
        slew = raw.get("slew")
        if isinstance(slew, list):
            for b in range(min(BANKS, len(slew))):
                if isinstance(slew[b], list):
                    for s_ in range(min(SLOTS, len(slew[b]))):
                        try:
                            d["slew"][b][s_] = max(0, min(SLEW_STEPS - 1, int(slew[b][s_])))
                        except (TypeError, ValueError):
                            pass
        names = raw.get("names")
        if isinstance(names, list):
            for b in range(min(BANKS, len(names))):
                if isinstance(names[b], list):
                    for s_ in range(min(SLOTS, len(names[b]))):
                        if isinstance(names[b][s_], str):
                            d["names"][b][s_] = names[b][s_][:40]
        bnames = raw.get("bank_names")
        if isinstance(bnames, list):
            for b in range(min(BANKS, len(bnames))):
                if isinstance(bnames[b], str):
                    d["bank_names"][b] = bnames[b][:40]
        return d

    @classmethod
    def normalise(cls, raw):
        """Clean up data from outside (Kontrolleur, a backup): unknown keys dropped, ranges clamped."""
        return cls._normalise(raw)

    # ---- access ----------------------------------------------------------------------------------
    @property
    def settings(self):
        return self.data["settings"]

    def slot(self, bank, slot):
        return self.data["banks"][bank][slot]

    def set_slot(self, bank, slot, state):
        self.data["banks"][bank][slot] = {"saved_at": time.time(), "state": state}
        self.save()

    def clear_slot(self, bank, slot):
        self.data["banks"][bank][slot] = None
        self.save()

    def slew(self, bank, slot) -> int:
        return self.data["slew"][bank][slot]

    def set_slew(self, bank, slot, step):
        step = max(0, min(SLEW_STEPS - 1, int(step)))
        if self.data["slew"][bank][slot] != step:
            self.data["slew"][bank][slot] = step
            self.save()

    def glide_time(self, bank, slot) -> float:
        times = self.settings.get("glide_times") or DEFAULT_SETTINGS["glide_times"]
        step = self.slew(bank, slot)
        try:
            return float(times[min(step, len(times) - 1)])
        except (TypeError, ValueError, IndexError):
            return 0.0

    def name(self, bank, slot) -> str:
        return self.data["names"][bank][slot]

    def bank_used(self, bank) -> bool:
        return any(s is not None for s in self.data["banks"][bank])

    def providers_for(self, bank):
        per_bank = self.settings.get("bank_providers") or {}
        names = per_bank.get(str(bank))
        if names is None:
            names = self.settings.get("providers") or DEFAULT_SETTINGS["providers"]
        return list(names)
