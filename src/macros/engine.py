"""
The grid macros: the left 8×8 of the grid, run by the Pi instead of Teletype.

    x = 0       bank buttons (8 banks, one per row)
    x = 1 … 7   one macro slot per column:
        y = 0       save   — hold (0.5 s) to save the current state into the slot
        y = 1 … 6   slew fader — how slowly a load moves there: y = 6 instant … y = 1 slowest
        y = 7       load   — tap to go to the slot's state
    hold save + tap load of the same column = clear the slot

LEDs (0–15): current bank bright, banks with saved slots dim; saved slots show their save and load keys,
the slot loaded last in a bank is bright (pulsing while it glides in), the fader shows the slew setting.

Everything is driven from outside: key(x, y, down) for presses, tick() regularly (it runs the glides and
returns new LED levels when they changed). The clock is injectable for tests.
"""
import math
import time

from .glide import Glide
from .pickup import Pickup
from .store import BANKS, SLOTS

SAVE_ROW = 0
LOAD_ROW = 7
FADER_ROWS = range(1, 7)
SAVED_FLASH = 0.35


def fader_step(y: int) -> int:
    """Grid row 1–6 → slew step 5–0 (row 6, just above load, is instant)."""
    return 6 - y


def fader_row(step: int) -> int:
    return 6 - step


class MacroEngine:
    def __init__(self, store, providers, clock=time.monotonic, log=print):
        self.store = store
        self.providers = providers                    # name -> Provider
        self.clock = clock
        self.log = log or (lambda *a: None)
        self.glide = Glide(store.settings.get("curve", "linear"))
        self.pickup = Pickup(int(store.settings.get("pickup_window", 2)))
        for p in providers.values():
            if hasattr(p, "glide"):
                p.glide = self.glide
            if hasattr(p, "pickup"):
                p.pickup = self.pickup
        self.bank = 0
        self.active = {}                              # bank -> slot loaded last
        self.loading = None                           # (bank, slot, t_end) while a load glides in
        self.save_held = {}                           # slot -> time pressed (current bank)
        self.save_done = set()                        # slots saved during the current hold
        self.flash = {}                               # (bank, slot) -> until
        self._last_leds = None

    # ---- input -----------------------------------------------------------------------------------
    def key(self, x, y, down):
        if not (0 <= x < 8 and 0 <= y < 8):
            return
        now = self.clock()
        if x == 0:
            if down:
                self.bank = y
                self.save_held.clear()
                self.save_done.clear()
            return
        slot = x - 1
        if y == SAVE_ROW:
            if down:
                self.save_held[slot] = now
            else:
                self.save_held.pop(slot, None)
                self.save_done.discard(slot)
            return
        if not down:
            return
        if y == LOAD_ROW:
            if slot in self.save_held:                # save held + load = clear
                self.clear(slot)
                self.save_held.pop(slot, None)
                self.save_done.add(slot)              # the release of save must not do anything
                return
            self.load(slot)
            return
        if y in FADER_ROWS:
            self.store.set_slew(self.bank, slot, fader_step(y))

    # ---- actions ---------------------------------------------------------------------------------
    def save(self, slot):
        bank = self.bank
        state = {}
        for name in self.store.providers_for(bank):
            p = self.providers.get(name)
            if p is None:
                continue
            try:
                data = p.capture()
            except Exception as e:                    # a provider must never break the grid
                self.log(f"⚠️  macro save: {name} failed: {e}")
                data = None
            if data is not None:
                state[name] = data
        self.store.set_slot(bank, slot, state)
        self.flash[(bank, slot)] = self.clock() + SAVED_FLASH
        self.log(f"💾 macro {bank + 1}.{slot + 1} saved ({', '.join(state) or 'nothing to remember'})")

    def load(self, slot):
        bank = self.bank
        snap = self.store.slot(bank, slot)
        if snap is None:
            return False
        now = self.clock()
        duration = self.store.glide_time(bank, slot)
        jump_at = self.store.settings.get("jump_at", "end")
        self.glide.set_curve(self.store.settings.get("curve", "linear"))
        self.glide.cancel_all()
        allowed = set(self.store.providers_for(bank))
        for name, data in (snap.get("state") or {}).items():
            p = self.providers.get(name)
            if p is None or name not in allowed:
                continue
            try:
                p.restore(data, self.glide, duration, now, jump_at)
            except Exception as e:
                self.log(f"⚠️  macro load: {name} failed: {e}")
        self.active[bank] = slot
        self.loading = (bank, slot, now + duration) if duration > 0 else None
        self.log(f"▶️  macro {bank + 1}.{slot + 1} loaded ({duration:g} s)")
        return True

    def clear(self, slot):
        self.store.clear_slot(self.bank, slot)
        if self.active.get(self.bank) == slot:
            del self.active[self.bank]
        self.log(f"🗑️  macro {self.bank + 1}.{slot + 1} cleared")

    # ---- time ------------------------------------------------------------------------------------
    def tick(self):
        """Run glides and pending saves. Returns 64 LED levels when they changed, else None."""
        now = self.clock()
        self.glide.tick(now)
        hold = float(self.store.settings.get("hold_to_save", 0.5))
        for slot, t in list(self.save_held.items()):
            if slot not in self.save_done and now - t >= hold:
                self.save(slot)
                self.save_done.add(slot)
        if self.loading and (now >= self.loading[2] or not self.glide.active):
            self.loading = None
        for k, until in list(self.flash.items()):
            if now >= until:
                del self.flash[k]
        leds = self.leds(now)
        if leds != self._last_leds:
            self._last_leds = leds
            return leds
        return None

    def force_redraw(self):
        self._last_leds = None

    # ---- LEDs ------------------------------------------------------------------------------------
    def leds(self, now=None):
        now = self.clock() if now is None else now
        lv = [0] * 64
        bank = self.bank
        for y in range(BANKS):
            lv[y * 8] = 15 if y == bank else (5 if self.store.bank_used(y) else 1)
        hold = float(self.store.settings.get("hold_to_save", 0.5)) or 0.5
        for slot in range(SLOTS):
            x = slot + 1
            saved = self.store.slot(bank, slot) is not None
            # save key
            if (bank, slot) in self.flash:
                level = 15
            elif slot in self.save_held and slot not in self.save_done:
                level = 4 + int(11 * min(1.0, (now - self.save_held[slot]) / hold))
            else:
                level = 6 if saved else 2
            lv[SAVE_ROW * 8 + x] = level
            # slew fader: filled from the bottom (row 6) up to the chosen step
            step = self.store.slew(bank, slot)
            for y in FADER_ROWS:
                s = fader_step(y)
                if s == step:
                    lv[y * 8 + x] = 10 if saved else 5
                elif s < step:
                    lv[y * 8 + x] = 4 if saved else 2
            # load key
            if self.active.get(bank) == slot and saved:
                if self.loading and self.loading[0] == bank and self.loading[1] == slot:
                    level = 8 + int(7 * abs(math.sin(now * 6)))
                else:
                    level = 15
            else:
                level = 6 if saved else 0
            lv[LOAD_ROW * 8 + x] = level
        return lv
