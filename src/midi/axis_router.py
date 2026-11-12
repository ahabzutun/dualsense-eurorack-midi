"""
AxisRouter: DualSense axes -> functions, as configured in AXES of config/controls.py.

dualsense_midi.py reads an axis (with freeze applied) and hands the value here together with a
function that sends the axis' own MIDI. Depending on the configured function the router sends
that MIDI, moves the grain slice (slice / section / block), scrubs the playing loop or picks the
quantize subdivision. The configuration comes from the ButtonRouter (same file, same reload).
"""
import time

import controls_config

SUBDIVISIONS = [32, 16, 8, 4]       # quantize grid, top -> bottom / low -> high


class AxisRouter:
    def __init__(self, controller, channel_manager, midiout, router=None):
        self.c = controller
        self.cm = channel_manager
        self.midiout = midiout
        self.router = router

    def fn(self, axis):
        cfg = getattr(self.router, "cfg", None)
        axes = cfg.get("axes") if isinstance(cfg, dict) else None
        if axes and axis in axes:
            return axes[axis]
        return controls_config.DEFAULTS["axes"][axis]

    # ---- continuous axes: value 0-127 --------------------------------------------------

    def value(self, axis, cc_val, send_midi):
        f = self.fn(axis)
        if f == "midi":
            send_midi()
            return
        self.apply(f, cc_val, axis)

    def apply(self, f, cc_val, axis=""):
        """Run a continuous function with a value 0-127 (also used for 16n / EC4 / pedal CCs)."""
        c = self.c
        if f == "slice_fine":
            c.update_fine_slice(int(cc_val * 255 / 127))
            c.send_grain_slice(self.midiout)
        elif f == "slice_section":
            bank = min(c.max_banks, cc_val * (c.max_banks + 1) // 128)
            if bank != c.slice_bank:
                c.slice_bank = bank
                print(f"🔢 Slice section {bank} ({axis})")
                c.send_grain_slice(self.midiout)
        elif f == "slice_block":
            block = min(c.max_blocks, cc_val * (c.max_blocks + 1) // 128)
            if block != c.sample_block:
                c.sample_block = block
                print(f"🔢 Sample block {block} (slices {block * 128}-{block * 128 + 127}) ({axis})")
                c.send_grain_slice(self.midiout)
        elif f == "loop_scrub":
            loop = self.cm.get_current_loop_state()
            if loop.playing and loop.loop_duration:
                c.window_position = cc_val / 127.0
        elif f == "quantize_grid":
            if time.time() - getattr(c, "_last_quantize_toggle_time", 0.0) < 0.3:
                return                      # a touchpad click re-registers as a touch
            sub = SUBDIVISIONS[min(cc_val // 32, 3)]
            if sub != c.last_quantize_subdivision:
                c.last_quantize_subdivision = sub
                loop = self.cm.get_current_loop_state()
                if c.quantize_on:
                    loop.quantize_subdivision = sub
                c.update_player_dots()
                print(f"🎵 Quantize subdivision: 1/{sub} ({axis})")

    # ---- step axes (D-pad): direction -1 / +1 ------------------------------------------

    def step(self, axis, direction, send_midi):
        f = self.fn(axis)
        if f == "midi":
            send_midi()
            return
        c = self.c
        if f == "slice_section":
            c.handle_dpad_right() if direction > 0 else c.handle_dpad_left()
            c.send_grain_slice(self.midiout)
        elif f == "slice_block":
            block = max(0, min(c.max_blocks, c.sample_block + (1 if direction > 0 else -1)))
            if block != c.sample_block:
                c.sample_block = block
                print(f"🔢 Sample block {block} (slices {block * 128}-{block * 128 + 127}) ({axis})")
                c.send_grain_slice(self.midiout)
