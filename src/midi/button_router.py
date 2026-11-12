"""
ButtonRouter: DualSense buttons -> actions, as configured in config/controls.py.

dualsense_midi.py hands every button event to the router first. The router decides tap /
hold / two-button chords and runs the same code paths the script used to hard-wire
(loop record/play/clear, motion toggle, slice blocks/sections, strum, freeze, quantize),
or sends the button's own CC ('midi'). The file is re-read when it changes; a broken file
is reported and the previous setting stays in use.
"""
import queue
import time

import controls_config
from config.mappings import CC_MAP, LONG_PRESS_DURATION

# evdev key codes of the gamepad device
KEY_CODES = {304: "cross", 305: "circle", 307: "triangle", 308: "square",
             310: "l1", 311: "r1", 314: "create", 315: "options", 316: "ps", 317: "l3", 318: "r3"}
LABEL = {"cross": "✕", "circle": "○", "triangle": "△", "square": "□", "l1": "L1", "r1": "R1", "l3": "L3",
         "r3": "R3", "create": "Create", "options": "Options", "ps": "PS", "mic": "Mic", "touchpad": "Touchpad click"}
RELOAD_INTERVAL = 1.0


class ButtonRouter:
    def __init__(self, controller, channel_manager, clock_source, midiout, path=controls_config.CONTROLS_PATH):
        self.c = controller
        self.cm = channel_manager
        self.clock = clock_source
        self.midiout = midiout
        self.path = path
        self.cfg = controls_config.load(path)   # raises if broken at startup -> caller falls back
        self._mtime = controls_config.mtime(path)
        self._last_check = time.time()
        self.down = {}            # button -> press time
        self.chord_used = set()   # buttons whose solo release actions are skipped
        self.held_midi = {}       # button -> last CC repeat time
        self.events = queue.Queue()   # (button, pressed) from other threads (mic via HID)
        self.routes_touchpad = True
        print(f"🎮 Controls loaded ({self._summary()})")

    # ---- config ----------------------------------------------------------------------

    def _summary(self):
        b = self.cfg["buttons"]
        fx = [f"{LABEL[n]}={v['tap']}" + (f"/{v['hold']}" if v["hold"] else "")
              for n, v in b.items() if v["tap"] not in ("midi", "none") or v["hold"]]
        ch = [f"{LABEL[a]}+{LABEL[b2]}={act}" for a, b2, act in self.cfg["chords"]]
        return ", ".join(fx + ch)

    def maybe_reload(self, now=None):
        now = time.time() if now is None else now
        if now - self._last_check < RELOAD_INTERVAL:
            return
        self._last_check = now
        m = controls_config.mtime(self.path)
        if m == self._mtime:
            return
        self._mtime = m
        try:
            self.cfg = controls_config.load(self.path)
            self.chord_used.clear()
            print(f"🎮 Controls reloaded ({self._summary()})")
        except controls_config.ControlsError as e:
            print(f"⚠️  controls.py NOT applied, keeping the previous setting: {e}")

    @property
    def dpad_horizontal(self):
        return self.cfg["dpad_horizontal"]

    @property
    def left_stick_x(self):
        return self.cfg["left_stick_x"]

    # ---- input -----------------------------------------------------------------------

    def handles(self, code):
        return code in KEY_CODES

    def on_key(self, code, value):
        if value == 2:            # autorepeat
            return
        self.on_button(KEY_CODES[code], value == 1)

    def post(self, button, pressed):
        """Thread-safe: queue a button event (used for the mic button from the HID thread)."""
        self.events.put((button, pressed))

    def poll(self):
        while True:
            try:
                button, pressed = self.events.get_nowait()
            except queue.Empty:
                return
            self.on_button(button, pressed)

    def on_button(self, name, pressed, now=None):
        now = time.time() if now is None else now
        if pressed:
            self._press(name, now)
        else:
            self._release(name, now)

    def _press(self, name, now):
        if name in self.down:
            return
        self.down[name] = now
        cfg = self.cfg["buttons"][name]
        if cfg["tap"] == "midi":
            self._midi(name, 127)
            self.held_midi[name] = now
        chord_fired = False
        for a, b, act in self.cfg["chords"]:
            if name in (a, b):
                other = b if name == a else a
                if other in self.down and other not in self.chord_used:
                    self.chord_used.update((a, b))
                    self.run(act, f"{LABEL[a]}+{LABEL[b]}")
                    chord_fired = True
        if chord_fired or cfg["hold"]:
            return
        tap = cfg["tap"]
        if tap not in ("midi", "none") and controls_config.ACTIONS[tap][0] == "press":
            self.run(tap, LABEL[name])

    def _release(self, name, now):
        t0 = self.down.pop(name, None)
        if t0 is None:
            return
        cfg = self.cfg["buttons"][name]
        if cfg["tap"] == "midi":
            self.held_midi.pop(name, None)
            self._midi(name, 0)
        if name in self.chord_used:
            self.chord_used.discard(name)
            return
        tap, hold = cfg["tap"], cfg["hold"]
        if hold:
            act = hold if now - t0 >= LONG_PRESS_DURATION else tap
            if act not in ("midi", "none"):
                self.run(act, LABEL[name] + (" hold" if act == hold else ""))
        elif tap not in ("midi", "none") and controls_config.ACTIONS[tap][0] == "release":
            self.run(tap, LABEL[name])

    def tick(self, now=None):
        """Call every loop: repeats the CC of held 'midi' buttons (MIDI learn) and reloads the config."""
        now = time.time() if now is None else now
        interval = getattr(self.c, "btn_repeat_interval", 0.05)
        for name, last in list(self.held_midi.items()):
            if now - last >= interval:
                self._midi(name, 127, quiet=True)
                self.held_midi[name] = now
        self.maybe_reload(now)

    # ---- outputs ---------------------------------------------------------------------

    def _send(self, msg):
        self.midiout.send_message(msg)
        loop_state = self.cm.get_current_loop_state()
        if loop_state.recording:
            loop_state.record_message(msg)

    def _midi(self, name, value, quiet=False):
        key, fallback = controls_config.BUTTON_CC[name]
        cc = CC_MAP.get(key, fallback)
        self._send([self.c.get_midi_channel_byte(0xB0), cc, value])
        if not quiet:
            print(f"🎚️  {LABEL[name]:<6} → CC{cc:2d}: {value:3d} (Ch {self.c.current_channel})")

    def _buzz(self, left, right, ms=0.05):
        try:
            if left: self.c.ds.setLeftMotor(100)
            if right: self.c.ds.setRightMotor(100)
            time.sleep(ms)
            if left: self.c.ds.setLeftMotor(0)
            if right: self.c.ds.setRightMotor(0)
        except Exception:
            pass

    def run(self, action, source=""):
        c, cm = self.c, self.cm
        ch = cm.current_channel
        loop_state = cm.get_current_loop_state()

        if action == "loop_record":
            if loop_state.recording:
                if loop_state.stop_recording():
                    print(f"\n⏹️  Channel {ch}: Recording STOPPED ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events) ({source})")
                else:
                    print(f"\n⚠️  Channel {ch}: Recording cancelled (empty or too long)")
            else:
                loop_state.start_recording()
                print(f"\n🔴 Channel {ch}: Recording STARTED ({source})")
            c.update_led_color()

        elif action == "loop_play":
            if loop_state.playing:
                loop_state.stop_playback()
                if c.quantize_on:
                    c.quantize_on = False
                    loop_state.quantize_subdivision = None
                    c.update_player_dots()
                c.update_led_color()
                print(f"\n⏸️  Channel {ch}: Playback STOPPED ({source})")
            elif loop_state.midi_buffer:
                if loop_state.start_playback(self.midiout,
                                             window_position_func=lambda: c.window_position,
                                             bpm_func=lambda: self.clock.get_current_bpm()):
                    c.update_led_color()
                    print(f"\n▶️  Channel {ch}: Playing loop ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events) ({source})")
            else:
                print(f"\n⚠️  Channel {ch}: No loop to play")

        elif action == "loop_clear":
            if loop_state.clear_loop():
                c.update_led_color()
                print(f"\n🗑️  Channel {ch}: Loop CLEARED ({source})")
            else:
                print(f"\n⚠️  Channel {ch}: Could not clear loop")

        elif action == "motion":
            idx = c.current_channel - 1
            c.motion_enabled[idx] = not c.motion_enabled[idx]
            if c.motion_enabled[idx]:
                c.tilt_baseline_x = c.smoothed_motion['tilt_x']
                c.tilt_baseline_y = c.smoothed_motion['tilt_y']
            print(f"\n🎛️  MOTION CONTROL {'ENABLED ✅' if c.motion_enabled[idx] else 'DISABLED ❌'} on Ch {c.current_channel} ({source})\n")
            c.update_led_color()
            if not c.is_motion_enabled():
                try:
                    c.ds.setLeftMotor(0); c.ds.setRightMotor(0)
                except Exception:
                    pass

        elif action in ("block_up", "block_down", "block_reset"):
            if action == "block_reset":
                c.sample_block = 0
            elif action == "block_up":
                c.sample_block = min(c.max_blocks, c.sample_block + 1)
                self._buzz(False, True)
            else:
                c.sample_block = max(0, c.sample_block - 1)
                self._buzz(True, False)
            base = c.sample_block * 128
            print(f"\n🔢 SAMPLE BLOCK {c.sample_block} (slices {base}-{base + 127}) ({source})\n")
            c.send_grain_slice(self.midiout)

        elif action == "section_up":
            c.handle_dpad_right()
            c.send_grain_slice(self.midiout)
        elif action == "section_down":
            c.handle_dpad_left()
            c.send_grain_slice(self.midiout)

        elif action == "strum":
            dpad_step = cm.dpad_vertical_steps[c.current_channel - 1]
            c.strummer.arpeggiate(self.midiout, c.current_channel,
                                  center_cc=cm.step_to_cc_value(dpad_step), loop_state=loop_state)

        elif action == "freeze_left":
            status = cm.toggle_left_group_freeze()
            print(f"\n❄️  Channel {ch}: L2/R2/Left Stick {status} ({source})")
            if status == "FROZEN":
                self._buzz(True, True)
        elif action == "freeze_right":
            status = cm.toggle_right_stick_freeze()
            print(f"\n❄️  Channel {ch}: Right Stick {status} ({source})")
            if status == "FROZEN":
                self._buzz(False, True)

        elif action == "quantize":
            c.toggle_quantize()


class NullRouter:
    """Used when controls can't be loaded at startup: the script keeps its built-in behaviour."""
    dpad_horizontal = "slice_section"
    left_stick_x = "slice_fine"
    routes_touchpad = False
    def handles(self, code): return False
    def on_key(self, code, value): pass
    def on_button(self, name, pressed, now=None): pass
    def post(self, button, pressed): pass
    def poll(self): pass
    def tick(self, now=None): pass
