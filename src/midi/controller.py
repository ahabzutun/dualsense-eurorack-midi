"""
MIDI Controller for DualSense
Handles MIDI message generation, LED control, haptics, and channel switching

FIX: LED pulse thread leak - start_led_pulse() was spawning new Thread objects
     without guaranteeing the old one exited, due to a shared led_pulse_active
     flag that the new thread could set True before the old one checked it.
     Fixed by giving each thread its own stop Event, so there is never ambiguity
     about which thread should be running.
"""

import time
import threading
import math
from pydualsense import pydualsense, PlayerID
from .harmonic_strummer import HarmonicStrummer

from config.mappings import CC_MAP, NRPN_MAP, NRPN_MOTION_THRESHOLD, MOTION_THRESHOLD, MOTION_SMOOTHING, TILT_DEADZONE_14BIT, GYRO_DEADZONE_14BIT, LONG_PRESS_DURATION


class MIDIController:
    def __init__(self, channel_manager, clock_source=None):
        self.channel_mgr = channel_manager
        self.clock_source = clock_source  # ClockSource reference for sync state queries  # Reference to the channel manager
        self.last_cc_values = {}  # Track last sent CC values
        self.last_motion_raw = {'tilt_x': 0, 'tilt_y': 0, 'twist': 0}
        self.smoothed_motion = {'tilt_x': 64, 'tilt_y': 64, 'twist': 64}

        # 14-bit motion smoothing state (for NRPN sends, center = 8192)
        self.smoothed_motion_14bit = {'tilt_x': 8192, 'tilt_y': 8192, 'twist': 8192}
        self.last_nrpn_values = {}  # Track last sent NRPN values for threshold gating
        self.active_notes = {}
        self.touchpad_active = False  # Track if finger is on touchpad

        # Touchpad-based loop controls
        self.touchpad_x = 0      # 0-1920
        self.touchpad_y = 0      # 0-1080
        self.touchpad_active = False
        self.window_position = 0.0    # 0.0 to 1.0 — scrub position via touchpad X
        self.window_size = 1.0        # Fixed at 1.0 (full loop); touchpad Y now controls quantization

        # Quantize toggle state (touchpad click = on/off)
        self.quantize_on = False
        self.last_quantize_subdivision = 4  # Default to 1/4 when first enabled
        self._last_quantize_toggle_time = 0.0  # Debounce: ignore Y changes right after click

        # LEFT STICK SLICE BANKING
        self.slice_bank = 0           # Current bank (0-7 for 128 slices total)
        self.slices_per_bank = 16     # Slices per bank
        self.max_banks = 7            # 8 banks total (0-7) = 128 MIDI values

        # Motion control toggle and loop recording
        # Per-channel: one flag per channel so enabling motion on ch1 doesn't
        # affect ch2/ch3.
        self.motion_enabled = [False, False, False]

        # Combo guard: set True when L1+R1 fires check_motion_toggle() so the
        # second button to be released doesn't also trigger its solo action.
        self.l1_r1_combo_used = False

        self.l1_pressed = False
        self.r1_pressed = False
        self.l1_press_time = 0
        self.r1_press_time = 0

        # Movement-based haptics
        self.previous_tilt_x = 64
        self.previous_tilt_y = 64
        self.last_haptic_pulse = 0
        self.haptic_pulse_duration = 0.1
        self.haptic_active = False

        # Channel switching with START/SELECT
        self.current_channel = 1  # Default to channel 1
        self.select_pressed = False  # BTN_SELECT (Create/Share)
        self.start_pressed = False   # BTN_START (Options)

        # Write initial channel so passthrough starts in sync
        try:
            with open('/tmp/dualsense_channel', 'w') as f:
                f.write('1')
        except Exception:
            pass

        # Button hold tracking for repeated CC messages (MIDI learn support)
        # All four face buttons behave identically: press → CC 127, release → CC 0
        self.btn_south_held = False  # X (✕)
        self.btn_east_held = False   # O (○)
        self.btn_north_held = False  # △
        self.btn_west_held = False   # □
        self.last_btn_send_time = {'south': 0, 'east': 0, 'north': 0, 'west': 0}
        self.btn_repeat_interval = 0.05  # Send CC every 50ms while held

        # Sequencer control - L2 as modifier
        self.l2_held = False
        self.l2_modifier_threshold = 200  # Value where L2 acts as modifier (0-255)

        # --- FIX: per-thread stop event instead of shared led_pulse_active flag ---
        # Each call to start_led_pulse() creates a fresh Event and passes it to
        # the new thread. stop_led_pulse() sets _current_pulse_stop, which only
        # the currently-live thread holds a reference to - so there is no race
        # between an old thread reading led_pulse_active and a new thread setting it.
        self.led_pulse_thread = None
        self.led_pulse_active = False           # kept for external state checks
        self._current_pulse_stop = threading.Event()

        # Create pydualsense controller for LED control
        self.ds = pydualsense()
        self.ds.init()

        # PERF: pydualsense.sendReport() is a single thread that both reads the
        # HID report from hidraw0 AND writes LED/haptic state back to it. It calls
        # readInput() 250x/sec to parse the full controller state into Python objects.
        # We use evdev for all input — we never read pydualsense's parsed state —
        # so those 250 parse cycles per second are pure waste, and the short-lived
        # objects they create are the source of the ~8 MB/min heap growth we measured.
        #
        # Fix: replace readInput with a no-op lambda. The sendReport thread keeps
        # running (so LED setColorI() and setLeftMotor() writes still work), and
        # device.read() still drains the kernel HID buffer, but no Python state
        # objects are built or discarded.
        self.ds.readInput = lambda inReport: None

        # Harmonic strummer for PlayStation button
        self.strummer = HarmonicStrummer(
            strum_delay_ms=80,     # Fast strumming
            notes_per_chord=4      # 4-note chords
        )

        # Set initial LED color
        self.update_led_color()

    def scale_value(self, value, in_min, in_max, out_min=0, out_max=127):
        """Scale input value to MIDI range (0-127)"""
        value = max(in_min, min(in_max, value))  # Clamp
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def calculate_slice_with_bank(self, stick_value):
        """Convert stick position (0-255) to absolute slice number with banking."""
        slice_in_bank = int((stick_value / 255.0) * (self.slices_per_bank - 1))
        absolute_slice = (self.slice_bank * self.slices_per_bank) + slice_in_bank
        midi_value = min(absolute_slice, 127)
        return midi_value

    def handle_dpad_left(self):
        """Decrease slice bank (left arrow)"""
        if self.slice_bank > 0:
            self.slice_bank -= 1
            start_slice = self.slice_bank * self.slices_per_bank + 1
            end_slice = (self.slice_bank + 1) * self.slices_per_bank
            print(f"\n⬅️  SLICE BANK {self.slice_bank}: Slices {start_slice}-{end_slice}\n")
            self.ds.setRightMotor(100)
            time.sleep(0.05)
            self.ds.setRightMotor(0)
        else:
            print(f"\n⬅️  Already at bank 0 (slices 1-16)\n")

    def handle_dpad_right(self):
        """Increase slice bank (right arrow)"""
        if self.slice_bank < self.max_banks:
            self.slice_bank += 1
            start_slice = self.slice_bank * self.slices_per_bank + 1
            end_slice = (self.slice_bank + 1) * self.slices_per_bank
            print(f"\n➡️  SLICE BANK {self.slice_bank}: Slices {start_slice}-{end_slice}\n")
            self.ds.setLeftMotor(100)
            time.sleep(0.05)
            self.ds.setLeftMotor(0)
        else:
            print(f"\n➡️  Already at bank {self.max_banks} (slices {self.max_banks * 16 + 1}-128)\n")

    def apply_deadzone(self, value, center=127, deadzone=10):
        """Apply deadzone around center position"""
        centered = value - center
        if abs(centered) < deadzone:
            return 0
        if centered > 0:
            return self.scale_value(centered - deadzone, 0, 127 - deadzone, 64, 127)
        else:
            return self.scale_value(centered + deadzone, -(127 - deadzone), 0, 0, 63)

    def smooth_motion(self, raw_value, key, smoothing=0.3):
        """Apply exponential smoothing to motion values (7-bit, 0-127)"""
        self.last_motion_raw[key] = raw_value
        self.smoothed_motion[key] = (smoothing * raw_value +
                                     (1 - smoothing) * self.smoothed_motion[key])
        return int(self.smoothed_motion[key])

    def smooth_motion_14bit(self, raw_14bit, key, smoothing=0.3):
        """Apply exponential smoothing in 14-bit space (0-16383, center 8192).
        Smoothing before quantization gives genuine sub-LSB averaging and
        eliminates the staircasing you get when upscaling 7-bit smoothed values.
        """
        self.smoothed_motion_14bit[key] = (
            smoothing * raw_14bit +
            (1 - smoothing) * self.smoothed_motion_14bit[key]
        )
        return int(self.smoothed_motion_14bit[key])

    def should_send_nrpn(self, param, value):
        """Gate NRPN sends: only send when value changed by more than threshold."""
        if param not in self.last_nrpn_values:
            self.last_nrpn_values[param] = value
            return True
        if abs(value - self.last_nrpn_values[param]) >= NRPN_MOTION_THRESHOLD:
            self.last_nrpn_values[param] = value
            return True
        return False

    def send_nrpn(self, midiout, param, value_14bit):
        """Send a 14-bit NRPN value on the current MIDI channel.

        NRPN 4-message sequence (all on same channel):
          CC 99 = NRPN parameter MSB  (param >> 7)
          CC 98 = NRPN parameter LSB  (param & 0x7F)
          CC 6  = value MSB           (value >> 7)
          CC 38 = value LSB           (value & 0x7F)
        """
        ch = self.get_midi_channel_byte(0xB0)
        p_msb = (param >> 7) & 0x7F
        p_lsb = param & 0x7F
        v_msb = (value_14bit >> 7) & 0x7F
        v_lsb = value_14bit & 0x7F
        midiout.send_message([ch, 99, p_msb])
        midiout.send_message([ch, 98, p_lsb])
        midiout.send_message([ch,  6, v_msb])
        midiout.send_message([ch, 38, v_lsb])

    def should_send_cc(self, cc_num, value):
        """Check if CC value changed enough to send"""
        if cc_num not in self.last_cc_values:
            self.last_cc_values[cc_num] = value
            return True
        if value != self.last_cc_values[cc_num]:
            self.last_cc_values[cc_num] = value
            return True
        return False

    def update_led_color(self):
        """Update LED based on current channel, quantize state, motion state, and loop state"""
        loop_state = self.channel_mgr.get_current_loop_state()

        # Recording states take top priority (always want to see record state)
        if loop_state.recording and loop_state.playing:
            self.start_led_pulse(255, 0, 255)   # Purple: overdub
            return
        if loop_state.recording:
            self.start_led_pulse(255, 0, 0)     # Red: recording
            return

        # Quantize active: GREEN = locked to NerdSEQ, channel color = internal/free
        if self.quantize_on:
            synced = self.clock_source is not None and self.clock_source.is_synced_external()
            if synced:
                self.start_led_pulse(0, 255, 0)         # Green pulse: locked to NerdSEQ
            else:
                if self.current_channel == 1:
                    self.start_led_pulse(150, 150, 150)  # White blink: internal clock
                elif self.current_channel == 2:
                    self.start_led_pulse(0, 150, 150)    # Turquoise blink: internal clock
                elif self.current_channel == 3:
                    self.start_led_pulse(150, 150, 0)    # Yellow blink: internal clock
            return

        # Playing free-running (no quantize): channel color pulse — NOT green
        # Green is reserved exclusively for "quantize locked to NerdSEQ"
        if loop_state.playing:
            if self.current_channel == 1:
                self.start_led_pulse(200, 200, 200)  # Bright white pulse
            elif self.current_channel == 2:
                self.start_led_pulse(0, 200, 200)    # Bright turquoise pulse
            elif self.current_channel == 3:
                self.start_led_pulse(200, 200, 0)    # Bright yellow pulse
            return
        if self.is_motion_enabled():
            self.stop_led_pulse()
            self.ds.light.setColorI(0, 100, 255)
            return

        # Idle: base channel colour
        self.stop_led_pulse()
        if self.current_channel == 1:
            self.ds.light.setColorI(50, 50, 50)    # Dim white
        elif self.current_channel == 2:
            self.ds.light.setColorI(0, 80, 80)     # Dim turquoise
        elif self.current_channel == 3:
            self.ds.light.setColorI(80, 80, 0)     # Dim yellow

    def start_led_pulse(self, r, g, b):
        """Start pulsing LED in background thread.

        FIX: Non-blocking. Signals the old thread to stop (it exits on its own
        since it's a daemon and checks stop_event), then immediately starts a
        new thread. No join() → main event loop never blocks waiting for LED.
        """
        # Signal old thread to exit (it checks stop_event before each HID write)
        self._current_pulse_stop.set()
        self.led_pulse_active = True

        stop_event = threading.Event()
        self._current_pulse_stop = stop_event

        self.led_pulse_thread = threading.Thread(
            target=self._led_pulse_loop,
            args=(r, g, b, stop_event),
            daemon=True
        )
        self.led_pulse_thread.start()

    def stop_led_pulse(self):
        """Signal LED pulse thread to stop. Non-blocking — daemon thread exits naturally.

        FIX: Removed join(). The old code blocked the main thread for up to 500ms
        on every LED state change (update_led_color → stop_led_pulse → join).
        Since the thread is a daemon and checks its stop_event each iteration,
        it exits within one 50ms cycle without needing the main thread to wait.
        """
        self._current_pulse_stop.set()
        self.led_pulse_active = False
        self.led_pulse_thread = None

    def _led_pulse_loop(self, r, g, b, stop_event):
        """Background thread for pulsing LED.

        FIX: Uses its own stop_event argument (not self.led_pulse_active) so it
        exits cleanly even if a new thread has already set led_pulse_active=True.
        """
        while not stop_event.is_set():
            t = time.time()
            brightness = (math.sin(t * math.pi) + 1) / 2  # 0 to 1
            brightness = max(0.2, brightness)               # min 20% brightness

            self.ds.light.setColorI(
                int(r * brightness),
                int(g * brightness),
                int(b * brightness)
            )

            stop_event.wait(timeout=0.2)  # 5Hz — 4× less USB HID writes (was 20Hz/50ms, caused 35% CPU on LED thread)

    def send_held_button_ccs(self, midiout):
        """Send repeated CC messages for held buttons (helps MIDI learn)"""
        current_time = time.time()

        for key, held, cc_key, label in [
            ('south', self.btn_south_held, 'btn_south', 'X (✕)'),
            ('east',  self.btn_east_held,  'btn_east',  'O (○)'),
            ('north', self.btn_north_held, 'btn_north', '△'),
            ('west',  self.btn_west_held,  'btn_west',  '□'),
        ]:
            if held:
                if current_time - self.last_btn_send_time[key] >= self.btn_repeat_interval:
                    status_byte = self.get_midi_channel_byte(0xB0)
                    midi_msg = [status_byte, CC_MAP[cc_key], 127]
                    midiout.send_message(midi_msg)

                    loop_state = self.channel_mgr.get_current_loop_state()
                    if loop_state.recording:
                        loop_state.record_message(midi_msg)

                    self.last_btn_send_time[key] = current_time
                    if int(current_time * 10) % 5 == 0:
                        print(f"🎚️  {label} HELD → CC{CC_MAP[cc_key]:2d}: 127 (Ch {self.current_channel})")

    def _write_current_channel(self):
        """Write current channel to shared file so passthrough can remap pedal messages."""
        try:
            with open('/tmp/dualsense_channel', 'w') as f:
                f.write(str(self.current_channel))
        except Exception:
            pass

    def check_channel_switch(self, midiout):
        """Check for START/SELECT button combinations to switch channels"""
        if self.select_pressed and self.start_pressed:
            if self.current_channel != 3:
                self.current_channel = 3
                self.channel_mgr.current_channel = 3
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(2)
                self.update_led_color()
                self._write_current_channel()
                print(f"\n🎛️  SWITCHED TO CHANNEL 3 (Yellow) 🟡")
                self.channel_mgr.send_frozen_values_on_channel_switch(midiout, self, CC_MAP)
                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.playing:
                    print(f"  🔄 Loop playing ({loop_state.loop_duration:.1f}s)")
                elif loop_state.midi_buffer:
                    print(f"  ⏸️  Loop ready ({loop_state.loop_duration:.1f}s)")
                print()

        elif self.select_pressed and not self.start_pressed:
            if self.current_channel != 1:
                self.current_channel = 1
                self.channel_mgr.current_channel = 1
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(0)
                self.update_led_color()
                self._write_current_channel()
                print(f"\n🎛️  SWITCHED TO CHANNEL 1 (White) ⚪")
                self.channel_mgr.send_frozen_values_on_channel_switch(midiout, self, CC_MAP)
                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.playing:
                    print(f"  🔄 Loop playing ({loop_state.loop_duration:.1f}s)")
                elif loop_state.midi_buffer:
                    print(f"  ⏸️  Loop ready ({loop_state.loop_duration:.1f}s)")
                print()

        elif self.start_pressed and not self.select_pressed:
            if self.current_channel != 2:
                self.current_channel = 2
                self.channel_mgr.current_channel = 2
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(1)
                self.update_led_color()
                self._write_current_channel()
                print(f"\n🎛️  SWITCHED TO CHANNEL 2 (Green) 🟢")
                self.channel_mgr.send_frozen_values_on_channel_switch(midiout, self, CC_MAP)
                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.playing:
                    print(f"  🔄 Loop playing ({loop_state.loop_duration:.1f}s)")
                elif loop_state.midi_buffer:
                    print(f"  ⏸️  Loop ready ({loop_state.loop_duration:.1f}s)")
                print()

    def get_midi_channel_byte(self, message_type):
        """Get MIDI status byte with current channel (channels are 0-indexed in MIDI)"""
        return message_type + (self.current_channel - 1)

    def is_motion_enabled(self):
        """Return motion-enabled state for the current channel."""
        return self.motion_enabled[self.current_channel - 1]

    def check_motion_toggle(self):
        """Check if L1+R1 pressed together to toggle motion"""
        if self.l1_pressed and self.r1_pressed:
            idx = self.current_channel - 1
            self.motion_enabled[idx] = not self.motion_enabled[idx]
            status = "ENABLED ✅" if self.motion_enabled[idx] else "DISABLED ❌"

            if self.motion_enabled[idx]:
                self.tilt_baseline_x = self.smoothed_motion['tilt_x']
                self.tilt_baseline_y = self.smoothed_motion['tilt_y']
                print(f"\n🎯 CALIBRATED: Baseline X={self.tilt_baseline_x:.1f}, Y={self.tilt_baseline_y:.1f}")

            print(f"\n🎛️  MOTION CONTROL {status} on Ch {self.current_channel} (L1+R1)\n")

            # Signal that a combo was handled — whichever button is released
            # second should NOT also fire its solo action.
            self.l1_r1_combo_used = True

        self.update_led_color()

        if not self.is_motion_enabled():
            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)

        return True

    def update_haptics_from_tilt(self, tilt_x_cc, tilt_y_cc):
        """Directional rumble: left/right for X-axis, both motors for Y-axis"""
        if not self.is_motion_enabled():
            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)
            return

        EXTREME_ZONE_START = 55
        rumble_left = 0
        rumble_right = 0

        if tilt_x_cc < 64:
            distance = 64 - tilt_x_cc
            if distance > EXTREME_ZONE_START:
                rumble_left = int(min(255, (distance - EXTREME_ZONE_START) * 20))
        else:
            distance = tilt_x_cc - 64
            if distance > EXTREME_ZONE_START:
                rumble_right = int(min(255, (distance - EXTREME_ZONE_START) * 20))

        y_distance = abs(tilt_y_cc - 64)
        if y_distance > EXTREME_ZONE_START:
            y_rumble = int(min(255, (y_distance - EXTREME_ZONE_START) * 20))
            rumble_left = min(255, rumble_left + y_rumble)
            rumble_right = min(255, rumble_right + y_rumble)

        self.ds.setLeftMotor(rumble_left)
        self.ds.setRightMotor(rumble_right)

    def toggle_quantize(self):
        """Toggle quantization on/off (touchpad click). Remembers last subdivision."""
        loop_state = self.channel_mgr.get_current_loop_state()
        self.quantize_on = not self.quantize_on
        self._last_quantize_toggle_time = time.time()  # Start debounce window

        if self.quantize_on:
            loop_state.quantize_subdivision = self.last_quantize_subdivision
            sub_str = f"1/{self.last_quantize_subdivision}"
            print(f"\n🎵 Quantize: ON ({sub_str})\n")
        else:
            # Save the last used subdivision before turning off
            if loop_state.quantize_subdivision is not None:
                self.last_quantize_subdivision = loop_state.quantize_subdivision
            loop_state.quantize_subdivision = None
            print(f"\n🎵 Quantize: OFF\n")

        self.update_player_dots()
        self.update_led_color()

    def update_player_dots(self):
        """Show quantize state on the 4 player indicator dots beneath the touchpad.

        Dot count = subdivision coarseness:
          0 dots = quantize OFF
          1 dot  = 1/4   (coarsest)
          2 dots = 1/8
          3 dots = 1/16
          4 dots = 1/32  (finest)

        FIX: Use self.ds.light.setPlayerID() — setPlayerID lives on DSLight, not
        on the top-level pydualsense object. The old self.ds.setPlayerID() was
        throwing AttributeError silently caught by except, so dots never changed.
        """
        if not self.quantize_on:
            try:
                self.ds.light.setPlayerID(PlayerID(0))
            except Exception as e:
                print(f"⚠️  Player dots clear failed: {e}")
            return

        sub_to_dots = {4: PlayerID.PLAYER_1, 8: PlayerID.PLAYER_2,
                       16: PlayerID.PLAYER_3, 32: PlayerID.PLAYER_4}
        dot_id = sub_to_dots.get(self.last_quantize_subdivision, PlayerID.PLAYER_1)
        try:
            self.ds.light.setPlayerID(dot_id)
        except Exception as e:
            print(f"⚠️  Player dots update failed: {e}")

    def update_touchpad(self, x, y, is_active):
        """
        Update touchpad position.

        X → loop scrub position (0.0-1.0 through the loop)
        Y → quantization subdivision selector (while quantize is ON):
              0- 216  (zone 0) = 1/32
            216- 432  (zone 1) = 1/16
            432- 648  (zone 2) = 1/8
            648-1080  (zone 3) = 1/4
        Touchpad click → toggle quantize on/off (via toggle_quantize())
        Finger lift → reset scrub position only; quantize state preserved
        """
        self.touchpad_x = x
        self.touchpad_y = y
        self.touchpad_active = is_active

        loop_state = self.channel_mgr.get_current_loop_state()

        if not is_active:
            self.window_position = 0.0
            # Quantize stays on/off as it was — only touchpad click toggles it
            return

        if not loop_state.playing or loop_state.loop_duration == 0:
            return

        # X → scrub position
        self.window_position = x / 1920.0

        # Y → quantization subdivision — skip for 300ms after a click toggle
        # (click re-registers as a new touch, which would immediately overwrite subdivision)
        if (time.time() - self._last_quantize_toggle_time) < 0.3:
            return

        # 4 equal zones over 1080px
        SUBDIVISIONS = [32, 16, 8, 4]  # top→bottom
        zone = min(int(y / 270), 3)    # 1080 / 4 zones = 270px per zone
        new_sub = SUBDIVISIONS[zone]

        if new_sub != self.last_quantize_subdivision:
            self.last_quantize_subdivision = new_sub
            if self.quantize_on:
                loop_state.quantize_subdivision = new_sub
            self.update_player_dots()
            print(f"🎵 Quantize subdivision: 1/{new_sub} (zone {zone})")

        if int(time.time() * 4) % 2 == 0:
            q_str = f"ON (1/{self.last_quantize_subdivision})" if self.quantize_on else f"OFF (last: 1/{self.last_quantize_subdivision})"
            print(f"👆 Touchpad: Scrub={self.window_position*100:.1f}% Quantize={q_str}")

    def cleanup(self):
        """Properly shut down controller and haptics"""
        try:
            for loop_state in self.channel_mgr.loop_states:
                loop_state.stop_playback()

            self.stop_led_pulse()

            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)
            self.ds.light.setColorI(0, 0, 0)
            try:
                self.ds.light.setPlayerID(PlayerID(0))   # Clear player dots
            except Exception:
                pass
            self.ds.close()

            print("🎮 Controller cleaned up")
        except Exception as e:
            print(f"⚠️  Cleanup error (non-critical): {e}")
