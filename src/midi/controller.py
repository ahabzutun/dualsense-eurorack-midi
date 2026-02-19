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
from pydualsense import pydualsense
from .harmonic_strummer import HarmonicStrummer

from config.mappings import CC_MAP, MOTION_THRESHOLD, MOTION_SMOOTHING, TILT_DEADZONE, GYRO_DEADZONE, LONG_PRESS_DURATION


class MIDIController:
    def __init__(self, channel_manager):
        self.channel_mgr = channel_manager  # Reference to the channel manager
        self.last_cc_values = {}  # Track last sent CC values
        self.last_motion_raw = {'tilt_x': 0, 'tilt_y': 0, 'twist': 0}
        self.smoothed_motion = {'tilt_x': 64, 'tilt_y': 64, 'twist': 64}
        self.active_notes = {}
        self.touchpad_active = False  # Track if finger is on touchpad

        # Touchpad-based loop scanning
        self.touchpad_x = 0      # 0-1920
        self.touchpad_y = 0      # 0-1080
        self.touchpad_active = False
        self.window_position = 0.0    # 0.0 to 1.0 (percentage of loop)
        self.window_size = 1.0        # 0.0 to 1.0 (percentage of loop)
        self.min_window_ms = 5        # Minimum window for glitchy sounds

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
        """Apply exponential smoothing to motion values"""
        self.last_motion_raw[key] = raw_value
        self.smoothed_motion[key] = (smoothing * raw_value +
                                     (1 - smoothing) * self.smoothed_motion[key])
        return int(self.smoothed_motion[key])

    def should_send_cc(self, cc_num, value):
        """Check if CC value changed enough to send"""
        if cc_num not in self.last_cc_values:
            self.last_cc_values[cc_num] = value
            return True

        if cc_num in [CC_MAP['tilt_x'], CC_MAP['tilt_y'], CC_MAP['twist']]:
            if abs(value - self.last_cc_values[cc_num]) >= MOTION_THRESHOLD:
                self.last_cc_values[cc_num] = value
                return True
            return False

        if value != self.last_cc_values[cc_num]:
            self.last_cc_values[cc_num] = value
            return True
        return False

    def update_led_color(self):
        """Update LED based on current channel, motion state, and loop state"""
        self.stop_led_pulse()

        loop_state = self.channel_mgr.get_current_loop_state()

        if hasattr(self, 'sequencer_manager'):
            seq = self.sequencer_manager.get_current_sequencer()
            if seq and seq.enabled:
                if not seq.quantized:
                    self.start_led_pulse(255, 255, 0)
                    return
                elif seq.pattern.value == '4/4':
                    self.start_led_pulse(0, 255, 0)
                    return
                elif seq.pattern.value == 'triplet':
                    self.start_led_pulse(255, 0, 255)
                    return
                else:
                    self.start_led_pulse(0, 255, 255)
                    return

        if loop_state.recording and loop_state.playing:
            self.start_led_pulse(255, 0, 255)  # Purple pulse
        elif loop_state.recording:
            self.start_led_pulse(255, 0, 0)    # Red pulse
        elif loop_state.playing:
            self.start_led_pulse(0, 255, 0)    # Green pulse
        elif self.is_motion_enabled():
            self.ds.light.setColorI(0, 100, 255)
        else:
            if self.current_channel == 1:
                self.ds.light.setColorI(50, 50, 50)   # Dim white
            elif self.current_channel == 2:
                self.ds.light.setColorI(0, 80, 0)     # Dim green
            elif self.current_channel == 3:
                self.ds.light.setColorI(80, 80, 0)    # Dim yellow

    def start_led_pulse(self, r, g, b):
        """Start pulsing LED in background thread.

        FIX: Each invocation creates a fresh stop Event and passes it directly
        to the new thread. The old thread gets its own stop_event set via
        stop_led_pulse(), so it exits cleanly regardless of what the new thread
        does with led_pulse_active. No more zombie LED threads.
        """
        self.stop_led_pulse()  # cleanly stop old thread first

        stop_event = threading.Event()
        self._current_pulse_stop = stop_event   # save ref so stop_led_pulse can reach it
        self.led_pulse_active = True

        self.led_pulse_thread = threading.Thread(
            target=self._led_pulse_loop,
            args=(r, g, b, stop_event),          # thread owns its stop_event
            daemon=True
        )
        self.led_pulse_thread.start()

    def stop_led_pulse(self):
        """Stop LED pulsing and wait for the thread to actually exit."""
        if self.led_pulse_active:
            self.led_pulse_active = False
            self._current_pulse_stop.set()       # signal the currently-live thread
            if self.led_pulse_thread and self.led_pulse_thread.is_alive():
                self.led_pulse_thread.join(timeout=0.5)
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

            stop_event.wait(timeout=0.05)  # 50ms update rate

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

    def check_channel_switch(self, midiout):
        """Check for START/SELECT button combinations to switch channels"""
        if self.select_pressed and self.start_pressed:
            if self.current_channel != 3:
                self.current_channel = 3
                self.channel_mgr.current_channel = 3
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(2)
                self.update_led_color()
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

    def update_touchpad(self, x, y, is_active):
        """Update touchpad position and calculate loop window"""
        self.touchpad_x = x
        self.touchpad_y = y
        self.touchpad_active = is_active

        if not is_active:
            self.window_position = 0.0
            self.window_size = 1.0
            return

        loop_state = self.channel_mgr.get_current_loop_state()
        if not loop_state.playing or loop_state.loop_duration == 0:
            return

        self.window_position = x / 1920.0

        min_window_size = self.min_window_ms / (loop_state.loop_duration * 1000.0)
        min_window_size = max(0.001, min(min_window_size, 1.0))

        normalized_y = y / 1080.0
        self.window_size = min_window_size + (normalized_y * (1.0 - min_window_size))

        if int(time.time() * 4) % 2 == 0:
            print(f"🎚️  Touchpad: Pos={self.window_position*100:.1f}% Size={self.window_size*100:.1f}%")

    def cleanup(self):
        """Properly shut down controller and haptics"""
        try:
            for loop_state in self.channel_mgr.loop_states:
                loop_state.stop_playback()

            self.stop_led_pulse()

            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)
            self.ds.light.setColorI(0, 0, 0)
            self.ds.close()

            print("🎮 Controller cleaned up")
        except Exception as e:
            print(f"⚠️  Cleanup error (non-critical): {e}")
