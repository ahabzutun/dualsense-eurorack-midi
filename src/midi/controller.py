"""
MIDI Controller for DualSense
Handles MIDI message generation, LED control, haptics, and channel switching
"""

import time
import threading
import math
from pydualsense import pydualsense

from config.mappings import CC_MAP, MOTION_THRESHOLD, MOTION_SMOOTHING, TILT_DEADZONE, GYRO_DEADZONE


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
        self.touchpad_active = False  # Already exists
        self.window_position = 0.0    # 0.0 to 1.0 (percentage of loop)
        self.window_size = 1.0        # 0.0 to 1.0 (percentage of loop)
        self.min_window_ms = 5        # Minimum window for glitchy sounds

        # LEFT STICK SLICE BANKING
        self.slice_bank = 0           # Current bank (0-7 for 128 slices total)
        self.slices_per_bank = 16     # Slices per bank
        self.max_banks = 7            # 8 banks total (0-7) = 128 MIDI values

        # Motion control toggle and loop recording
        self.motion_enabled = False
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
        self.btn_south_held = False  # X button
        self.btn_east_held = False   # O button
        self.last_btn_send_time = {'south': 0, 'east': 0}
        self.btn_repeat_interval = 0.05  # Send CC every 50ms while held

        # Sequencer control - L2 as modifier
        self.l2_held = False
        self.l2_modifier_threshold = 200  # Value where L2 acts as modifier (0-255)

        # Loop recording - Triangle button timing
        self.triangle_pressed = False
        self.triangle_press_time = 0
        self.square_pressed = False  # For clear combo

        # LED pulsing for loop feedback
        self.led_pulse_thread = None
        self.led_pulse_active = False
        self.led_pulse_stop_event = threading.Event()

        # Create pydualsense controller for LED control
        self.ds = pydualsense()
        self.ds.init()

        # Set initial LED color
        self.update_led_color()

    def scale_value(self, value, in_min, in_max, out_min=0, out_max=127):
        """Scale input value to MIDI range (0-127)"""
        value = max(in_min, min(in_max, value))  # Clamp
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def calculate_slice_with_bank(self, stick_value):
        """
        Convert stick position (0-255) to absolute slice number with banking.

        Args:
            stick_value: Raw stick value 0-255
        Returns:
            MIDI CC value (0-127) representing the slice
        """
        # Map stick position to 0-15 range within the current bank
        slice_in_bank = int((stick_value / 255.0) * (self.slices_per_bank - 1))

        # Add the bank offset
        absolute_slice = (self.slice_bank * self.slices_per_bank) + slice_in_bank

        # Constrain to MIDI range (0-127)
        midi_value = min(absolute_slice, 127)

        return midi_value

    def handle_dpad_left(self):
        """Decrease slice bank (left arrow)"""
        if self.slice_bank > 0:
            self.slice_bank -= 1
            start_slice = self.slice_bank * self.slices_per_bank + 1
            end_slice = (self.slice_bank + 1) * self.slices_per_bank
            print(f"\n⬅️  SLICE BANK {self.slice_bank}: Slices {start_slice}-{end_slice}\n")

            # Optional: Give haptic feedback
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

            # Optional: Give haptic feedback
            self.ds.setLeftMotor(100)
            time.sleep(0.05)
            self.ds.setLeftMotor(0)
        else:
            print(f"\n➡️  Already at bank {self.max_banks} (slices {self.max_banks * 16 + 1}-128)\n")

    def apply_deadzone(self, value, center=127, deadzone=10):
        """Apply deadzone around center position"""
        # Convert 0-255 range to -127 to 127
        centered = value - center

        # Apply deadzone
        if abs(centered) < deadzone:
            return 0

        # Scale back to 0-127 MIDI range
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

        # For motion CCs, use threshold
        if cc_num in [CC_MAP['tilt_x'], CC_MAP['tilt_y'], CC_MAP['twist']]:
            if abs(value - self.last_cc_values[cc_num]) >= MOTION_THRESHOLD:
                self.last_cc_values[cc_num] = value
                return True
            return False

        # For other CCs, send if different
        if value != self.last_cc_values[cc_num]:
            self.last_cc_values[cc_num] = value
            return True
        return False

    def update_led_color(self):
        """Update LED based on current channel, motion state, and loop state"""
        # Stop any active pulsing
        self.stop_led_pulse()

        loop_state = self.channel_mgr.get_current_loop_state()

        if hasattr(self, 'sequencer_manager'):
            seq = self.sequencer_manager.get_current_sequencer()
            if seq and seq.enabled:
                # Sequencer is running - show pattern-specific colors
                if not seq.quantized:
                    # Free-running: Yellow pulse (fast)
                    self.start_led_pulse(255, 255, 0)
                    return
                elif seq.pattern.value == '4/4':
                    # 4/4: Green pulse
                    self.start_led_pulse(0, 255, 0)
                    return
                elif seq.pattern.value == 'triplet':
                    # Triplet: Purple pulse
                    self.start_led_pulse(255, 0, 255)
                    return
                else:
                    # Other patterns: Cyan pulse
                    self.start_led_pulse(0, 255, 255)
                    return

        if loop_state.recording and loop_state.playing:
            # Recording while playing - shouldn't happen but just in case
            self.start_led_pulse(255, 0, 255)  # Purple pulse
        elif loop_state.recording:
            # Recording - Red pulse
            self.start_led_pulse(255, 0, 0)
        elif loop_state.playing:
            # Playing - Green pulse
            self.start_led_pulse(0, 255, 0)
        elif self.motion_enabled:
            # Motion mode - bright cyan (solid)
            self.ds.light.setColorI(0, 100, 255)
        else:
            # Show channel color when motion/loop is off
            if self.current_channel == 1:
                self.ds.light.setColorI(50, 50, 50)  # Dim white
            elif self.current_channel == 2:
                self.ds.light.setColorI(0, 80, 0)    # Dim green
            elif self.current_channel == 3:
                self.ds.light.setColorI(80, 80, 0)   # Dim yellow

    def start_led_pulse(self, r, g, b):
        """Start pulsing LED in background thread"""
        if self.led_pulse_active:
            self.stop_led_pulse()

        self.led_pulse_active = True
        self.led_pulse_stop_event.clear()
        self.led_pulse_thread = threading.Thread(
            target=self._led_pulse_loop,
            args=(r, g, b),
            daemon=True
        )
        self.led_pulse_thread.start()

    def stop_led_pulse(self):
        """Stop LED pulsing"""
        if self.led_pulse_active:
            self.led_pulse_active = False
            self.led_pulse_stop_event.set()
            if self.led_pulse_thread:
                self.led_pulse_thread.join(timeout=0.5)

    def _led_pulse_loop(self, r, g, b):
        """Background thread for pulsing LED"""
        while self.led_pulse_active and not self.led_pulse_stop_event.is_set():
            # Sine wave pulse (0.5 Hz - 2 second period)
            t = time.time()
            brightness = (math.sin(t * math.pi) + 1) / 2  # 0 to 1
            brightness = max(0.2, brightness)  # Min 20% brightness

            self.ds.light.setColorI(
                int(r * brightness),
                int(g * brightness),
                int(b * brightness)
            )

            self.led_pulse_stop_event.wait(timeout=0.05)  # 50ms update rate

    def send_held_button_ccs(self, midiout):
        """Send repeated CC messages for held buttons (helps MIDI learn)"""
        current_time = time.time()

        # X button (south)
        if self.btn_south_held:
            if current_time - self.last_btn_send_time['south'] >= self.btn_repeat_interval:
                status_byte = self.get_midi_channel_byte(0xB0)
                midi_msg = [status_byte, CC_MAP['btn_south'], 127]
                midiout.send_message(midi_msg)

                # Record to loop if recording
                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.recording:
                    loop_state.record_message(midi_msg)

                self.last_btn_send_time['south'] = current_time
                # Only print occasionally to avoid spam
                if int(current_time * 10) % 5 == 0:  # Print every ~500ms
                    print(f"🎚️  X (✕) HELD → CC{CC_MAP['btn_south']:2d}: 127 (Ch {self.current_channel})")

        # O button (east)
        if self.btn_east_held:
            if current_time - self.last_btn_send_time['east'] >= self.btn_repeat_interval:
                status_byte = self.get_midi_channel_byte(0xB0)
                midi_msg = [status_byte, CC_MAP['btn_east'], 127]
                midiout.send_message(midi_msg)

                # Record to loop if recording
                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.recording:
                    loop_state.record_message(midi_msg)

                self.last_btn_send_time['east'] = current_time
                # Only print occasionally to avoid spam
                if int(current_time * 10) % 5 == 0:  # Print every ~500ms
                    print(f"🎚️  O (○) HELD → CC{CC_MAP['btn_east']:2d}: 127 (Ch {self.current_channel})")

    def check_channel_switch(self, midiout):
        """Check for START/SELECT button combinations to switch channels"""
        # Both pressed = Channel 3
        if self.select_pressed and self.start_pressed:
            if self.current_channel != 3:
                self.current_channel = 3
                self.channel_mgr.current_channel = 3  # Sync with channel manager
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(2)  # 0-indexed
                self.update_led_color()
                print(f"\n🎛️  SWITCHED TO CHANNEL 3 (Yellow) 🟡")
                # Send frozen values for this channel
                self.channel_mgr.send_frozen_values_on_channel_switch(midiout, self, CC_MAP)

                # Show loop status
                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.playing:
                    print(f"  🔄 Loop playing ({loop_state.loop_duration:.1f}s)")
                elif loop_state.midi_buffer:
                    print(f"  ⏸️  Loop ready ({loop_state.loop_duration:.1f}s)")
                print()

        # Only SELECT pressed = Channel 1
        elif self.select_pressed and not self.start_pressed:
            if self.current_channel != 1:
                self.current_channel = 1
                self.channel_mgr.current_channel = 1
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(0)  # 0-indexed
                self.update_led_color()
                print(f"\n🎛️  SWITCHED TO CHANNEL 1 (White) ⚪")
                self.channel_mgr.send_frozen_values_on_channel_switch(midiout, self, CC_MAP)

                loop_state = self.channel_mgr.get_current_loop_state()
                if loop_state.playing:
                    print(f"  🔄 Loop playing ({loop_state.loop_duration:.1f}s)")
                elif loop_state.midi_buffer:
                    print(f"  ⏸️  Loop ready ({loop_state.loop_duration:.1f}s)")
                print()

        # Only START pressed = Channel 2
        elif self.start_pressed and not self.select_pressed:
            if self.current_channel != 2:
                self.current_channel = 2
                self.channel_mgr.current_channel = 2
                if hasattr(self, 'sequencer_manager'):
                    self.sequencer_manager.set_current_channel(1)  # 0-indexed
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
        # message_type: 0x90 = Note On, 0x80 = Note Off, 0xB0 = CC
        # MIDI channels are 0-15, so subtract 1 from our 1-3 channel numbering
        return message_type + (self.current_channel - 1)

    def check_motion_toggle(self):
        """Check if L1+R1 pressed together to toggle motion"""
        if self.l1_pressed and self.r1_pressed:
            self.motion_enabled = not self.motion_enabled
            status = "ENABLED ✅" if self.motion_enabled else "DISABLED ❌"

            # Calibrate baseline when enabling motion mode
            if self.motion_enabled:
                self.tilt_baseline_x = self.smoothed_motion['tilt_x']
                self.tilt_baseline_y = self.smoothed_motion['tilt_y']
                print(f"\n🎯 CALIBRATED: Baseline X={self.tilt_baseline_x:.1f}, Y={self.tilt_baseline_y:.1f}")

            print(f"\n🎛️  MOTION CONTROL {status} (L1+R1)\n")

        # Update LED color (handles both motion and channel states)
        self.update_led_color()

        # Turn off haptics when disabling motion mode
        if not self.motion_enabled:
            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)

        return True

    def update_haptics_from_tilt(self, tilt_x_cc, tilt_y_cc):
        """Directional rumble: left/right for X-axis, both motors for Y-axis"""
        if not self.motion_enabled:
            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)
            return

        EXTREME_ZONE_START = 55

        rumble_left = 0
        rumble_right = 0

        # X-axis (left/right tilt) - directional
        if tilt_x_cc < 64:  # Tilting LEFT
            distance = 64 - tilt_x_cc
            if distance > EXTREME_ZONE_START:
                rumble_left = int(min(255, (distance - EXTREME_ZONE_START) * 20))
        else:  # Tilting RIGHT
            distance = tilt_x_cc - 64
            if distance > EXTREME_ZONE_START:
                rumble_right = int(min(255, (distance - EXTREME_ZONE_START) * 20))

        # Y-axis (forward/back tilt) - affects BOTH motors
        y_distance = abs(tilt_y_cc - 64)
        if y_distance > EXTREME_ZONE_START:
            y_rumble = int(min(255, (y_distance - EXTREME_ZONE_START) * 20))
            # Add Y rumble to both motors
            rumble_left = min(255, rumble_left + y_rumble)
            rumble_right = min(255, rumble_right + y_rumble)

        # Send to motors
        self.ds.setLeftMotor(rumble_left)
        self.ds.setRightMotor(rumble_right)

    def update_touchpad(self, x, y, is_active):
        """
        Update touchpad position and calculate loop window
        Called from main event loop when touchpad events arrive
        """
        self.touchpad_x = x
        self.touchpad_y = y
        self.touchpad_active = is_active

        if not is_active:
            # When finger lifts, reset to full loop playback
            self.window_position = 0.0
            self.window_size = 1.0
            return

        # Get current loop to calculate window
        loop_state = self.channel_mgr.get_current_loop_state()
        if not loop_state.playing or loop_state.loop_duration == 0:
            return

        # X-axis: Window starting position (0-100% of loop)
        self.window_position = x / 1920.0

        # Y-axis: Window size (minimum window to full loop)
        # Inverted: Higher Y value = larger window
        min_window_size = self.min_window_ms / (loop_state.loop_duration * 1000.0)
        min_window_size = max(0.001, min(min_window_size, 1.0))  # Clamp to 0.1% - 100%

        normalized_y = y / 1080.0
        self.window_size = min_window_size + (normalized_y * (1.0 - min_window_size))

        # Optional: Print for debugging
        if int(time.time() * 4) % 2 == 0:  # Print occasionally
            print(f"🎚️  Touchpad: Pos={self.window_position*100:.1f}% Size={self.window_size*100:.1f}%")

    def cleanup(self):
        """Properly shut down controller and haptics"""
        try:
            # Stop all loop playback
            for loop_state in self.channel_mgr.loop_states:
                loop_state.stop_playback()

            # Stop LED pulsing
            self.stop_led_pulse()

            # Turn off haptic motors
            self.ds.setLeftMotor(0)
            self.ds.setRightMotor(0)

            # Reset LED to off/dim
            self.ds.light.setColorI(0, 0, 0)

            # Close pydualsense connection
            self.ds.close()

            print("🎮 Controller cleaned up")
        except Exception as e:
            print(f"⚠️  Cleanup error (non-critical): {e}")
