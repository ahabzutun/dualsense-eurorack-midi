#!/usr/bin/env python3
import evdev
from evdev import ecodes
import rtmidi
import time
import select
from pydualsense import pydualsense
import threading


# ===== FREEZE STATE MANAGEMENT =====
class FreezeState:
    """Manages freeze state for a single channel/instrument"""
    def __init__(self):
        # Left stick press (L3) freezes these:
        self.l2_frozen = False
        self.l2_value = 0

        self.r2_frozen = False
        self.r2_value = 0

        self.left_stick_frozen = False
        self.left_x_value = 0
        self.left_y_value = 0

        # Right stick press (R3) freezes these:
        self.right_stick_frozen = False
        self.right_x_value = 0
        self.right_y_value = 0


# ===== MIDI LOOP RECORDING =====
class LoopState:
    """Manages MIDI loop recording and playback for a single channel"""
    def __init__(self):
        self.recording = False
        self.playing = False
        self.midi_buffer = []  # List of (timestamp, midi_message) tuples
        self.record_start_time = 0
        self.loop_duration = 0
        self.playback_thread = None
        self.playback_stop_event = threading.Event()
        self.buffer_lock = threading.Lock()  # Thread-safe buffer access
        self.MAX_LOOP_DURATION = 60.0  # 60 seconds max

    def start_recording(self):
        """Start recording MIDI messages"""
        with self.buffer_lock:
            # If playing, stop playback first
            if self.playing:
                self.stop_playback()

            # Clear buffer and start fresh
            self.midi_buffer = []
            self.recording = True
            self.record_start_time = time.time()
            self.loop_duration = 0

    def stop_recording(self):
        """Stop recording and calculate loop duration"""
        with self.buffer_lock:
            if self.recording:
                self.recording = False
                self.loop_duration = time.time() - self.record_start_time
                # Only keep the loop if it has content and is under max duration
                if not self.midi_buffer or self.loop_duration > self.MAX_LOOP_DURATION:
                    self.midi_buffer = []
                    self.loop_duration = 0
                    return False  # Recording failed/discarded
                return True  # Recording successful
            return False

    def record_message(self, midi_message):
        """Record a MIDI message with timestamp"""
        if not self.recording:
            return

        with self.buffer_lock:
            current_time = time.time()
            relative_time = current_time - self.record_start_time

            # Check if we've exceeded max duration
            if relative_time > self.MAX_LOOP_DURATION:
                print(f"\n⚠️  Max loop duration ({self.MAX_LOOP_DURATION}s) reached - stopping recording")
                self.stop_recording()
                return

            self.midi_buffer.append((relative_time, midi_message))

    def start_playback(self, midiout):
        """Start loop playback in background thread"""
        if not self.midi_buffer or self.playing:
            return False

        self.playing = True
        self.playback_stop_event.clear()
        self.playback_thread = threading.Thread(
            target=self._playback_loop,
            args=(midiout,),
            daemon=True
        )
        self.playback_thread.start()
        return True

    def stop_playback(self):
        """Stop loop playback"""
        if self.playing:
            self.playing = False
            self.playback_stop_event.set()
            if self.playback_thread:
                self.playback_thread.join(timeout=0.5)

    def _playback_loop(self, midiout):
        """Background thread that plays back the loop continuously"""
        while self.playing and not self.playback_stop_event.is_set():
            loop_start = time.time()

            with self.buffer_lock:
                buffer_copy = list(self.midi_buffer)  # Copy for thread safety

            for timestamp, midi_msg in buffer_copy:
                if not self.playing or self.playback_stop_event.is_set():
                    break

                # Wait until it's time to play this message
                target_time = loop_start + timestamp
                sleep_time = target_time - time.time()

                if sleep_time > 0:
                    # Use event.wait for interruptible sleep
                    if self.playback_stop_event.wait(timeout=sleep_time):
                        break  # Stop event was set

                # Send the MIDI message
                try:
                    midiout.send_message(midi_msg)
                except:
                    pass  # Ignore errors during playback

            # Wait for loop to complete before repeating
            if self.playing and not self.playback_stop_event.is_set():
                remaining_time = self.loop_duration - (time.time() - loop_start)
                if remaining_time > 0:
                    self.playback_stop_event.wait(timeout=remaining_time)

    def clear_loop(self):
        """Clear the loop buffer (safe version that prevents deadlock)"""
        # First, signal the playback thread to stop
        if self.playing:
            self.playing = False
            self.playback_stop_event.set()

        # Try to acquire lock with timeout to prevent permanent hang
        lock_acquired = self.buffer_lock.acquire(timeout=1.0)

        if lock_acquired:
            try:
                # Clear all loop data
                self.midi_buffer = []
                self.loop_duration = 0
                self.recording = False

                # Wait for playback thread to finish (non-blocking)
                if self.playback_thread and self.playback_thread.is_alive():
                    self.playback_thread.join(timeout=0.2)

            finally:
                self.buffer_lock.release()
            return True
        else:
            print("⚠️  Could not clear loop (timeout) - try again")
            return False


class ChannelManager:
    """Manages the 3 channels and their freeze states"""
    def __init__(self):
        self.current_channel = 1  # 1, 2, 3 to match the main code
        self.freeze_states = [FreezeState() for _ in range(3)]
        self.loop_states = [LoopState() for _ in range(3)]  # NEW: Loop states

        # Track previous button states for edge detection
        self.prev_l3 = False
        self.prev_r3 = False

        # Track current raw values (updated continuously)
        self.current_l2 = 0
        self.current_r2 = 0
        self.current_left_x = 0
        self.current_left_y = 0
        self.current_right_x = 0
        self.current_right_y = 0

    def get_current_freeze_state(self):
        """Get the freeze state for the current channel (1-indexed to 0-indexed)"""
        return self.freeze_states[self.current_channel - 1]

    def get_current_loop_state(self):
        """Get the loop state for the current channel"""
        return self.loop_states[self.current_channel - 1]

    def toggle_left_group_freeze(self):
        """Toggle freeze for L2, R2, and left stick"""
        state = self.get_current_freeze_state()

        if state.left_stick_frozen:
            # Unfreeze everything
            state.l2_frozen = False
            state.r2_frozen = False
            state.left_stick_frozen = False
            return "UNFROZEN"
        else:
            # Freeze current values
            state.l2_frozen = True
            state.l2_value = self.current_l2

            state.r2_frozen = True
            state.r2_value = self.current_r2

            state.left_stick_frozen = True
            state.left_x_value = self.current_left_x
            state.left_y_value = self.current_left_y
            return "FROZEN"

    def toggle_right_stick_freeze(self):
        """Toggle freeze for right stick"""
        state = self.get_current_freeze_state()

        if state.right_stick_frozen:
            # Unfreeze
            state.right_stick_frozen = False
            return "UNFROZEN"
        else:
            # Freeze current values
            state.right_stick_frozen = True
            state.right_x_value = self.current_right_x
            state.right_y_value = self.current_right_y
            return "FROZEN"

    def get_l2_value(self, current_value):
        """Get L2 value (frozen or live)"""
        self.current_l2 = current_value  # Always update current value
        state = self.get_current_freeze_state()
        return state.l2_value if state.l2_frozen else current_value

    def get_r2_value(self, current_value):
        """Get R2 value (frozen or live)"""
        self.current_r2 = current_value
        state = self.get_current_freeze_state()
        return state.r2_value if state.r2_frozen else current_value

    def get_left_stick_values(self, current_x, current_y):
        """Get left stick values (frozen or live)"""
        self.current_left_x = current_x
        self.current_left_y = current_y
        state = self.get_current_freeze_state()
        if state.left_stick_frozen:
            return state.left_x_value, state.left_y_value
        return current_x, current_y

    def get_right_stick_values(self, current_x, current_y):
        """Get right stick values (frozen or live)"""
        self.current_right_x = current_x
        self.current_right_y = current_y
        state = self.get_current_freeze_state()
        if state.right_stick_frozen:
            return state.right_x_value, state.right_y_value
        return current_x, current_y

    def send_frozen_values_on_channel_switch(self, midiout, controller_obj, CC_MAP):
        """Send frozen values immediately when switching channels"""
        state = self.get_current_freeze_state()

        if state.l2_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['l2_trigger'], state.l2_value])
            print(f"  ❄️  L2 frozen → {state.l2_value}")

        if state.r2_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['r2_trigger'], state.r2_value])
            print(f"  ❄️  R2 frozen → {state.r2_value}")

        if state.left_stick_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_x'], state.left_x_value])
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_y'], state.left_y_value])
            print(f"  ❄️  Left stick frozen → X:{state.left_x_value}, Y:{state.left_y_value}")

        if state.right_stick_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_x'], state.right_x_value])
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_y'], state.right_y_value])
            print(f"  ❄️  Right stick frozen → X:{state.right_x_value}, Y:{state.right_y_value}")


# MIDI CC Mapping Configuration
CC_MAP = {
    'left_stick_x': 1,   # Mod Wheel
    'left_stick_y': 2,   # Breath
    'right_stick_x': 74, # Filter Cutoff
    'right_stick_y': 71, # Resonance
    'l2_trigger': 7,     # Volume
    'r2_trigger': 10,    # Pan
    'tilt_x': 16,        # Motion Tilt X
    'tilt_y': 17,        # Motion Tilt Y
    'twist': 18,         # Motion Twist
    'touchpad_x': 20,    # Touchpad X
    'touchpad_y': 21,    # Touchpad Y
    'btn_south': 14,     # X button (✕) trigger (General Purpose)
    'btn_east': 15,      # O button (○) trigger (General Purpose)
}

# Note mapping for buttons
NOTE_MAP = {
    # 304: BTN_SOUTH (✕) - Now CC trigger (CC 80)
    # 305: BTN_EAST (○) - Now CC trigger (CC 81)
    # 307: BTN_NORTH (△) - Now used for LOOP RECORDING
    308: 67,  # BTN_WEST (□) → G4
    272: 72,  # BTN_LEFT (Touchpad Click) → C5
    # 317: BTN_THUMBL (L3 - Left Stick Click) - Now used for FREEZE
    # 318: BTN_THUMBR (R3 - Right Stick Click) - Now used for FREEZE
    # 314: BTN_SELECT (Create/Share) - Used for channel switching
    # 315: BTN_START (Options) - Used for channel switching
}

# D-pad notes (handled separately as HAT axes)
DPAD_NOTES = {
    'up': 65,     # F4
    'down': 63,   # D#4
    'left': 61,   # C#4
    'right': 66,  # F#4
}

# Configuration
STICK_DEADZONE = 10      # Ignore changes smaller than this (out of 127)
MOTION_THRESHOLD = 8     # Minimum change to send motion CC
STICK_CENTER = 127       # Stick center value (0-255 range, center ~127)
MOTION_SMOOTHING = 0.3   # Smoothing factor for motion (0-1, lower = smoother)

# Motion deadzones (only send when significantly away from center)
TILT_DEADZONE = 25       # Tilt must be 25+ away from center (64) to send
GYRO_DEADZONE = 30       # Gyro must be 30+ away from center (64) to send

# Loop recording
LONG_PRESS_DURATION = 1.0  # 1 second for long press


class MIDIController:
    def __init__(self, channel_manager):
        self.channel_mgr = channel_manager  # Reference to the channel manager
        self.last_cc_values = {}  # Track last sent CC values
        self.last_motion_raw = {'tilt_x': 0, 'tilt_y': 0, 'twist': 0}
        self.smoothed_motion = {'tilt_x': 64, 'tilt_y': 64, 'twist': 64}
        self.active_notes = {}
        self.touchpad_active = False  # Track if finger is on touchpad

        # Motion control toggle and loop recording
        self.motion_enabled = False
        self.l1_pressed = False
        self.r1_pressed = False
        self.l1_press_time = 0  # NEW: Track L1 press timing
        self.r1_press_time = 0  # NEW: Track R1 press timing

        # NEW: Movement-based haptics
        self.previous_tilt_x = 64
        self.previous_tilt_y = 64
        self.last_haptic_pulse = 0
        self.haptic_pulse_duration = 0.1
        self.haptic_active = False

        # NEW: Channel switching with START/SELECT
        self.current_channel = 1  # Default to channel 1
        self.select_pressed = False  # BTN_SELECT (Create/Share)
        self.start_pressed = False   # BTN_START (Options)

        # NEW: Button hold tracking for repeated CC messages (MIDI learn support)
        self.btn_south_held = False  # X button
        self.btn_east_held = False   # O button
        self.last_btn_send_time = {'south': 0, 'east': 0}
        self.btn_repeat_interval = 0.05  # Send CC every 50ms while held

        # NEW: Loop recording - Triangle button timing
        self.triangle_pressed = False
        self.triangle_press_time = 0
        self.square_pressed = False  # For clear combo

        # NEW: LED pulsing for loop feedback
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
        import math
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

            # NEW: Calibrate baseline when enabling motion mode
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


def main():
    # Initialize channel manager first
    channel_manager = ChannelManager()

    # Pass channel manager to MIDIController
    controller_obj = MIDIController(channel_manager)

    # ===== STEP 1: Try to find DualSense devices =====
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

    controller = None
    motion = None
    touchpad = None

    for device in devices:
        if "DualSense" in device.name:
            if "Motion" in device.name:
                motion = device
            elif "Touchpad" in device.name:
                touchpad = device
            elif "Touchpad" not in device.name and "Motion" not in device.name:
                controller = device

    # ===== STEP 2: Check if we found everything =====
    controller_available = (controller is not None and motion is not None and touchpad is not None)

    if controller_available:
        print(f"✅ Controller: {controller.name}")
        print(f"✅ Motion: {motion.name}")
        print(f"✅ Touchpad: {touchpad.name}")
    else:
        print("⚠️  DualSense controller not found!")
        print(f"   Controller: {controller is not None}, Motion: {motion is not None}, Touchpad: {touchpad is not None}")
        print("   Running with virtual MIDI port only...")

    # ===== STEP 3: Set up MIDI - ALWAYS create output =====
    midiout = rtmidi.MidiOut()

    # Always use virtual port (consistent behavior)
    midiout.open_virtual_port("DualSense_Controller")
    port_name = "DualSense_Controller (Virtual)"

    print(f"✅ MIDI Output: {port_name}\n")

    if controller_available:
        print("🎮 DualSense → 🎹 MIDI → 🎛️ Passthrough Service")
        print("=" * 50)
        print("Controls:")
        print("  Button (□) → Note")
        print("  Buttons (✕○) → CC Triggers (14, 15) - with MIDI learn repeat")
        print("  D-Pad (↑↓←→) → Notes")
        print("  Touchpad Click → Note")
        print("  L3 (Left Stick Click) → FREEZE L2/R2/Left Stick ❄️")
        print("  R3 (Right Stick Click) → FREEZE Right Stick ❄️")
        print("  Sticks → CC 1,2,74,71")
        print("  Triggers → CC 7,10")
        print("  Touchpad → CC 20,21")
        print("  Motion → CC 16,17,18 (Press L1+R1 to toggle)")
        print("=" * 50)
        print("🎛️  MIDI Channel: 1 (White LED) ⚪")
        print("   SELECT (Create) → Channel 1 (White)")
        print("   START (Options) → Channel 2 (Green)")
        print("   BOTH together → Channel 3 (Yellow)")
        print("=" * 50)
        print("❄️  FREEZE FEATURE:")
        print("   L3 → Freeze/Unfreeze L2, R2, and Left Stick")
        print("   R3 → Freeze/Unfreeze Right Stick")
        print("   Freeze states are saved per channel!")
        print("=" * 50)
        print("🔴 LOOP RECORDING (per channel):")
        print("   △ Long Press (>1s) → Start/Stop Recording")
        print("   △ Short Press → Play/Stop Loop")
        print("   △ + □ → Clear Loop")
        print("   Recording: Red pulse | Playing: Green pulse")
        print("=" * 50)
        print("🎛️  Motion Control: DISABLED (Press L1+R1 to enable)")
        print("=" * 50)
        print("Press Ctrl+C to exit\n")
    else:
        print("⏸️  Waiting for DualSense controller to be connected...")
        print("   Virtual MIDI port active for passthrough service")
        print("   Press Ctrl+C to exit\n")

    # ===== STEP 4: Main loop - only process if controller available =====
    if controller_available:
        # Use select() to monitor all three devices
        devices_dict = {
            controller.fd: controller,
            motion.fd: motion,
            touchpad.fd: touchpad
        }

        try:
            while True:
                r, w, x = select.select(devices_dict.keys(), [], [], 0.01)

                for fd in r:
                    device = devices_dict[fd]

                    for event in device.read():
                        # Handle controller events
                        if device == controller:
                            # Track L1 and R1 for motion toggle
                            if event.type == ecodes.EV_KEY:
                                if event.code == 310:  # BTN_TL (L1)
                                    if event.value == 1:  # Button pressed
                                        controller_obj.l1_pressed = True
                                        controller_obj.l1_press_time = time.time()
                                    else:  # Button released
                                        # Check if this was a combo press with R1
                                        if controller_obj.r1_pressed:
                                            # Both were held = Motion toggle
                                            controller_obj.check_motion_toggle()
                                        else:
                                            # Solo L1 press = Clear loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.clear_loop():
                                                controller_obj.update_led_color()
                                                print(f"\n🗑️  Channel {channel_manager.current_channel}: Loop CLEARED (L1)")
                                            else:
                                                print(f"\n⚠️  Channel {channel_manager.current_channel}: Could not clear loop")

                                        controller_obj.l1_pressed = False

                                elif event.code == 311:  # BTN_TR (R1)
                                    if event.value == 1:  # Button pressed
                                        controller_obj.r1_pressed = True
                                        controller_obj.r1_press_time = time.time()
                                    else:  # Button released
                                        # Check if this was a combo press with L1
                                        if controller_obj.l1_pressed:
                                            # Both were held = Motion toggle
                                            controller_obj.check_motion_toggle()
                                        else:
                                            # Solo R1 press = Loop recording controls
                                            loop_state = channel_manager.get_current_loop_state()
                                            press_duration = time.time() - controller_obj.r1_press_time

                                            if press_duration >= LONG_PRESS_DURATION:
                                                # LONG PRESS: Toggle recording
                                                if loop_state.recording:
                                                    if loop_state.stop_recording():
                                                        print(f"\n⏹️  Channel {channel_manager.current_channel}: Recording STOPPED ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events)")
                                                    else:
                                                        print(f"\n⚠️  Channel {channel_manager.current_channel}: Recording cancelled (empty or too long)")
                                                    controller_obj.update_led_color()
                                                else:
                                                    loop_state.start_recording()
                                                    controller_obj.update_led_color()
                                                    print(f"\n🔴 Channel {channel_manager.current_channel}: Recording STARTED (R1)")
                                            else:
                                                # SHORT PRESS: Toggle playback
                                                if loop_state.playing:
                                                    loop_state.stop_playback()
                                                    controller_obj.update_led_color()
                                                    print(f"\n⏸️  Channel {channel_manager.current_channel}: Playback STOPPED (R1)")
                                                elif loop_state.midi_buffer:
                                                    if loop_state.start_playback(midiout):
                                                        controller_obj.update_led_color()
                                                        print(f"\n▶️  Channel {channel_manager.current_channel}: Playing loop ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events) (R1)")
                                                else:
                                                    print(f"\n⚠️  Channel {channel_manager.current_channel}: No loop to play")

                                        controller_obj.r1_pressed = False

                                # === FREEZE FEATURE: L3 (Left Stick Click) ===
                                elif event.code == 317:  # BTN_THUMBL (L3)
                                    l3_pressed = (event.value == 1)

                                    # Edge detection: only trigger on press (not release)
                                    if l3_pressed and not channel_manager.prev_l3:
                                        status = channel_manager.toggle_left_group_freeze()
                                        print(f"\n❄️  Channel {channel_manager.current_channel}: L2/R2/Left Stick {status}")

                                        # Brief haptic feedback
                                        if status == "FROZEN":
                                            controller_obj.ds.setLeftMotor(100)
                                            controller_obj.ds.setRightMotor(100)
                                            time.sleep(0.05)
                                            controller_obj.ds.setLeftMotor(0)
                                            controller_obj.ds.setRightMotor(0)

                                    channel_manager.prev_l3 = l3_pressed

                                # === FREEZE FEATURE: R3 (Right Stick Click) ===
                                elif event.code == 318:  # BTN_THUMBR (R3)
                                    r3_pressed = (event.value == 1)

                                    if r3_pressed and not channel_manager.prev_r3:
                                        status = channel_manager.toggle_right_stick_freeze()
                                        print(f"\n❄️  Channel {channel_manager.current_channel}: Right Stick {status}")

                                        # Brief haptic feedback
                                        if status == "FROZEN":
                                            controller_obj.ds.setRightMotor(100)
                                            time.sleep(0.05)
                                            controller_obj.ds.setRightMotor(0)

                                    channel_manager.prev_r3 = r3_pressed

                                # === LOOP RECORDING: Triangle Button ===
                                elif event.code == 307:  # BTN_NORTH (△)
                                    loop_state = channel_manager.get_current_loop_state()

                                    if event.value == 1:  # Button pressed
                                        controller_obj.triangle_pressed = True
                                        controller_obj.triangle_press_time = time.time()

                                        # Check for clear combo: Triangle + Square
                                        if controller_obj.square_pressed:
                                            loop_state.clear_loop()
                                            controller_obj.update_led_color()
                                            print(f"\n🗑️  Channel {channel_manager.current_channel}: Loop CLEARED")

                                    elif event.value == 0:  # Button released
                                        if controller_obj.triangle_pressed:
                                            press_duration = time.time() - controller_obj.triangle_press_time

                                            if press_duration >= LONG_PRESS_DURATION:
                                                # LONG PRESS: Toggle recording
                                                if loop_state.recording:
                                                    if loop_state.stop_recording():
                                                        print(f"\n⏹️  Channel {channel_manager.current_channel}: Recording STOPPED ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events)")
                                                    else:
                                                        print(f"\n⚠️  Channel {channel_manager.current_channel}: Recording cancelled (empty or too long)")
                                                    controller_obj.update_led_color()
                                                else:
                                                    loop_state.start_recording()
                                                    controller_obj.update_led_color()
                                                    print(f"\n🔴 Channel {channel_manager.current_channel}: Recording STARTED")
                                            else:
                                                # SHORT PRESS: Toggle playback
                                                if loop_state.playing:
                                                    loop_state.stop_playback()
                                                    controller_obj.update_led_color()
                                                    print(f"\n⏸️  Channel {channel_manager.current_channel}: Loop STOPPED")
                                                elif loop_state.midi_buffer:
                                                    if loop_state.start_playback(midiout):
                                                        controller_obj.update_led_color()
                                                        print(f"\n▶️  Channel {channel_manager.current_channel}: Loop PLAYING ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events)")
                                                else:
                                                    print(f"\n⚠️  Channel {channel_manager.current_channel}: No loop to play")

                                        controller_obj.triangle_pressed = False

                                # Track Square button for clear combo
                                elif event.code == 308:  # BTN_WEST (□)
                                    if event.value == 1:  # Pressed
                                        controller_obj.square_pressed = True

                                        # Check for clear combo: Triangle + Square
                                        if controller_obj.triangle_pressed:
                                            loop_state = channel_manager.get_current_loop_state()
                                            loop_state.clear_loop()
                                            controller_obj.update_led_color()
                                            print(f"\n🗑️  Channel {channel_manager.current_channel}: Loop CLEARED")
                                        else:
                                            # Square as normal note
                                            if event.code in NOTE_MAP:
                                                note = NOTE_MAP[event.code]
                                                note_on = [controller_obj.get_midi_channel_byte(0x90), note, 100]
                                                midiout.send_message(note_on)

                                                # Record to loop
                                                loop_state = channel_manager.get_current_loop_state()
                                                if loop_state.recording:
                                                    loop_state.record_message(note_on)

                                                controller_obj.active_notes[event.code] = note
                                                print(f"🎵 Note ON:  {note} (Ch {controller_obj.current_channel})")
                                    else:  # Released
                                        controller_obj.square_pressed = False

                                        # Square note off
                                        if event.code in NOTE_MAP and event.code in controller_obj.active_notes:
                                            note = NOTE_MAP[event.code]
                                            note_off = [controller_obj.get_midi_channel_byte(0x80), note, 0]
                                            midiout.send_message(note_off)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_off)

                                            del controller_obj.active_notes[event.code]
                                            print(f"🎵 Note OFF: {note} (Ch {controller_obj.current_channel})")

                                # NEW: SELECT button for channel switching (no notes)
                                elif event.code == 314:  # BTN_SELECT (Create/Share)
                                    if event.value == 1:  # Pressed
                                        controller_obj.select_pressed = True
                                        controller_obj.check_channel_switch(midiout)
                                    else:  # Released
                                        controller_obj.select_pressed = False

                                # NEW: START button for channel switching (no notes)
                                elif event.code == 315:  # BTN_START (Options)
                                    if event.value == 1:  # Pressed
                                        controller_obj.start_pressed = True
                                        controller_obj.check_channel_switch(midiout)
                                    else:  # Released
                                        controller_obj.start_pressed = False

                                # CC Triggers: X and O buttons
                                elif event.code == 304:  # BTN_SOUTH (✕)
                                    if event.value == 1:  # Pressed
                                        controller_obj.btn_south_held = True
                                        controller_obj.last_btn_send_time['south'] = time.time()
                                        # Send initial CC 127
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_south'], 127]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  X (✕)  → CC{CC_MAP['btn_south']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:  # Released
                                        controller_obj.btn_south_held = False
                                        # Send final CC 0
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_south'], 0]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  X (✕)  → CC{CC_MAP['btn_south']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                                elif event.code == 305:  # BTN_EAST (○)
                                    if event.value == 1:  # Pressed
                                        controller_obj.btn_east_held = True
                                        controller_obj.last_btn_send_time['east'] = time.time()
                                        # Send initial CC 127
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_east'], 127]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  O (○)  → CC{CC_MAP['btn_east']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:  # Released
                                        controller_obj.btn_east_held = False
                                        # Send final CC 0
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_east'], 0]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  O (○)  → CC{CC_MAP['btn_east']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                            # Analog inputs → CC (WITH FREEZE SUPPORT + LOOP RECORDING)
                            elif event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:  # Left Stick X
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    # Get frozen or live value
                                    cc_val, _ = channel_manager.get_left_stick_values(raw_val, channel_manager.current_left_y)
                                    if controller_obj.should_send_cc(CC_MAP['left_stick_x'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_x'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🕹️  Left X  → CC{CC_MAP['left_stick_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:  # Left Stick Y
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    _, cc_val = channel_manager.get_left_stick_values(channel_manager.current_left_x, raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['left_stick_y'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_y'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🕹️  Left Y  → CC{CC_MAP['left_stick_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RX:  # Right Stick X
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    cc_val, _ = channel_manager.get_right_stick_values(raw_val, channel_manager.current_right_y)
                                    if controller_obj.should_send_cc(CC_MAP['right_stick_x'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_x'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🕹️  Right X → CC{CC_MAP['right_stick_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RY:  # Right Stick Y
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    _, cc_val = channel_manager.get_right_stick_values(channel_manager.current_right_x, raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['right_stick_y'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_y'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🕹️  Right Y → CC{CC_MAP['right_stick_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Z:  # L2 Trigger
                                    raw_val = controller_obj.scale_value(event.value, 0, 255)
                                    cc_val = channel_manager.get_l2_value(raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['l2_trigger'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['l2_trigger'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  L2     → CC{CC_MAP['l2_trigger']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RZ:  # R2 Trigger
                                    raw_val = controller_obj.scale_value(event.value, 0, 255)
                                    cc_val = channel_manager.get_r2_value(raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['r2_trigger'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['r2_trigger'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  R2     → CC{CC_MAP['r2_trigger']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                # D-pad handling (HAT axes, not buttons!)
                                elif event.code == ecodes.ABS_HAT0X:  # D-pad Left/Right
                                    if event.value == -1:  # Left
                                        note = DPAD_NOTES['left']
                                        note_on = [controller_obj.get_midi_channel_byte(0x90), note, 100]
                                        midiout.send_message(note_on)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(note_on)

                                        controller_obj.active_notes['dpad_left'] = note
                                        print(f"🎵 D-pad LEFT → Note ON: {note} (Ch {controller_obj.current_channel})")
                                    elif event.value == 1:  # Right
                                        note = DPAD_NOTES['right']
                                        note_on = [controller_obj.get_midi_channel_byte(0x90), note, 100]
                                        midiout.send_message(note_on)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(note_on)

                                        controller_obj.active_notes['dpad_right'] = note
                                        print(f"🎵 D-pad RIGHT → Note ON: {note} (Ch {controller_obj.current_channel})")
                                    elif event.value == 0:  # Released
                                        if 'dpad_left' in controller_obj.active_notes:
                                            note_off = [controller_obj.get_midi_channel_byte(0x80), controller_obj.active_notes['dpad_left'], 0]
                                            midiout.send_message(note_off)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_off)

                                            print(f"🎵 D-pad LEFT → Note OFF: {controller_obj.active_notes['dpad_left']} (Ch {controller_obj.current_channel})")
                                            del controller_obj.active_notes['dpad_left']
                                        if 'dpad_right' in controller_obj.active_notes:
                                            note_off = [controller_obj.get_midi_channel_byte(0x80), controller_obj.active_notes['dpad_right'], 0]
                                            midiout.send_message(note_off)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_off)

                                            print(f"🎵 D-pad RIGHT → Note OFF: {controller_obj.active_notes['dpad_right']} (Ch {controller_obj.current_channel})")
                                            del controller_obj.active_notes['dpad_right']

                                elif event.code == ecodes.ABS_HAT0Y:  # D-pad Up/Down
                                    if event.value == -1:  # Up
                                        note = DPAD_NOTES['up']
                                        note_on = [controller_obj.get_midi_channel_byte(0x90), note, 100]
                                        midiout.send_message(note_on)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(note_on)

                                        controller_obj.active_notes['dpad_up'] = note
                                        print(f"🎵 D-pad UP → Note ON: {note} (Ch {controller_obj.current_channel})")
                                    elif event.value == 1:  # Down
                                        note = DPAD_NOTES['down']
                                        note_on = [controller_obj.get_midi_channel_byte(0x90), note, 100]
                                        midiout.send_message(note_on)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(note_on)

                                        controller_obj.active_notes['dpad_down'] = note
                                        print(f"🎵 D-pad DOWN → Note ON: {note} (Ch {controller_obj.current_channel})")
                                    elif event.value == 0:  # Released
                                        if 'dpad_up' in controller_obj.active_notes:
                                            note_off = [controller_obj.get_midi_channel_byte(0x80), controller_obj.active_notes['dpad_up'], 0]
                                            midiout.send_message(note_off)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_off)

                                            print(f"🎵 D-pad UP → Note OFF: {controller_obj.active_notes['dpad_up']} (Ch {controller_obj.current_channel})")
                                            del controller_obj.active_notes['dpad_up']
                                        if 'dpad_down' in controller_obj.active_notes:
                                            note_off = [controller_obj.get_midi_channel_byte(0x80), controller_obj.active_notes['dpad_down'], 0]
                                            midiout.send_message(note_off)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_off)

                                            print(f"🎵 D-pad DOWN → Note OFF: {controller_obj.active_notes['dpad_down']} (Ch {controller_obj.current_channel})")
                                            del controller_obj.active_notes['dpad_down']

                        # Handle motion sensor events
                        elif device == motion:
                            if event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:  # Tilt X (left/right)
                                    raw = controller_obj.scale_value(event.value, -500, 500)
                                    cc_val = controller_obj.smooth_motion(raw, 'tilt_x', MOTION_SMOOTHING)
                                    controller_obj.update_haptics_from_tilt(cc_val, controller_obj.smoothed_motion['tilt_y'])
                                    if controller_obj.motion_enabled and abs(cc_val - 64) > TILT_DEADZONE:
                                        if controller_obj.should_send_cc(CC_MAP['tilt_x'], cc_val):
                                            midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['tilt_x'], cc_val]
                                            midiout.send_message(midi_msg)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(midi_msg)

                                            print(f"📐 Tilt X  → CC{CC_MAP['tilt_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:  # Tilt Y (forward/back)
                                    raw = controller_obj.scale_value(event.value, 7500, 8500)
                                    cc_val = controller_obj.smooth_motion(raw, 'tilt_y', MOTION_SMOOTHING)
                                    controller_obj.update_haptics_from_tilt(controller_obj.smoothed_motion['tilt_x'], cc_val)
                                    if controller_obj.motion_enabled and abs(cc_val - 64) > TILT_DEADZONE:
                                        if controller_obj.should_send_cc(CC_MAP['tilt_y'], cc_val):
                                            midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['tilt_y'], cc_val]
                                            midiout.send_message(midi_msg)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(midi_msg)

                                            print(f"📐 Tilt Y  → CC{CC_MAP['tilt_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RZ:  # Twist (yaw)
                                    raw = controller_obj.scale_value(event.value, -1000, 1000)
                                    cc_val = controller_obj.smooth_motion(raw, 'twist', MOTION_SMOOTHING)
                                    if controller_obj.motion_enabled and abs(cc_val - 64) > GYRO_DEADZONE:
                                        if controller_obj.should_send_cc(CC_MAP['twist'], cc_val):
                                            midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['twist'], cc_val]
                                            midiout.send_message(midi_msg)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(midi_msg)

                                            print(f"🔄 Twist   → CC{CC_MAP['twist']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                        # Handle touchpad events
                        elif device == touchpad:
                            if event.type == ecodes.EV_KEY:
                                if event.code == ecodes.BTN_TOUCH:
                                    controller_obj.touchpad_active = (event.value == 1)
                                    if event.value == 1:
                                        print("👆 Touchpad: Finger DOWN")
                                    else:
                                        print("👆 Touchpad: Finger UP")

                                elif event.code == ecodes.BTN_LEFT:
                                    if event.code in NOTE_MAP:
                                        note = NOTE_MAP[event.code]
                                        if event.value == 1:
                                            note_on = [controller_obj.get_midi_channel_byte(0x90), note, 100]
                                            midiout.send_message(note_on)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_on)

                                            controller_obj.active_notes[event.code] = note
                                            print(f"🎵 Touchpad Click → Note ON: {note} (Ch {controller_obj.current_channel})")
                                        elif event.value == 0:
                                            note_off = [controller_obj.get_midi_channel_byte(0x80), note, 0]
                                            midiout.send_message(note_off)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(note_off)

                                            if event.code in controller_obj.active_notes:
                                                del controller_obj.active_notes[event.code]
                                            print(f"🎵 Touchpad Click → Note OFF: {note} (Ch {controller_obj.current_channel})")

                            elif event.type == ecodes.EV_ABS:
                                if controller_obj.touchpad_active:
                                    if event.code == ecodes.ABS_X:  # Touchpad X
                                        cc_val = controller_obj.scale_value(event.value, 0, 1920)
                                        if controller_obj.should_send_cc(CC_MAP['touchpad_x'], cc_val):
                                            midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_x'], cc_val]
                                            midiout.send_message(midi_msg)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(midi_msg)

                                            print(f"👆 Touch X → CC{CC_MAP['touchpad_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                    elif event.code == ecodes.ABS_Y:  # Touchpad Y
                                        cc_val = controller_obj.scale_value(event.value, 0, 1080)
                                        if controller_obj.should_send_cc(CC_MAP['touchpad_y'], cc_val):
                                            midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_y'], cc_val]
                                            midiout.send_message(midi_msg)

                                            # Record to loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                loop_state.record_message(midi_msg)

                                            print(f"👆 Touch Y → CC{CC_MAP['touchpad_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                # After processing all events, send repeated CCs for held buttons
                controller_obj.send_held_button_ccs(midiout)

        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")

            # Send note offs for any active notes
            for note in controller_obj.active_notes.values():
                midiout.send_message([controller_obj.get_midi_channel_byte(0x80), note, 0])

            controller_obj.cleanup()

            if controller:
                controller.close()
            if motion:
                motion.close()
            if touchpad:
                touchpad.close()

            del midiout
            print("✅ Clean exit!")

    else:
        # No controller - just keep virtual port alive
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
            controller_obj.cleanup()
            del midiout
            print("✅ Clean exit!")

if __name__ == "__main__":
    main()
