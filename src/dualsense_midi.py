#!/usr/bin/env python3
import evdev
from evdev import ecodes
import rtmidi
import time
import select

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
}

# Note mapping for buttons
NOTE_MAP = {
    304: 60,  # BTN_SOUTH (✕) → C4
    305: 62,  # BTN_EAST (○) → D4
    307: 64,  # BTN_NORTH (△) → E4
    308: 67,  # BTN_WEST (□) → G4
    272: 72,  # BTN_LEFT (Touchpad Click) → C5
    317: 69,  # BTN_THUMBL (L3 - Left Stick Click) → A4
    318: 71,  # BTN_THUMBR (R3 - Right Stick Click) → B4
    314: 73,  # BTN_SELECT (Create/Share) → C#5
    315: 74,  # BTN_START (Options) → D5
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

class MIDIController:
    def __init__(self):
        self.last_cc_values = {}  # Track last sent CC values
        self.last_motion_raw = {'tilt_x': 0, 'tilt_y': 0, 'twist': 0}
        self.smoothed_motion = {'tilt_x': 0, 'tilt_y': 0, 'twist': 0}
        self.active_notes = {}
        self.touchpad_active = False  # Track if finger is on touchpad

        # Motion control toggle
        self.motion_enabled = False
        self.l1_pressed = False
        self.r1_pressed = False

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

    def check_motion_toggle(self):
        """Check if L1+R1 pressed together to toggle motion"""
        if self.l1_pressed and self.r1_pressed:
            self.motion_enabled = not self.motion_enabled
            status = "ENABLED ✅" if self.motion_enabled else "DISABLED ❌"
            print(f"\n🎛️  MOTION CONTROL {status} (L1+R1)\n")
            return True
        return False

def main():
    controller_obj = MIDIController()

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
        print("  Buttons (✕○□△) → Notes")
        print("  D-Pad (↑↓←→) → Notes")
        print("  L3/R3 (Stick Clicks) → Notes")
        print("  Create/Options → Notes")
        print("  Sticks → CC 1,2,74,71")
        print("  Triggers → CC 7,10")
        print("  Touchpad → CC 20,21 + Note (click)")
        print("  Motion → CC 16,17,18 (Press L1+R1 to toggle)")
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
                                    controller_obj.l1_pressed = (event.value == 1)
                                    print(f"🔘 L1: {'PRESSED' if event.value == 1 else 'RELEASED'}")
                                    if event.value == 1 and controller_obj.r1_pressed:
                                        controller_obj.check_motion_toggle()

                                elif event.code == 311:  # BTN_TR (R1)
                                    controller_obj.r1_pressed = (event.value == 1)
                                    print(f"🔘 R1: {'PRESSED' if event.value == 1 else 'RELEASED'}")
                                    if event.value == 1 and controller_obj.l1_pressed:
                                        controller_obj.check_motion_toggle()

                                # Button press/release → Note On/Off
                                elif event.code in NOTE_MAP:
                                    note = NOTE_MAP[event.code]
                                    if event.value == 1:  # Press
                                        note_on = [0x90, note, 100]
                                        midiout.send_message(note_on)
                                        controller_obj.active_notes[event.code] = note
                                        print(f"🎵 Note ON:  {note}")
                                    elif event.value == 0:  # Release
                                        note_off = [0x80, note, 0]
                                        midiout.send_message(note_off)
                                        if event.code in controller_obj.active_notes:
                                            del controller_obj.active_notes[event.code]
                                        print(f"🎵 Note OFF: {note}")

                            # Analog inputs → CC
                            elif event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:  # Left Stick X
                                    cc_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    if controller_obj.should_send_cc(CC_MAP['left_stick_x'], cc_val):
                                        midiout.send_message([0xB0, CC_MAP['left_stick_x'], cc_val])
                                        print(f"🕹️  Left X  → CC{CC_MAP['left_stick_x']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_Y:  # Left Stick Y
                                    cc_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    if controller_obj.should_send_cc(CC_MAP['left_stick_y'], cc_val):
                                        midiout.send_message([0xB0, CC_MAP['left_stick_y'], cc_val])
                                        print(f"🕹️  Left Y  → CC{CC_MAP['left_stick_y']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_RX:  # Right Stick X
                                    cc_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    if controller_obj.should_send_cc(CC_MAP['right_stick_x'], cc_val):
                                        midiout.send_message([0xB0, CC_MAP['right_stick_x'], cc_val])
                                        print(f"🕹️  Right X → CC{CC_MAP['right_stick_x']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_RY:  # Right Stick Y
                                    cc_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    if controller_obj.should_send_cc(CC_MAP['right_stick_y'], cc_val):
                                        midiout.send_message([0xB0, CC_MAP['right_stick_y'], cc_val])
                                        print(f"🕹️  Right Y → CC{CC_MAP['right_stick_y']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_Z:  # L2 Trigger
                                    cc_val = controller_obj.scale_value(event.value, 0, 255)
                                    if controller_obj.should_send_cc(CC_MAP['l2_trigger'], cc_val):
                                        midiout.send_message([0xB0, CC_MAP['l2_trigger'], cc_val])
                                        print(f"🎚️  L2     → CC{CC_MAP['l2_trigger']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_RZ:  # R2 Trigger
                                    cc_val = controller_obj.scale_value(event.value, 0, 255)
                                    if controller_obj.should_send_cc(CC_MAP['r2_trigger'], cc_val):
                                        midiout.send_message([0xB0, CC_MAP['r2_trigger'], cc_val])
                                        print(f"🎚️  R2     → CC{CC_MAP['r2_trigger']:2d}: {cc_val:3d}")

                                # D-pad handling (HAT axes, not buttons!)
                                elif event.code == ecodes.ABS_HAT0X:  # D-pad Left/Right
                                    if event.value == -1:  # Left
                                        note = DPAD_NOTES['left']
                                        note_on = [0x90, note, 100]
                                        midiout.send_message(note_on)
                                        controller_obj.active_notes['dpad_left'] = note
                                        print(f"🎵 D-pad LEFT → Note ON: {note}")
                                    elif event.value == 1:  # Right
                                        note = DPAD_NOTES['right']
                                        note_on = [0x90, note, 100]
                                        midiout.send_message(note_on)
                                        controller_obj.active_notes['dpad_right'] = note
                                        print(f"🎵 D-pad RIGHT → Note ON: {note}")
                                    elif event.value == 0:  # Released
                                        # Turn off both left and right
                                        if 'dpad_left' in controller_obj.active_notes:
                                            note_off = [0x80, controller_obj.active_notes['dpad_left'], 0]
                                            midiout.send_message(note_off)
                                            print(f"🎵 D-pad LEFT → Note OFF: {controller_obj.active_notes['dpad_left']}")
                                            del controller_obj.active_notes['dpad_left']
                                        if 'dpad_right' in controller_obj.active_notes:
                                            note_off = [0x80, controller_obj.active_notes['dpad_right'], 0]
                                            midiout.send_message(note_off)
                                            print(f"🎵 D-pad RIGHT → Note OFF: {controller_obj.active_notes['dpad_right']}")
                                            del controller_obj.active_notes['dpad_right']

                                elif event.code == ecodes.ABS_HAT0Y:  # D-pad Up/Down
                                    if event.value == -1:  # Up
                                        note = DPAD_NOTES['up']
                                        note_on = [0x90, note, 100]
                                        midiout.send_message(note_on)
                                        controller_obj.active_notes['dpad_up'] = note
                                        print(f"🎵 D-pad UP → Note ON: {note}")
                                    elif event.value == 1:  # Down
                                        note = DPAD_NOTES['down']
                                        note_on = [0x90, note, 100]
                                        midiout.send_message(note_on)
                                        controller_obj.active_notes['dpad_down'] = note
                                        print(f"🎵 D-pad DOWN → Note ON: {note}")
                                    elif event.value == 0:  # Released
                                        # Turn off both up and down
                                        if 'dpad_up' in controller_obj.active_notes:
                                            note_off = [0x80, controller_obj.active_notes['dpad_up'], 0]
                                            midiout.send_message(note_off)
                                            print(f"🎵 D-pad UP → Note OFF: {controller_obj.active_notes['dpad_up']}")
                                            del controller_obj.active_notes['dpad_up']
                                        if 'dpad_down' in controller_obj.active_notes:
                                            note_off = [0x80, controller_obj.active_notes['dpad_down'], 0]
                                            midiout.send_message(note_off)
                                            print(f"🎵 D-pad DOWN → Note OFF: {controller_obj.active_notes['dpad_down']}")
                                            del controller_obj.active_notes['dpad_down']

                        # Handle motion sensor events
                        elif device == motion:
                            if event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:  # Tilt X (left/right)
                                    raw = controller_obj.scale_value(event.value, -500, 500)
                                    cc_val = controller_obj.smooth_motion(raw, 'tilt_x', MOTION_SMOOTHING)
                                    # Only send if motion enabled AND significantly tilted
                                    if controller_obj.motion_enabled and abs(cc_val - 64) > TILT_DEADZONE:
                                        if controller_obj.should_send_cc(CC_MAP['tilt_x'], cc_val):
                                            midiout.send_message([0xB0, CC_MAP['tilt_x'], cc_val])
                                            print(f"📐 Tilt X  → CC{CC_MAP['tilt_x']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_Y:  # Tilt Y (forward/back)
                                    raw = controller_obj.scale_value(event.value, 7500, 8500)
                                    cc_val = controller_obj.smooth_motion(raw, 'tilt_y', MOTION_SMOOTHING)
                                    # Only send if motion enabled AND significantly tilted
                                    if controller_obj.motion_enabled and abs(cc_val - 64) > TILT_DEADZONE:
                                        if controller_obj.should_send_cc(CC_MAP['tilt_y'], cc_val):
                                            midiout.send_message([0xB0, CC_MAP['tilt_y'], cc_val])
                                            print(f"📐 Tilt Y  → CC{CC_MAP['tilt_y']:2d}: {cc_val:3d}")

                                elif event.code == ecodes.ABS_RZ:  # Twist (yaw)
                                    raw = controller_obj.scale_value(event.value, -1000, 1000)
                                    cc_val = controller_obj.smooth_motion(raw, 'twist', MOTION_SMOOTHING)
                                    # Only send if motion enabled AND actively rotating
                                    if controller_obj.motion_enabled and abs(cc_val - 64) > GYRO_DEADZONE:
                                        if controller_obj.should_send_cc(CC_MAP['twist'], cc_val):
                                            midiout.send_message([0xB0, CC_MAP['twist'], cc_val])
                                            print(f"🔄 Twist   → CC{CC_MAP['twist']:2d}: {cc_val:3d}")

                        # Handle touchpad events
                        elif device == touchpad:
                            if event.type == ecodes.EV_KEY:
                                if event.code == ecodes.BTN_TOUCH:
                                    # Finger touched/released touchpad
                                    controller_obj.touchpad_active = (event.value == 1)
                                    if event.value == 1:
                                        print("👆 Touchpad: Finger DOWN")
                                    else:
                                        print("👆 Touchpad: Finger UP")

                                elif event.code == ecodes.BTN_LEFT:
                                    # Touchpad clicked (pressed down)
                                    if event.code in NOTE_MAP:
                                        note = NOTE_MAP[event.code]
                                        if event.value == 1:
                                            note_on = [0x90, note, 100]
                                            midiout.send_message(note_on)
                                            controller_obj.active_notes[event.code] = note
                                            print(f"🎵 Touchpad Click → Note ON: {note}")
                                        elif event.value == 0:
                                            note_off = [0x80, note, 0]
                                            midiout.send_message(note_off)
                                            if event.code in controller_obj.active_notes:
                                                del controller_obj.active_notes[event.code]
                                            print(f"🎵 Touchpad Click → Note OFF: {note}")

                            elif event.type == ecodes.EV_ABS:
                                if controller_obj.touchpad_active:
                                    if event.code == ecodes.ABS_X:  # Touchpad X
                                        # Touchpad X range is typically 0-1920
                                        cc_val = controller_obj.scale_value(event.value, 0, 1920)
                                        if controller_obj.should_send_cc(CC_MAP['touchpad_x'], cc_val):
                                            midiout.send_message([0xB0, CC_MAP['touchpad_x'], cc_val])
                                            print(f"👆 Touch X → CC{CC_MAP['touchpad_x']:2d}: {cc_val:3d}")

                                    elif event.code == ecodes.ABS_Y:  # Touchpad Y
                                        # Touchpad Y range is typically 0-1080
                                        cc_val = controller_obj.scale_value(event.value, 0, 1080)
                                        if controller_obj.should_send_cc(CC_MAP['touchpad_y'], cc_val):
                                            midiout.send_message([0xB0, CC_MAP['touchpad_y'], cc_val])
                                            print(f"👆 Touch Y → CC{CC_MAP['touchpad_y']:2d}: {cc_val:3d}")

        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
            # Send note offs for any active notes
            for note in controller_obj.active_notes.values():
                midiout.send_message([0x80, note, 0])
            del midiout
            print("✅ Clean exit!")

    else:
        # No controller - just keep virtual port alive
        try:
            while True:
                time.sleep(1)  # Just wait, virtual port stays open
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
            del midiout
            print("✅ Clean exit!")

if __name__ == "__main__":
    main()
