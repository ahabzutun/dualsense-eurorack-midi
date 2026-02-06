#!/usr/bin/env python3
import evdev
from evdev import ecodes
import rtmidi
import time
import select
from pydualsense import pydualsense
import threading
from config.mappings import CC_MAP, NOTE_MAP, STICK_DEADZONE, MOTION_THRESHOLD, STICK_CENTER, MOTION_SMOOTHING, TILT_DEADZONE, GYRO_DEADZONE, LONG_PRESS_DURATION
from state.freeze import FreezeState
from state.loop import LoopState
from state.channel_manager import ChannelManager
from midi.controller import MIDIController

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
        print("  D-Pad (↑↓) → CC 11 (8 steps)")
        print("  D-Pad (←→) → CC 13 (8 steps)")
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
                                                    if loop_state.start_playback(midiout,
                                                        window_position_func=lambda: controller_obj.window_position,
                                                        window_size_func=lambda: controller_obj.window_size):
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
                                                    if loop_state.stop_recording():  # ✅ Correct!
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
                                                    if loop_state.start_playback(midiout,
                                                        window_position_func=lambda: controller_obj.window_position,
                                                        window_size_func=lambda: controller_obj.window_size):  # ✅ Fixed!
                                                        controller_obj.update_led_color()
                                                        print(f"\n▶️  Channel {channel_manager.current_channel}: Loop PLAYING...")
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

                                # D-pad handling (HAT axes, now CC step controllers!)
                                elif event.code == ecodes.ABS_HAT0X:  # D-pad Left/Right
                                    if event.value == -1:  # Left - decrement step
                                        step = channel_manager.decrement_horizontal_step()
                                        cc_val = channel_manager.step_to_cc_value(step)

                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['dpad_horizontal'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"⬅️  D-pad LEFT  → CC{CC_MAP['dpad_horizontal']:2d}: {cc_val:3d} (Step {step}/7) (Ch {controller_obj.current_channel})")

                                    elif event.value == 1:  # Right - increment step
                                        step = channel_manager.increment_horizontal_step()
                                        cc_val = channel_manager.step_to_cc_value(step)

                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['dpad_horizontal'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"➡️  D-pad RIGHT → CC{CC_MAP['dpad_horizontal']:2d}: {cc_val:3d} (Step {step}/7) (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_HAT0Y:  # D-pad Up/Down
                                    if event.value == -1:  # Up - increment step
                                        step = channel_manager.increment_vertical_step()
                                        cc_val = channel_manager.step_to_cc_value(step)

                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['dpad_vertical'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"⬆️  D-pad UP    → CC{CC_MAP['dpad_vertical']:2d}: {cc_val:3d} (Step {step}/7) (Ch {controller_obj.current_channel})")

                                    elif event.value == 1:  # Down - decrement step
                                        step = channel_manager.decrement_vertical_step()
                                        cc_val = channel_manager.step_to_cc_value(step)

                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['dpad_vertical'], cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"⬇️  D-pad DOWN  → CC{CC_MAP['dpad_vertical']:2d}: {cc_val:3d} (Step {step}/7) (Ch {controller_obj.current_channel})")

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
                                        # Reset window when finger lifts
                                        controller_obj.update_touchpad(0, 0, False)

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
                                if event.code == ecodes.ABS_X:  # Touchpad X
                                    touchpad_x = event.value

                                    # Update window position if finger is down and loop is playing
                                    if controller_obj.touchpad_active:
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.playing:
                                            # Update scanning window - Y value comes from next event or stored value
                                            controller_obj.update_touchpad(touchpad_x, controller_obj.touchpad_y, True)

                                        # Also send as CC when NOT manipulating loop
                                        if not loop_state.playing:
                                            cc_val = controller_obj.scale_value(touchpad_x, 0, 1920)
                                            if controller_obj.should_send_cc(CC_MAP['touchpad_x'], cc_val):
                                                midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_x'], cc_val]
                                                midiout.send_message(midi_msg)

                                                # Record to loop
                                                if loop_state.recording:
                                                    loop_state.record_message(midi_msg)

                                                print(f"👆 Touch X → CC{CC_MAP['touchpad_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:  # Touchpad Y
                                    touchpad_y = event.value

                                    # Update window size if finger is down and loop is playing
                                    if controller_obj.touchpad_active:
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.playing:
                                            # Update scanning window
                                            controller_obj.update_touchpad(controller_obj.touchpad_x, touchpad_y, True)

                                        # Also send as CC when NOT manipulating loop
                                        if not loop_state.playing:
                                            cc_val = controller_obj.scale_value(touchpad_y, 0, 1080)
                                            if controller_obj.should_send_cc(CC_MAP['touchpad_y'], cc_val):
                                                midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_y'], cc_val]
                                                midiout.send_message(midi_msg)

                                                # Record to loop
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
