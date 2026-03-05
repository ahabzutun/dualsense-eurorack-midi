#!/usr/bin/env python3
import evdev
from evdev import ecodes
import rtmidi
import time
import select
import gc
import os
import ctypes
from pydualsense import pydualsense
import threading
from config.mappings import CC_MAP, NOTE_MAP, NRPN_MAP, STICK_DEADZONE, MOTION_THRESHOLD, STICK_CENTER, MOTION_SMOOTHING, TILT_DEADZONE_14BIT, GYRO_DEADZONE_14BIT, LONG_PRESS_DURATION
from state.freeze import FreezeState
from state.loop import LoopState
from state.channel_manager import ChannelManager
from state.clock_source import ClockSource
from midi.controller import MIDIController

# ── Memory management setup ───────────────────────────────────────────────────
# The DualSense motion sensor generates ~1500 evdev InputEvent objects/second
# even when the controller is idle. Python's default GC thresholds (700,10,10)
# can't keep up, causing RSS to grow ~5 MB/10s continuously.
#
# Fix 1: tighten GC thresholds so gen0 collects much more frequently.
gc.set_threshold(200, 5, 2)

# Fix 2: load libc so we can call malloc_trim() to return freed pages to the OS.
# GC alone is not enough — Python holds onto freed memory pages indefinitely.
try:
    _libc = ctypes.CDLL("libc.so.6")
    def _trim_heap():
        _libc.malloc_trim(0)
except (OSError, AttributeError):
    def _trim_heap():
        pass  # non-Linux fallback (e.g. macOS during dev)

# Motion sensor rate limit: process IMU at max 50 Hz (every 20ms).
# We still READ all events from the kernel buffer (to drain it), but only
# run the MIDI/smoothing logic on the first event per 20ms window.
_MOTION_INTERVAL = 0.02   # seconds between motion processing
_last_motion_time = 0.0

# struct input_event size on 64-bit Linux:
#   struct timeval  = 8 (tv_sec) + 8 (tv_usec)  = 16 bytes
#   __u16 type                                   =  2 bytes
#   __u16 code                                   =  2 bytes
#   __s32 value                                  =  4 bytes
#   Total                                        = 24 bytes
# Used for OS-level draining (no Python objects created).
_MOTION_EVENT_SIZE = 24

# Heap trim interval: return freed pages to OS every 30 seconds.
_TRIM_INTERVAL   = 30.0
_last_trim_time  = 0.0

def main():
    global _last_motion_time, _last_trim_time
    _last_trim_time = time.time()

    # Initialize channel manager first
    channel_manager = ChannelManager()

    # Initialize and start clock source (NerdSEQ external clock, internal BPM fallback)
    clock_source = ClockSource(internal_bpm=120)
    clock_source.start()

    # Pass channel manager and clock source to MIDIController
    controller_obj = MIDIController(channel_manager, clock_source=clock_source)

    # When NerdSEQ clock lock state changes, immediately refresh LED colour
    def _on_sync_change(is_synced):
        if controller_obj.quantize_on:
            controller_obj.update_led_color()

    clock_source.on_sync_change = _on_sync_change

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

    print(f"✅ MIDI Output: {port_name}")

    # ===== STEP 3b: Open DOReMIDI/CH345 pedal as loop input =====
    # Pedal messages are forwarded live by the passthrough service.
    # We also open the pedal here so its CCs get recorded into the looper.
    pedal_in = rtmidi.MidiIn()
    pedal_in.ignore_types(sysex=True, timing=True, active_sense=True)
    _pedal_port_index = None
    for i in range(pedal_in.get_port_count()):
        name = pedal_in.get_port_name(i)
        if "DOREMiDi" in name or "CH345" in name:
            _pedal_port_index = i
            print(f"✅ Pedal input: {name}")
            break

    if _pedal_port_index is not None:
        pedal_in.open_port(_pedal_port_index)
        def _pedal_callback(message, data=None):
            """Record pedal CC into the active loop when recording.
            Remap to current instrument channel so playback goes to the right destination."""
            midi_bytes, _ = message
            loop_state = channel_manager.get_current_loop_state()
            if loop_state.recording and len(midi_bytes) >= 1:
                # Remap channel byte to current DualSense instrument channel
                status   = midi_bytes[0]
                msg_type = status & 0xF0
                target_ch = controller_obj.current_channel - 1  # 0-indexed
                remapped = [msg_type | target_ch] + list(midi_bytes[1:])
                loop_state.record_message(remapped)
            elif loop_state.recording:
                loop_state.record_message(list(midi_bytes))
        pedal_in.set_callback(_pedal_callback)
    else:
        print("⚠️  Pedal not found — pedal CCs will not be recorded into loops")
        pedal_in = None


    if controller_available:
        print("🎮 DualSense → 🎹 MIDI → 🎛️ Passthrough Service")
        print("=" * 50)
        print("Controls:")
        print("  Buttons (✕○△□) → CC Triggers (14, 15, 22, 23) - with MIDI learn repeat")
        print("  D-Pad (↑↓) → CC 11 (8 steps)")
        print("  D-Pad (←→) → CC 13 (8 steps)")
        print("  Touchpad Click → Toggle Quantize ON/OFF (dots show subdivision)")
        print("  L3 (Left Stick Click) → FREEZE L2/R2/Left Stick ❄️")
        print("  R3 (Right Stick Click) → FREEZE Right Stick ❄️")
        print("  Sticks → CC 1,2,74,71")
        print("  Triggers → CC 7,10")
        print("  Touchpad X  → Scrub loop position")
        print("  Touchpad Y  → Quantize subdivision (top→bottom: 1/32 / 1/16 / 1/8 / 1/4)")
        print("  Motion → NRPN 0,1,2 (14-bit tilt X/Y, twist) (Press L1+R1 to toggle)")
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

                    # ── Motion device: rate-limited at the READ level ────────
                    # device.read() calls evdev's device_read_many() which
                    # allocates a Python InputEvent object for EVERY kernel
                    # event — ~1500/sec from the IMU alone, even when still.
                    # Fix: when it's not time to process motion, drain the
                    # kernel buffer with os.read() (raw bytes → zero Python
                    # objects). Only call device.read() at 50 Hz.
                    if device == motion:
                        _motion_now = time.time()
                        if (_motion_now - _last_motion_time) < _MOTION_INTERVAL:
                            # Drain kernel buffer with zero Python allocations
                            try:
                                os.read(motion.fd, _MOTION_EVENT_SIZE * 128)
                            except OSError:
                                pass
                            continue  # skip device.read() entirely
                        _last_motion_time = _motion_now

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
                                        if controller_obj.r1_pressed:
                                            # Both held → Motion toggle (combo)
                                            controller_obj.check_motion_toggle()
                                            # combo_used flag is set inside check_motion_toggle
                                        elif not controller_obj.l1_r1_combo_used:
                                            # Solo L1 release: Clear loop
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.clear_loop():
                                                controller_obj.update_led_color()
                                                print(f"\n🗑️  Channel {channel_manager.current_channel}: Loop CLEARED (L1)")
                                            else:
                                                print(f"\n⚠️  Channel {channel_manager.current_channel}: Could not clear loop")
                                        else:
                                            # L1 was the second button released after a combo — skip solo action
                                            controller_obj.l1_r1_combo_used = False

                                        controller_obj.l1_pressed = False

                                elif event.code == 311:  # BTN_TR (R1)
                                    if event.value == 1:  # Button pressed
                                        controller_obj.r1_pressed = True
                                        controller_obj.r1_press_time = time.time()
                                    else:  # Button released
                                        if controller_obj.l1_pressed:
                                            # Both held → Motion toggle (combo)
                                            controller_obj.check_motion_toggle()
                                            # combo_used flag is set inside check_motion_toggle
                                        elif not controller_obj.l1_r1_combo_used:
                                            # Solo R1 release: Loop recording controls
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
                                                    # Auto-exit quantize when loop stops
                                                    if controller_obj.quantize_on:
                                                        controller_obj.quantize_on = False
                                                        loop_state.quantize_subdivision = None
                                                        controller_obj.update_player_dots()
                                                    controller_obj.update_led_color()
                                                    print(f"\n⏸️  Channel {channel_manager.current_channel}: Playback STOPPED (R1)")
                                                elif loop_state.midi_buffer:
                                                    if loop_state.start_playback(midiout,
                                                        window_position_func=lambda: controller_obj.window_position,
                                                        bpm_func=lambda: clock_source.get_current_bpm()):
                                                        controller_obj.update_led_color()
                                                        print(f"\n▶️  Channel {channel_manager.current_channel}: Playing loop ({loop_state.loop_duration:.1f}s, {len(loop_state.midi_buffer)} events) (R1)")
                                                else:
                                                    print(f"\n⚠️  Channel {channel_manager.current_channel}: No loop to play")
                                        else:
                                            # R1 was the second button released after a combo — skip solo action
                                            controller_obj.l1_r1_combo_used = False

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

                                # CC Trigger: △ button
                                elif event.code == 307:  # BTN_NORTH (△)
                                    if event.value == 1:  # Pressed
                                        controller_obj.btn_north_held = True
                                        controller_obj.last_btn_send_time['north'] = time.time()
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_north'], 127]
                                        midiout.send_message(midi_msg)

                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  △      → CC{CC_MAP['btn_north']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:  # Released
                                        controller_obj.btn_north_held = False
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_north'], 0]
                                        midiout.send_message(midi_msg)

                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  △      → CC{CC_MAP['btn_north']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                                # CC Trigger: □ button
                                elif event.code == 308:  # BTN_WEST (□)
                                    if event.value == 1:  # Pressed
                                        controller_obj.btn_west_held = True
                                        controller_obj.last_btn_send_time['west'] = time.time()
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_west'], 127]
                                        midiout.send_message(midi_msg)

                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  □      → CC{CC_MAP['btn_west']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:  # Released
                                        controller_obj.btn_west_held = False
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_west'], 0]
                                        midiout.send_message(midi_msg)

                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🎚️  □      → CC{CC_MAP['btn_west']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                                # PlayStation button for harmonic strumming
                                elif event.code == 316:  # BTN_MODE (PlayStation button)
                                    if event.value == 1:  # Pressed
                                        # Get current D-pad vertical step
                                        dpad_step = channel_manager.dpad_vertical_steps[controller_obj.current_channel - 1]
                                        center_cc = channel_manager.step_to_cc_value(dpad_step)

                                        # Get current loop state for recording
                                        loop_state = channel_manager.get_current_loop_state()

                                        # Trigger strum on current channel around current pitch
                                        controller_obj.strummer.arpeggiate(
                                            midiout,
                                            controller_obj.current_channel,
                                            center_cc=center_cc,
                                            loop_state=loop_state
                                        )

                                # SELECT button for channel switching
                                elif event.code == 314:  # BTN_SELECT (Create/Share)
                                    if event.value == 1:  # Pressed
                                        controller_obj.select_pressed = True
                                        controller_obj.check_channel_switch(midiout)
                                    else:  # Released
                                        controller_obj.select_pressed = False

                                # START button for channel switching
                                elif event.code == 315:  # BTN_START (Options)
                                    if event.value == 1:  # Pressed
                                        controller_obj.start_pressed = True
                                        controller_obj.check_channel_switch(midiout)
                                    else:  # Released
                                        controller_obj.start_pressed = False

                                # CC Triggers: X button
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

                                # CC Triggers: O button
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
                                if event.code == ecodes.ABS_X:  # Left Stick X (with slice banking)
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    # Get frozen or live value (this handles freeze state)
                                    cc_val, _ = channel_manager.get_left_stick_values(raw_val, channel_manager.current_left_y)

                                    # Apply slice banking to the CC value (0-127 range)
                                    # Scale cc_val (0-127) to stick range (0-255) for banking calculation
                                    stick_val = int((cc_val / 127.0) * 255)
                                    banked_cc_val = controller_obj.calculate_slice_with_bank(stick_val)

                                    if controller_obj.should_send_cc(CC_MAP['left_stick_x'], banked_cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_x'], banked_cc_val]
                                        midiout.send_message(midi_msg)

                                        # Record to loop
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)

                                        print(f"🕹️  Left X  → CC{CC_MAP['left_stick_x']:2d}: {banked_cc_val:3d} [Bank {controller_obj.slice_bank}] (Ch {controller_obj.current_channel})")

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
                                    if event.value == -1:  # Left - decrease slice bank
                                        controller_obj.handle_dpad_left()
                                    elif event.value == 1:  # Right - increase slice bank
                                        controller_obj.handle_dpad_right()

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
                                    # Scale raw (-500..500) → 14-bit (0..16383), smooth in 14-bit space
                                    raw_14 = controller_obj.scale_value(event.value, -500, 500, 0, 16383)
                                    val_14 = controller_obj.smooth_motion_14bit(raw_14, 'tilt_x', MOTION_SMOOTHING)
                                    # Also keep 7-bit smoothed value for haptics
                                    raw_7 = controller_obj.scale_value(event.value, -500, 500)
                                    cc_val = controller_obj.smooth_motion(raw_7, 'tilt_x', MOTION_SMOOTHING)
                                    controller_obj.update_haptics_from_tilt(cc_val, controller_obj.smoothed_motion['tilt_y'])
                                    if controller_obj.is_motion_enabled() and abs(val_14 - 8192) > TILT_DEADZONE_14BIT:
                                        if controller_obj.should_send_nrpn(NRPN_MAP['tilt_x'], val_14):
                                            controller_obj.send_nrpn(midiout, NRPN_MAP['tilt_x'], val_14)
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                # Record as 4 individual CC messages that make up the NRPN
                                                ch = controller_obj.get_midi_channel_byte(0xB0)
                                                p = NRPN_MAP['tilt_x']
                                                for msg in ([ch,99,(p>>7)&0x7F],[ch,98,p&0x7F],[ch,6,(val_14>>7)&0x7F],[ch,38,val_14&0x7F]):
                                                    loop_state.record_message(msg)
                                            print(f"📐 Tilt X  → NRPN {NRPN_MAP['tilt_x']}: {val_14:5d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:  # Tilt Y (forward/back)
                                    raw_14 = controller_obj.scale_value(event.value, 7500, 8500, 0, 16383)
                                    val_14 = controller_obj.smooth_motion_14bit(raw_14, 'tilt_y', MOTION_SMOOTHING)
                                    raw_7 = controller_obj.scale_value(event.value, 7500, 8500)
                                    cc_val = controller_obj.smooth_motion(raw_7, 'tilt_y', MOTION_SMOOTHING)
                                    controller_obj.update_haptics_from_tilt(controller_obj.smoothed_motion['tilt_x'], cc_val)
                                    if controller_obj.is_motion_enabled() and abs(val_14 - 8192) > TILT_DEADZONE_14BIT:
                                        if controller_obj.should_send_nrpn(NRPN_MAP['tilt_y'], val_14):
                                            controller_obj.send_nrpn(midiout, NRPN_MAP['tilt_y'], val_14)
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                ch = controller_obj.get_midi_channel_byte(0xB0)
                                                p = NRPN_MAP['tilt_y']
                                                for msg in ([ch,99,(p>>7)&0x7F],[ch,98,p&0x7F],[ch,6,(val_14>>7)&0x7F],[ch,38,val_14&0x7F]):
                                                    loop_state.record_message(msg)
                                            print(f"📐 Tilt Y  → NRPN {NRPN_MAP['tilt_y']}: {val_14:5d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RZ:  # Twist (yaw)
                                    raw_14 = controller_obj.scale_value(event.value, -1000, 1000, 0, 16383)
                                    val_14 = controller_obj.smooth_motion_14bit(raw_14, 'twist', MOTION_SMOOTHING)
                                    if controller_obj.is_motion_enabled() and abs(val_14 - 8192) > GYRO_DEADZONE_14BIT:
                                        if controller_obj.should_send_nrpn(NRPN_MAP['twist'], val_14):
                                            controller_obj.send_nrpn(midiout, NRPN_MAP['twist'], val_14)
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                ch = controller_obj.get_midi_channel_byte(0xB0)
                                                p = NRPN_MAP['twist']
                                                for msg in ([ch,99,(p>>7)&0x7F],[ch,98,p&0x7F],[ch,6,(val_14>>7)&0x7F],[ch,38,val_14&0x7F]):
                                                    loop_state.record_message(msg)
                                            print(f"🔄 Twist   → NRPN {NRPN_MAP['twist']}: {val_14:5d} (Ch {controller_obj.current_channel})")

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
                                    # Touchpad physical click → toggle quantize on/off
                                    if event.value == 1:  # Press only (not release)
                                        controller_obj.toggle_quantize()

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

                                    if controller_obj.touchpad_active:
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.playing:
                                            # Y controls quantization subdivision
                                            controller_obj.update_touchpad(controller_obj.touchpad_x, touchpad_y, True)
                                        else:
                                            # Not playing: send as CC
                                            cc_val = controller_obj.scale_value(touchpad_y, 0, 1080)
                                            if controller_obj.should_send_cc(CC_MAP['touchpad_y'], cc_val):
                                                midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_y'], cc_val]
                                                midiout.send_message(midi_msg)
                                                if loop_state.recording:
                                                    loop_state.record_message(midi_msg)
                                                print(f"👆 Touch Y → CC{CC_MAP['touchpad_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                # After processing all events, send repeated CCs for held buttons
                controller_obj.send_held_button_ccs(midiout)

                # ── Periodic memory housekeeping ────────────────────────────
                # gen0 GC every iteration (very cheap, <0.1ms). Prevents the
                # 16,000 evdev InputEvent objects/sec from piling up faster
                # than the default GC thresholds can collect them.
                gc.collect(0)

                # Every 30 seconds: full collection + return freed pages to OS.
                # Python holds onto freed memory internally; malloc_trim() is
                # the only way to actually give it back to the kernel.
                _now = time.time()
                if _now - _last_trim_time >= _TRIM_INTERVAL:
                    gc.collect()
                    _trim_heap()
                    _last_trim_time = _now

        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")

            # Send note offs for any active notes
            for note in controller_obj.active_notes.values():
                midiout.send_message([controller_obj.get_midi_channel_byte(0x80), note, 0])

            controller_obj.cleanup()
            clock_source.stop()

            if pedal_in is not None:
                pedal_in.close_port()
                del pedal_in

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
