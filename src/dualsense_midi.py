#!/usr/bin/env python3
import evdev
from evdev import ecodes
import rtmidi
import time
import select
import gc
import os
import ctypes
import threading
from config.mappings import CC_MAP, NOTE_MAP, NRPN_MAP, STICK_DEADZONE, MOTION_THRESHOLD, STICK_CENTER, MOTION_SMOOTHING, TILT_DEADZONE_14BIT, GYRO_DEADZONE_14BIT, LONG_PRESS_DURATION
from state.freeze import FreezeState
from state.loop import LoopState
from state.channel_manager import ChannelManager
from state.clock_source import ClockSource
from midi.controller import MIDIController

# ── Memory management setup ───────────────────────────────────────────────────
gc.set_threshold(200, 5, 2)

try:
    _libc = ctypes.CDLL("libc.so.6")
    def _trim_heap():
        _libc.malloc_trim(0)
except (OSError, AttributeError):
    def _trim_heap():
        pass

_MOTION_INTERVAL = 0.02
_last_motion_time = 0.0
_MOTION_EVENT_SIZE = 24
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
    midiout.open_virtual_port("DualSense_Controller")
    port_name = "DualSense_Controller (Virtual)"
    print(f"✅ MIDI Output: {port_name}")

    # ===== STEP 3b: Open DOReMIDI/CH345 pedal as loop input =====
    #
    # FIX: The original code scanned for the pedal once at startup and never
    # retried. If the pedal wasn't connected at boot, or disconnected mid-session,
    # loop recording of pedal CCs was permanently lost until a full service restart.
    #
    # Fix: _pedal_holder is a mutable list so the reconnect thread can swap the
    # rtmidi object in and out without the callback needing to know.
    # The reconnect thread mirrors the same hot-plug pattern already used in
    # midi_passthrough.py's monitor_loop().

    # Create scanner ONCE and reuse — never re-create in the hot-plug loop.
    # Creating a new rtmidi.MidiIn() every 2 seconds leaks ALSA sequencer
    # clients because Python's GC delays C++ destructor calls. Accumulated
    # clients hit the 64-client kernel limit and break all MIDI services.
    _pedal_scanner = rtmidi.MidiIn()

    def _find_pedal_port(scanner):
        """Scan current ALSA ports and return (index, name) for the pedal, or (None, None)."""
        for i in range(scanner.get_port_count()):
            n = scanner.get_port_name(i)
            if "DOREMiDi" in n or "CH345" in n:
                return (i, n)
        return (None, None)

    def _pedal_callback(message, data=None):
        """Record pedal CC into the active loop when recording.
        Remaps to current DualSense instrument channel so playback goes to the right destination."""
        midi_bytes, _ = message
        loop_state = channel_manager.get_current_loop_state()
        if loop_state.recording and len(midi_bytes) >= 1:
            status   = midi_bytes[0]
            msg_type = status & 0xF0
            target_ch = controller_obj.current_channel - 1  # 0-indexed
            remapped = [msg_type | target_ch] + list(midi_bytes[1:])
            loop_state.record_message(remapped)
        elif loop_state.recording:
            loop_state.record_message(list(midi_bytes))

    # Initial scan
    _initial_idx, _initial_name = _find_pedal_port(_pedal_scanner)
    _pedal_holder = [None]  # list so _reconnect_pedal thread can mutate it

    if _initial_idx is not None:
        _first_in = rtmidi.MidiIn()
        _first_in.ignore_types(sysex=True, timing=True, active_sense=True)
        try:
            _first_in.open_port(_initial_idx)
            _first_in.set_callback(_pedal_callback)
            _pedal_holder[0] = _first_in
            print(f"✅ Pedal input: {_initial_name}")
        except Exception as e:
            print(f"⚠️  Could not open pedal port: {e}")
            del _first_in
    else:
        print("⚠️  Pedal not found at startup — will keep scanning for it...")

    def _reconnect_pedal():
        """Hot-plug monitor for the DOReMIDI pedal. Runs as a daemon thread.

        Checks every 2 seconds whether the pedal port exists in ALSA.
        - If the port appears and _pedal_holder[0] is None → open and attach callback.
        - If the port disappears and _pedal_holder[0] is set → close and clear.
        This mirrors the same pattern used by midi_passthrough.py's monitor_loop().
        """
        while True:
            time.sleep(2.0)
            try:
                found_idx, found_name = _find_pedal_port(_pedal_scanner)
                current = _pedal_holder[0]

                if found_idx is None and current is not None:
                    # Pedal disappeared
                    try:
                        current.cancel_callback()
                        current.close_port()
                    except Exception:
                        pass
                    del current
                    _pedal_holder[0] = None
                    print("❌ Pedal disconnected — waiting for reconnect...")

                elif found_idx is not None and current is None:
                    # Pedal (re)appeared
                    new_in = rtmidi.MidiIn()
                    new_in.ignore_types(sysex=True, timing=True, active_sense=True)
                    try:
                        new_in.open_port(found_idx)
                        new_in.set_callback(_pedal_callback)
                        _pedal_holder[0] = new_in
                        print(f"✅ Pedal reconnected: {found_name}")
                    except Exception as e:
                        print(f"⚠️  Could not reconnect pedal: {e}")
                        try:
                            new_in.cancel_callback()
                        except Exception:
                            pass
                        del new_in

            except Exception as e:
                print(f"⚠️  Pedal monitor error: {e}")

    threading.Thread(target=_reconnect_pedal, daemon=True).start()

    # ===== Print startup info =====
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
        print("   START (Options) → Channel 2 (Turquoise)")
        print("   BOTH together → Channel 3 (Yellow)")
        print("=" * 50)
        print("❄️  FREEZE FEATURE:")
        print("   L3 → Freeze/Unfreeze L2, R2, and Left Stick")
        print("   R3 → Freeze/Unfreeze Right Stick")
        print("   Freeze states are saved per channel!")
        print("=" * 50)
        print("🔴 LOOP RECORDING (per channel):")
        print("   R1 Long Press (>1s) → Start/Stop Recording")
        print("   R1 Short Press → Play/Stop Loop")
        print("   L1 → Clear Loop")
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

                    if device == motion:
                        _motion_now = time.time()
                        if (_motion_now - _last_motion_time) < _MOTION_INTERVAL:
                            try:
                                os.read(motion.fd, _MOTION_EVENT_SIZE * 128)
                            except OSError:
                                pass
                            continue
                        _last_motion_time = _motion_now

                    for event in device.read():
                        if device == controller:
                            if event.type == ecodes.EV_KEY:
                                if event.code == 310:  # BTN_TL (L1)
                                    if event.value == 1:
                                        controller_obj.l1_pressed = True
                                        controller_obj.l1_press_time = time.time()
                                    else:
                                        if controller_obj.r1_pressed:
                                            controller_obj.check_motion_toggle()
                                        elif not controller_obj.l1_r1_combo_used:
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.clear_loop():
                                                controller_obj.update_led_color()
                                                print(f"\n🗑️  Channel {channel_manager.current_channel}: Loop CLEARED (L1)")
                                            else:
                                                print(f"\n⚠️  Channel {channel_manager.current_channel}: Could not clear loop")
                                        else:
                                            controller_obj.l1_r1_combo_used = False
                                        controller_obj.l1_pressed = False

                                elif event.code == 311:  # BTN_TR (R1)
                                    if event.value == 1:
                                        controller_obj.r1_pressed = True
                                        controller_obj.r1_press_time = time.time()
                                    else:
                                        if controller_obj.l1_pressed:
                                            controller_obj.check_motion_toggle()
                                        elif not controller_obj.l1_r1_combo_used:
                                            loop_state = channel_manager.get_current_loop_state()
                                            press_duration = time.time() - controller_obj.r1_press_time

                                            if press_duration >= LONG_PRESS_DURATION:
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
                                                if loop_state.playing:
                                                    loop_state.stop_playback()
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
                                            controller_obj.l1_r1_combo_used = False
                                        controller_obj.r1_pressed = False

                                elif event.code == 317:  # BTN_THUMBL (L3)
                                    l3_pressed = (event.value == 1)
                                    if l3_pressed and not channel_manager.prev_l3:
                                        status = channel_manager.toggle_left_group_freeze()
                                        print(f"\n❄️  Channel {channel_manager.current_channel}: L2/R2/Left Stick {status}")
                                        if status == "FROZEN":
                                            controller_obj.ds.setLeftMotor(100)
                                            controller_obj.ds.setRightMotor(100)
                                            time.sleep(0.05)
                                            controller_obj.ds.setLeftMotor(0)
                                            controller_obj.ds.setRightMotor(0)
                                    channel_manager.prev_l3 = l3_pressed

                                elif event.code == 318:  # BTN_THUMBR (R3)
                                    r3_pressed = (event.value == 1)
                                    if r3_pressed and not channel_manager.prev_r3:
                                        status = channel_manager.toggle_right_stick_freeze()
                                        print(f"\n❄️  Channel {channel_manager.current_channel}: Right Stick {status}")
                                        if status == "FROZEN":
                                            controller_obj.ds.setRightMotor(100)
                                            time.sleep(0.05)
                                            controller_obj.ds.setRightMotor(0)
                                    channel_manager.prev_r3 = r3_pressed

                                elif event.code == 307:  # BTN_NORTH (△)
                                    if event.value == 1:
                                        controller_obj.btn_north_held = True
                                        controller_obj.last_btn_send_time['north'] = time.time()
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_north'], 127]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  △      → CC{CC_MAP['btn_north']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:
                                        controller_obj.btn_north_held = False
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_north'], 0]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  △      → CC{CC_MAP['btn_north']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                                elif event.code == 308:  # BTN_WEST (□)
                                    if event.value == 1:
                                        controller_obj.btn_west_held = True
                                        controller_obj.last_btn_send_time['west'] = time.time()
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_west'], 127]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  □      → CC{CC_MAP['btn_west']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:
                                        controller_obj.btn_west_held = False
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_west'], 0]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  □      → CC{CC_MAP['btn_west']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                                elif event.code == 316:  # BTN_MODE (PlayStation button)
                                    if event.value == 1:
                                        dpad_step = channel_manager.dpad_vertical_steps[controller_obj.current_channel - 1]
                                        center_cc = channel_manager.step_to_cc_value(dpad_step)
                                        loop_state = channel_manager.get_current_loop_state()
                                        controller_obj.strummer.arpeggiate(
                                            midiout,
                                            controller_obj.current_channel,
                                            center_cc=center_cc,
                                            loop_state=loop_state
                                        )

                                elif event.code == 314:  # BTN_SELECT (Create/Share)
                                    if event.value == 1:
                                        controller_obj.select_pressed = True
                                        controller_obj.check_channel_switch(midiout)
                                    else:
                                        controller_obj.select_pressed = False

                                elif event.code == 315:  # BTN_START (Options)
                                    if event.value == 1:
                                        controller_obj.start_pressed = True
                                        controller_obj.check_channel_switch(midiout)
                                    else:
                                        controller_obj.start_pressed = False

                                elif event.code == 304:  # BTN_SOUTH (✕)
                                    if event.value == 1:
                                        controller_obj.btn_south_held = True
                                        controller_obj.last_btn_send_time['south'] = time.time()
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_south'], 127]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  X (✕)  → CC{CC_MAP['btn_south']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:
                                        controller_obj.btn_south_held = False
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_south'], 0]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  X (✕)  → CC{CC_MAP['btn_south']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                                elif event.code == 305:  # BTN_EAST (○)
                                    if event.value == 1:
                                        controller_obj.btn_east_held = True
                                        controller_obj.last_btn_send_time['east'] = time.time()
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_east'], 127]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  O (○)  → CC{CC_MAP['btn_east']:2d}: 127 (Trigger ON) (Ch {controller_obj.current_channel})")
                                    else:
                                        controller_obj.btn_east_held = False
                                        status_byte = controller_obj.get_midi_channel_byte(0xB0)
                                        midi_msg = [status_byte, CC_MAP['btn_east'], 0]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  O (○)  → CC{CC_MAP['btn_east']:2d}:   0 (Trigger OFF) (Ch {controller_obj.current_channel})")

                            elif event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    cc_val, _ = channel_manager.get_left_stick_values(raw_val, channel_manager.current_left_y)
                                    stick_val = int((cc_val / 127.0) * 255)
                                    banked_cc_val = controller_obj.calculate_slice_with_bank(stick_val)
                                    if controller_obj.should_send_cc(CC_MAP['left_stick_x'], banked_cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_x'], banked_cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🕹️  Left X  → CC{CC_MAP['left_stick_x']:2d}: {banked_cc_val:3d} [Bank {controller_obj.slice_bank}] (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    _, cc_val = channel_manager.get_left_stick_values(channel_manager.current_left_x, raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['left_stick_y'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_y'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🕹️  Left Y  → CC{CC_MAP['left_stick_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RX:
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    cc_val, _ = channel_manager.get_right_stick_values(raw_val, channel_manager.current_right_y)
                                    if controller_obj.should_send_cc(CC_MAP['right_stick_x'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_x'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🕹️  Right X → CC{CC_MAP['right_stick_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RY:
                                    raw_val = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                    _, cc_val = channel_manager.get_right_stick_values(channel_manager.current_right_x, raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['right_stick_y'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_y'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🕹️  Right Y → CC{CC_MAP['right_stick_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Z:
                                    raw_val = controller_obj.scale_value(event.value, 0, 255)
                                    cc_val = channel_manager.get_l2_value(raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['l2_trigger'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['l2_trigger'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  L2     → CC{CC_MAP['l2_trigger']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_RZ:
                                    raw_val = controller_obj.scale_value(event.value, 0, 255)
                                    cc_val = channel_manager.get_r2_value(raw_val)
                                    if controller_obj.should_send_cc(CC_MAP['r2_trigger'], cc_val):
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['r2_trigger'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"🎚️  R2     → CC{CC_MAP['r2_trigger']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_HAT0X:
                                    if event.value == -1:
                                        controller_obj.handle_dpad_left()
                                    elif event.value == 1:
                                        controller_obj.handle_dpad_right()

                                elif event.code == ecodes.ABS_HAT0Y:
                                    if event.value == -1:
                                        step = channel_manager.increment_vertical_step()
                                        cc_val = channel_manager.step_to_cc_value(step)
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['dpad_vertical'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"⬆️  D-pad UP    → CC{CC_MAP['dpad_vertical']:2d}: {cc_val:3d} (Step {step}/7) (Ch {controller_obj.current_channel})")
                                    elif event.value == 1:
                                        step = channel_manager.decrement_vertical_step()
                                        cc_val = channel_manager.step_to_cc_value(step)
                                        midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['dpad_vertical'], cc_val]
                                        midiout.send_message(midi_msg)
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.recording:
                                            loop_state.record_message(midi_msg)
                                        print(f"⬇️  D-pad DOWN  → CC{CC_MAP['dpad_vertical']:2d}: {cc_val:3d} (Step {step}/7) (Ch {controller_obj.current_channel})")

                        elif device == motion:
                            if event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:
                                    raw_14 = controller_obj.scale_value(event.value, -500, 500, 0, 16383)
                                    val_14 = controller_obj.smooth_motion_14bit(raw_14, 'tilt_x', MOTION_SMOOTHING)
                                    raw_7 = controller_obj.scale_value(event.value, -500, 500)
                                    cc_val = controller_obj.smooth_motion(raw_7, 'tilt_x', MOTION_SMOOTHING)
                                    controller_obj.update_haptics_from_tilt(cc_val, controller_obj.smoothed_motion['tilt_y'])
                                    if controller_obj.is_motion_enabled() and abs(val_14 - 8192) > TILT_DEADZONE_14BIT:
                                        if controller_obj.should_send_nrpn(NRPN_MAP['tilt_x'], val_14):
                                            controller_obj.send_nrpn(midiout, NRPN_MAP['tilt_x'], val_14)
                                            loop_state = channel_manager.get_current_loop_state()
                                            if loop_state.recording:
                                                ch = controller_obj.get_midi_channel_byte(0xB0)
                                                p = NRPN_MAP['tilt_x']
                                                for msg in ([ch,99,(p>>7)&0x7F],[ch,98,p&0x7F],[ch,6,(val_14>>7)&0x7F],[ch,38,val_14&0x7F]):
                                                    loop_state.record_message(msg)
                                            print(f"📐 Tilt X  → NRPN {NRPN_MAP['tilt_x']}: {val_14:5d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:
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

                                elif event.code == ecodes.ABS_RZ:
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

                        elif device == touchpad:
                            if event.type == ecodes.EV_KEY:
                                if event.code == ecodes.BTN_TOUCH:
                                    controller_obj.touchpad_active = (event.value == 1)
                                    if event.value == 1:
                                        print("👆 Touchpad: Finger DOWN")
                                    else:
                                        print("👆 Touchpad: Finger UP")
                                        controller_obj.update_touchpad(0, 0, False)

                                elif event.code == ecodes.BTN_LEFT:
                                    if event.value == 1:
                                        controller_obj.toggle_quantize()

                            elif event.type == ecodes.EV_ABS:
                                if event.code == ecodes.ABS_X:
                                    touchpad_x = event.value
                                    if controller_obj.touchpad_active:
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.playing:
                                            controller_obj.update_touchpad(touchpad_x, controller_obj.touchpad_y, True)
                                        if not loop_state.playing:
                                            cc_val = controller_obj.scale_value(touchpad_x, 0, 1920)
                                            if controller_obj.should_send_cc(CC_MAP['touchpad_x'], cc_val):
                                                midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_x'], cc_val]
                                                midiout.send_message(midi_msg)
                                                if loop_state.recording:
                                                    loop_state.record_message(midi_msg)
                                                print(f"👆 Touch X → CC{CC_MAP['touchpad_x']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                                elif event.code == ecodes.ABS_Y:
                                    touchpad_y = event.value
                                    if controller_obj.touchpad_active:
                                        loop_state = channel_manager.get_current_loop_state()
                                        if loop_state.playing:
                                            controller_obj.update_touchpad(controller_obj.touchpad_x, touchpad_y, True)
                                        else:
                                            cc_val = controller_obj.scale_value(touchpad_y, 0, 1080)
                                            if controller_obj.should_send_cc(CC_MAP['touchpad_y'], cc_val):
                                                midi_msg = [controller_obj.get_midi_channel_byte(0xB0), CC_MAP['touchpad_y'], cc_val]
                                                midiout.send_message(midi_msg)
                                                if loop_state.recording:
                                                    loop_state.record_message(midi_msg)
                                                print(f"👆 Touch Y → CC{CC_MAP['touchpad_y']:2d}: {cc_val:3d} (Ch {controller_obj.current_channel})")

                controller_obj.send_held_button_ccs(midiout)

                gc.collect(0)

                _now = time.time()
                if _now - _last_trim_time >= _TRIM_INTERVAL:
                    gc.collect()
                    _trim_heap()
                    _last_trim_time = _now

        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")

            for note in controller_obj.active_notes.values():
                midiout.send_message([controller_obj.get_midi_channel_byte(0x80), note, 0])

            controller_obj.cleanup()
            clock_source.stop()

            # ── Pedal cleanup: use _pedal_holder, not a stale local reference ──
            pedal_in = _pedal_holder[0]
            if pedal_in is not None:
                try:
                    pedal_in.cancel_callback()
                    pedal_in.close_port()
                except Exception:
                    pass
                del pedal_in
                _pedal_holder[0] = None

            if controller:
                controller.close()
            if motion:
                motion.close()
            if touchpad:
                touchpad.close()

            del midiout
            print("✅ Clean exit!")

    else:
        # No controller found at startup — keep virtual port alive and rescan.
        # The service starts before evdev input devices are fully enumerated
        # after boot. Rather than dying silently, we poll every 3 seconds and
        # restart main() the moment the DualSense appears.
        print("⏳ Rescanning for DualSense every 3 seconds...")
        try:
            while True:
                time.sleep(3)
                found = [evdev.InputDevice(p) for p in evdev.list_devices()]
                has_controller = any("DualSense" in d.name and
                                     "Motion" not in d.name and
                                     "Touchpad" not in d.name
                                     for d in found)
                for d in found:
                    try:
                        d.close()
                    except Exception:
                        pass
                if has_controller:
                    print("🎮 DualSense detected — restarting main loop...")
                    controller_obj.cleanup()
                    clock_source.stop()
                    del midiout
                    main()  # restart cleanly from the top
                    return
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
            controller_obj.cleanup()
            del midiout
            print("✅ Clean exit!")

if __name__ == "__main__":
    main()