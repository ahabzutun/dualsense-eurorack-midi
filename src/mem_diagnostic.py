#!/usr/bin/env python3
"""
Memory leak diagnostic for dualsense-eurorack-midi.
Run this ON THE PI instead of the main script:

    cd ~/dualsense-eurorack-midi/src
    python3 mem_diagnostic.py

It takes two snapshots 30 seconds apart and prints the top 20 growing
allocation sites, ranked by bytes. This will tell us exactly which line
of which file is responsible for the leak.
"""

import tracemalloc
import linecache
import time
import gc
import os
import sys
import threading

# ── Start tracemalloc before importing anything else ──────────────────────────
tracemalloc.start(10)  # keep 10-frame tracebacks

snapshot1 = None

def take_snapshot(label):
    gc.collect()  # force GC so we only see true leaks, not GC backlog
    snap = tracemalloc.take_snapshot()
    print(f"\n{'='*60}")
    print(f"  SNAPSHOT: {label}")
    print(f"  Process RSS: {get_rss_mb():.1f} MB")
    print(f"{'='*60}")
    return snap

def get_rss_mb():
    try:
        with open(f"/proc/{os.getpid()}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024
    except Exception:
        pass
    return 0

def print_diff(snap1, snap2):
    stats = snap2.compare_to(snap1, 'lineno')
    print(f"\n{'='*60}")
    print("  TOP 20 GROWING ALLOCATIONS (since last snapshot)")
    print(f"{'='*60}")
    for i, stat in enumerate(stats[:20]):
        if stat.size_diff <= 0:
            continue
        frame = stat.traceback[0]
        print(f"\n#{i+1:2d}  +{stat.size_diff/1024:.1f} KB  ({stat.count_diff:+d} objects)")
        print(f"     File: {frame.filename}:{frame.lineno}")
        line = linecache.getline(frame.filename, frame.lineno).strip()
        if line:
            print(f"     Code: {line}")

def print_top_allocs(snap):
    stats = snap.statistics('lineno')
    print(f"\n{'='*60}")
    print("  TOP 20 TOTAL ALLOCATIONS RIGHT NOW")
    print(f"{'='*60}")
    for i, stat in enumerate(stats[:20]):
        frame = stat.traceback[0]
        print(f"\n#{i+1:2d}  {stat.size/1024:.1f} KB  ({stat.count} objects)")
        print(f"     File: {frame.filename}:{frame.lineno}")
        line = linecache.getline(frame.filename, frame.lineno).strip()
        if line:
            print(f"     Code: {line}")

# ── Now import the actual application code ────────────────────────────────────
print("Importing application modules...")

import evdev
from evdev import ecodes
import rtmidi
import select as sel_module
from pydualsense import pydualsense
from config.mappings import CC_MAP, NOTE_MAP, STICK_DEADZONE, MOTION_THRESHOLD, STICK_CENTER, MOTION_SMOOTHING, TILT_DEADZONE, GYRO_DEADZONE, LONG_PRESS_DURATION
from state.freeze import FreezeState
from state.loop import LoopState
from state.channel_manager import ChannelManager
from midi.controller import MIDIController

print("Imports done. Initialising controller objects...")

# ── Initialise exactly the same objects the main script creates ────────────────
channel_manager = ChannelManager()
controller_obj = MIDIController(channel_manager)

devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
controller = motion = touchpad = None
for device in devices:
    if "DualSense" in device.name:
        if "Motion" in device.name:
            motion = device
        elif "Touchpad" in device.name:
            touchpad = device
        elif "Touchpad" not in device.name and "Motion" not in device.name:
            controller = device

controller_available = (controller is not None and motion is not None and touchpad is not None)
if controller_available:
    print(f"✅ Controller: {controller.name}")
    print(f"✅ Motion:     {motion.name}")
    print(f"✅ Touchpad:   {touchpad.name}")
else:
    print("⚠️  DualSense not fully detected — some leak sources may not be visible")

midiout = rtmidi.MidiOut()
midiout.open_virtual_port("DualSense_Diagnostic")

print("\nStarting diagnostic loop. Will run for 90 seconds with snapshots at 0s, 30s, 60s, 90s.\n")
print("DO NOT TOUCH THE CONTROLLER during this test.\n")

# ── Snapshot 0 — baseline after init ──────────────────────────────────────────
time.sleep(2)   # let background threads settle
snap0 = take_snapshot("BASELINE (t=0s)")
print_top_allocs(snap0)

# ── Run the exact same event loop as the main script ──────────────────────────
if controller_available:
    devices_dict = {
        controller.fd: controller,
        motion.fd: motion,
        touchpad.fd: touchpad,
    }

    start_time = time.time()
    next_snapshot_at = [30, 60, 90]
    last_snap = snap0

    try:
        while True:
            elapsed = time.time() - start_time

            # Take snapshots at 30s, 60s, 90s
            if next_snapshot_at and elapsed >= next_snapshot_at[0]:
                t = next_snapshot_at.pop(0)
                snap = take_snapshot(f"t={t}s (elapsed={elapsed:.0f}s)")
                print_diff(last_snap, snap)
                last_snap = snap

                if not next_snapshot_at:
                    print("\n✅ Diagnostic complete. Review output above to identify the leak.")
                    break

            r, w, x = sel_module.select(devices_dict.keys(), [], [], 0.01)

            for fd in r:
                device = devices_dict[fd]
                for event in device.read():
                    if device == controller:
                        if event.type == ecodes.EV_ABS:
                            if event.code == ecodes.ABS_X:
                                raw = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                cc_val, _ = channel_manager.get_left_stick_values(raw, channel_manager.current_left_y)
                            elif event.code == ecodes.ABS_RX:
                                raw = controller_obj.apply_deadzone(event.value, STICK_CENTER, STICK_DEADZONE)
                                cc_val, _ = channel_manager.get_right_stick_values(raw, channel_manager.current_right_y)
                            elif event.code == ecodes.ABS_Z:
                                raw = controller_obj.scale_value(event.value, 0, 255)
                                channel_manager.get_l2_value(raw)
                            elif event.code == ecodes.ABS_RZ:
                                raw = controller_obj.scale_value(event.value, 0, 255)
                                channel_manager.get_r2_value(raw)

                    elif device == motion:
                        if event.type == ecodes.EV_ABS:
                            if event.code == ecodes.ABS_X:
                                raw = controller_obj.scale_value(event.value, -500, 500)
                                cc_val = controller_obj.smooth_motion(raw, 'tilt_x', MOTION_SMOOTHING)
                                controller_obj.update_haptics_from_tilt(cc_val, controller_obj.smoothed_motion['tilt_y'])
                            elif event.code == ecodes.ABS_Y:
                                raw = controller_obj.scale_value(event.value, 7500, 8500)
                                cc_val = controller_obj.smooth_motion(raw, 'tilt_y', MOTION_SMOOTHING)
                                controller_obj.update_haptics_from_tilt(controller_obj.smoothed_motion['tilt_x'], cc_val)
                            elif event.code == ecodes.ABS_RZ:
                                raw = controller_obj.scale_value(event.value, -1000, 1000)
                                controller_obj.smooth_motion(raw, 'twist', MOTION_SMOOTHING)

    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted by user.")
else:
    # No controller — just watch what pydualsense allocates on its own
    print("Running without controller — watching pydualsense background thread only.")
    start_time = time.time()
    next_snapshot_at = [30, 60, 90]
    last_snap = snap0
    try:
        while next_snapshot_at:
            elapsed = time.time() - start_time
            if elapsed >= next_snapshot_at[0]:
                t = next_snapshot_at.pop(0)
                snap = take_snapshot(f"t={t}s")
                print_diff(last_snap, snap)
                last_snap = snap
            time.sleep(1)
    except KeyboardInterrupt:
        pass

# Cleanup
controller_obj.cleanup()
del midiout
tracemalloc.stop()
print("\nDone.")
