#!/usr/bin/env python3
import evdev
from evdev import InputDevice, categorize, ecodes

# Find all devices
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
print("Available devices:")
for device in devices:
    print(f"  {device.path}: {device.name}")

# Look for DualSense Motion Sensors
motion = None
for device in devices:
    if "DualSense" in device.name and "Motion" in device.name:
        motion = device
        break

if motion is None:
    print("\nDualSense Motion Sensors not found!")
    exit(1)

print(f"\nConnected to: {motion.name}")
print("Move and tilt the controller! (Ctrl+C to exit)\n")

# Read motion events
for event in motion.read_loop():
    if event.type == ecodes.EV_ABS:
        # Get axis name
        axis_name = ecodes.ABS[event.code]
        print(f"Motion: {axis_name:12s} = {event.value:6d}")
