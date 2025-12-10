#!/usr/bin/env python3
import evdev
from evdev import InputDevice, categorize, ecodes

# Find the DualSense controller
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
print("Available devices:")
for device in devices:
    print(f"  {device.path}: {device.name}")

# Look for DualSense main controller (not touchpad or motion sensors)
dualsense = None
for device in devices:
    if "DualSense" in device.name and "Touchpad" not in device.name and "Motion" not in device.name:
        dualsense = device
        break

if dualsense is None:
    print("\nDualSense controller not found! Make sure it's plugged in.")
    exit(1)

print(f"\nConnected to: {dualsense.name}")
print("Press buttons and move sticks! (Ctrl+C to exit)\n")

# Read events
for event in dualsense.read_loop():
    if event.type == ecodes.EV_KEY:
        print(f"Button: {ecodes.BTN[event.code]} = {event.value}")
    elif event.type == ecodes.EV_ABS:
        print(f"Axis: {ecodes.ABS[event.code]} = {event.value}")
