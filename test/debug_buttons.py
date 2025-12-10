#!/usr/bin/env python3
import evdev
from evdev import ecodes

# Find the main DualSense controller
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]

controller = None
for device in devices:
    if "DualSense" in device.name and "Touchpad" not in device.name and "Motion" not in device.name:
        controller = device
        break

if not controller:
    print("Controller not found!")
    exit(1)

print(f"Connected to: {controller.name}")
print("Press buttons to see their codes!\n")

# Read all events
for event in controller.read_loop():
    if event.type == ecodes.EV_KEY:
        # Handle both single names and tuples of names
        button_name = ecodes.BTN.get(event.code, f"UNKNOWN_{event.code}")
        if isinstance(button_name, tuple):
            button_name = button_name[0]  # Take first name from tuple
        print(f"Button: {button_name:20s} Code: {event.code:3d} Value: {event.value}")
    elif event.type == ecodes.EV_ABS:
        # Show D-pad events (they might be analog!)
        axis_name = ecodes.ABS.get(event.code, f"UNKNOWN_{event.code}")
        if isinstance(axis_name, tuple):
            axis_name = axis_name[0]
        if "HAT" in str(axis_name):  # D-pad often shows as HAT
            print(f"D-pad: {axis_name:20s} Code: {event.code:3d} Value: {event.value}")
