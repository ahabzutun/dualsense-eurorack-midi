"""
MIDI CC and Note mappings for DualSense controller
Configuration constants for controller behavior
"""

# MIDI CC Mapping Configuration
CC_MAP = {
    'left_stick_x': 1,   # Mod Wheel
    'left_stick_y': 2,   # Breath
    'right_stick_x': 74, # Filter Cutoff
    'right_stick_y': 71, # Resonance
    'l2_trigger': 7,     # Volume
    'r2_trigger': 10,    # Pan
    'tilt_x': 16,        # Motion Tilt X  (kept for reference, now sent as NRPN)
    'tilt_y': 17,        # Motion Tilt Y  (kept for reference, now sent as NRPN)
    'twist': 18,         # Motion Twist   (kept for reference, now sent as NRPN)
    'touchpad_x': 20,    # Touchpad X
    'touchpad_y': 21,    # Touchpad Y
    'btn_south': 14,     # X button (✕) trigger (General Purpose)
    'btn_east': 15,      # O button (○) trigger (General Purpose)
    'btn_north': 22,     # △ button trigger (General Purpose)
    'btn_west': 23,      # □ button trigger (General Purpose)
    'dpad_vertical': 11,   # D-pad Up/Down (Expression)
    'dpad_horizontal': 13, # D-pad Left/Right (Effect Control 1)
}

# NRPN mapping for high-resolution motion sensor output (14-bit, 0-16383)
# Parameter numbers start from 0 (fresh bank).
# Smoothing and deadzones are applied in 14-bit space before sending.
NRPN_MAP = {
    'tilt_x': 0,   # Motion Tilt X → NRPN 0
    'tilt_y': 1,   # Motion Tilt Y → NRPN 1
    'twist':  2,   # Motion Twist  → NRPN 2
}

# Minimum 14-bit value change before sending NRPN (filters noise at rest).
# Proportional to MOTION_THRESHOLD: 8/127 * 16383 ≈ 1032, rounded to 1024.
NRPN_MOTION_THRESHOLD = 1024

# Note mapping for buttons
NOTE_MAP = {
    # 304: BTN_SOUTH (✕) - CC trigger (CC 14)
    # 305: BTN_EAST (○)  - CC trigger (CC 15)
    # 307: BTN_NORTH (△) - CC trigger (CC 22)
    # 308: BTN_WEST (□)  - CC trigger (CC 23)
    272: 72,  # BTN_LEFT (Touchpad Click) → C5
    # 317: BTN_THUMBL (L3 - Left Stick Click) - Used for FREEZE
    # 318: BTN_THUMBR (R3 - Right Stick Click) - Used for FREEZE
    # 314: BTN_SELECT (Create/Share) - Used for channel switching
    # 315: BTN_START (Options) - Used for channel switching
}

# Controller Configuration
STICK_DEADZONE = 10      # Ignore changes smaller than this (out of 127)
MOTION_THRESHOLD = 8     # Minimum change to send motion CC (7-bit, legacy)
STICK_CENTER = 127       # Stick center value (0-255 range, center ~127)
MOTION_SMOOTHING = 0.3   # Smoothing factor for motion (0-1, lower = smoother)

# Motion deadzones
# 7-bit versions (legacy, kept for reference):
TILT_DEADZONE = 25       # Tilt must be 25+ away from center (64) to send
GYRO_DEADZONE = 30       # Gyro must be 30+ away from center (64) to send

# 14-bit equivalents (used for NRPN sends, center = 8192):
TILT_DEADZONE_14BIT = 3225   # 25/64 * 8192
GYRO_DEADZONE_14BIT = 3840   # 30/64 * 8192

# Loop recording
LONG_PRESS_DURATION = 1.0  # 1 second for long press
