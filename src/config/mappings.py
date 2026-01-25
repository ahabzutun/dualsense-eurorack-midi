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
    'tilt_x': 16,        # Motion Tilt X
    'tilt_y': 17,        # Motion Tilt Y
    'twist': 18,         # Motion Twist
    'touchpad_x': 20,    # Touchpad X
    'touchpad_y': 21,    # Touchpad Y
    'btn_south': 14,     # X button (✕) trigger (General Purpose)
    'btn_east': 15,      # O button (○) trigger (General Purpose)
    'dpad_vertical': 11,   # D-pad Up/Down (Expression)
    'dpad_horizontal': 13, # D-pad Left/Right (Effect Control 1)
}

# Note mapping for buttons
NOTE_MAP = {
    # 304: BTN_SOUTH (✕) - Now CC trigger (CC 14)
    # 305: BTN_EAST (○) - Now CC trigger (CC 15)
    # 307: BTN_NORTH (△) - Now used for LOOP RECORDING
    308: 67,  # BTN_WEST (□) → G4
    272: 72,  # BTN_LEFT (Touchpad Click) → C5
    # 317: BTN_THUMBL (L3 - Left Stick Click) - Now used for FREEZE
    # 318: BTN_THUMBR (R3 - Right Stick Click) - Now used for FREEZE
    # 314: BTN_SELECT (Create/Share) - Used for channel switching
    # 315: BTN_START (Options) - Used for channel switching
}

# Controller Configuration
STICK_DEADZONE = 10      # Ignore changes smaller than this (out of 127)
MOTION_THRESHOLD = 8     # Minimum change to send motion CC
STICK_CENTER = 127       # Stick center value (0-255 range, center ~127)
MOTION_SMOOTHING = 0.3   # Smoothing factor for motion (0-1, lower = smoother)

# Motion deadzones (only send when significantly away from center)
TILT_DEADZONE = 25       # Tilt must be 25+ away from center (64) to send
GYRO_DEADZONE = 30       # Gyro must be 30+ away from center (64) to send

# Loop recording
LONG_PRESS_DURATION = 1.0  # 1 second for long press
