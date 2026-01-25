"""
Freeze state management for DualSense controller
Handles freezing control values per channel
"""

class FreezeState:
    """Manages freeze state for a single channel/instrument"""
    def __init__(self):
        # Left stick press (L3) freezes these:
        self.l2_frozen = False
        self.l2_value = 0

        self.r2_frozen = False
        self.r2_value = 0

        self.left_stick_frozen = False
        self.left_x_value = 0
        self.left_y_value = 0

        # Right stick press (R3) freezes these:
        self.right_stick_frozen = False
        self.right_x_value = 0
        self.right_y_value = 0
