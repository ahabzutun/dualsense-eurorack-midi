"""
Channel management for DualSense MIDI controller
Manages 3 independent channels with freeze states, loop states, and D-pad step tracking
"""

from state.freeze import FreezeState
from state.loop import LoopState


class ChannelManager:
    """Manages the 3 channels and their freeze states"""
    def __init__(self):
        self.current_channel = 1  # 1, 2, 3 to match the main code
        self.freeze_states = [FreezeState() for _ in range(3)]
        self.loop_states = [LoopState() for _ in range(3)]

        # D-pad step tracking (per channel)
        self.dpad_vertical_steps = [0, 0, 0]     # Up/Down for each channel (0-7)
        self.dpad_horizontal_steps = [0, 0, 0]   # Left/Right for each channel (0-7)

        # Track previous button states for edge detection
        self.prev_l3 = False
        self.prev_r3 = False

        # Track current raw values (updated continuously)
        self.current_l2 = 0
        self.current_r2 = 0
        self.current_left_x = 0
        self.current_left_y = 0
        self.current_right_x = 0
        self.current_right_y = 0

    def get_current_freeze_state(self):
        """Get the freeze state for the current channel (1-indexed to 0-indexed)"""
        return self.freeze_states[self.current_channel - 1]

    def get_current_loop_state(self):
        """Get the loop state for the current channel"""
        return self.loop_states[self.current_channel - 1]

    def toggle_left_group_freeze(self):
        """Toggle freeze for L2, R2, and left stick"""
        state = self.get_current_freeze_state()

        if state.left_stick_frozen:
            # Unfreeze everything
            state.l2_frozen = False
            state.r2_frozen = False
            state.left_stick_frozen = False
            return "UNFROZEN"
        else:
            # Freeze current values
            state.l2_frozen = True
            state.l2_value = self.current_l2

            state.r2_frozen = True
            state.r2_value = self.current_r2

            state.left_stick_frozen = True
            state.left_x_value = self.current_left_x
            state.left_y_value = self.current_left_y
            return "FROZEN"

    def toggle_right_stick_freeze(self):
        """Toggle freeze for right stick"""
        state = self.get_current_freeze_state()

        if state.right_stick_frozen:
            # Unfreeze
            state.right_stick_frozen = False
            return "UNFROZEN"
        else:
            # Freeze current values
            state.right_stick_frozen = True
            state.right_x_value = self.current_right_x
            state.right_y_value = self.current_right_y
            return "FROZEN"

    def get_l2_value(self, current_value):
        """Get L2 value (frozen or live)"""
        self.current_l2 = current_value  # Always update current value
        state = self.get_current_freeze_state()
        return state.l2_value if state.l2_frozen else current_value

    def get_r2_value(self, current_value):
        """Get R2 value (frozen or live)"""
        self.current_r2 = current_value
        state = self.get_current_freeze_state()
        return state.r2_value if state.r2_frozen else current_value

    def get_left_stick_values(self, current_x, current_y):
        """Get left stick values (frozen or live)"""
        self.current_left_x = current_x
        self.current_left_y = current_y
        state = self.get_current_freeze_state()
        if state.left_stick_frozen:
            return state.left_x_value, state.left_y_value
        return current_x, current_y

    def get_right_stick_values(self, current_x, current_y):
        """Get right stick values (frozen or live)"""
        self.current_right_x = current_x
        self.current_right_y = current_y
        state = self.get_current_freeze_state()
        if state.right_stick_frozen:
            return state.right_x_value, state.right_y_value
        return current_x, current_y

    def increment_vertical_step(self):
        """Increment Up/Down step (max 7)"""
        idx = self.current_channel - 1
        self.dpad_vertical_steps[idx] = min(7, self.dpad_vertical_steps[idx] + 1)
        return self.dpad_vertical_steps[idx]

    def decrement_vertical_step(self):
        """Decrement Up/Down step (min 0)"""
        idx = self.current_channel - 1
        self.dpad_vertical_steps[idx] = max(0, self.dpad_vertical_steps[idx] - 1)
        return self.dpad_vertical_steps[idx]

    def increment_horizontal_step(self):
        """Increment Left/Right step (max 7)"""
        idx = self.current_channel - 1
        self.dpad_horizontal_steps[idx] = min(7, self.dpad_horizontal_steps[idx] + 1)
        return self.dpad_horizontal_steps[idx]

    def decrement_horizontal_step(self):
        """Decrement Left/Right step (min 0)"""
        idx = self.current_channel - 1
        self.dpad_horizontal_steps[idx] = max(0, self.dpad_horizontal_steps[idx] - 1)
        return self.dpad_horizontal_steps[idx]

    def step_to_cc_value(self, step):
        """Convert step (0-7) to CC value (0-127)"""
        # 8 steps: 0, 18, 36, 54, 72, 90, 108, 127
        if step == 7:
            return 127
        return step * 18

    def send_frozen_values_on_channel_switch(self, midiout, controller_obj, CC_MAP):
        """Send frozen values immediately when switching channels"""
        state = self.get_current_freeze_state()

        if state.l2_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['l2_trigger'], state.l2_value])
            print(f"  ❄️  L2 frozen → {state.l2_value}")

        if state.r2_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['r2_trigger'], state.r2_value])
            print(f"  ❄️  R2 frozen → {state.r2_value}")

        if state.left_stick_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_x'], state.left_x_value])
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['left_stick_y'], state.left_y_value])
            print(f"  ❄️  Left stick frozen → X:{state.left_x_value}, Y:{state.left_y_value}")

        if state.right_stick_frozen:
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_x'], state.right_x_value])
            midiout.send_message([controller_obj.get_midi_channel_byte(0xB0), CC_MAP['right_stick_y'], state.right_y_value])
            print(f"  ❄️  Right stick frozen → X:{state.right_x_value}, Y:{state.right_y_value}")
