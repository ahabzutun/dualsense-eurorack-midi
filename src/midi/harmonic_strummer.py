"""
Harmonic Strummer for DualSense Controller
Generates random chord progressions mixing consonant and dissonant intervals
Outputs fast arpeggiated CC11 values on CC14 triggers
"""

import random
import time
import threading
from typing import List


class HarmonicStrummer:
    """
    Generates random chord progressions with consonant and dissonant harmonics.
    Arpeggiates chords as CC11 values with CC14 gate triggers.
    """

    def __init__(self,
                 strum_delay_ms=80,  # Fast strumming
                 notes_per_chord=4):

        self.strum_delay = strum_delay_ms / 1000.0  # Convert to seconds
        self.notes_per_chord = notes_per_chord
        self.strum_thread = None

        # Interval pools in CC value offsets (for audible differences)
        # Consonant: stable harmonic intervals (larger spacing)
        self.consonant_intervals = [0, 5, 7, 10, 12, 15]  # Root, m3, P4, P5, M6

        # Dissonant: tense intervals (odd spacing for "off" sound)
        self.dissonant_intervals = [2, 3, 13, 17, 18]  # m2, M2, m7, tritone-ish

    def generate_chord(self, center_cc: int) -> List[int]:
        """
        Generate random chord mixing consonant and dissonant intervals.
        60% consonant, 40% dissonant for experimental flavor.

        Args:
            center_cc: Current CC11 value from D-pad (0-127)

        Returns:
            List of CC values spread around center
        """
        cc_values = []

        # Generate intervals around center
        for _ in range(self.notes_per_chord):
            if random.random() < 0.6:  # 60% consonant
                offset = random.choice(self.consonant_intervals)
            else:  # 40% dissonant
                offset = random.choice(self.dissonant_intervals)

            # Randomly go above or below center
            if random.random() < 0.5:
                offset = -offset

            cc_value = center_cc + offset

            # Clamp to MIDI CC range
            cc_value = max(0, min(127, cc_value))
            cc_values.append(cc_value)

        # Sort for arpeggiation and remove duplicates
        cc_values = sorted(list(set(cc_values)))

        # If we lost notes due to duplicates/clamping, ensure at least 2 notes
        while len(cc_values) < 2:
            # Add a note between existing ones
            offset = random.choice([8, 12, 15, 20])
            new_cc = center_cc + offset
            new_cc = max(0, min(127, new_cc))
            if new_cc not in cc_values:
                cc_values.append(new_cc)
                cc_values.sort()

        return cc_values

    def arpeggiate(self, midi_out, midi_channel=1, center_cc=64, loop_state=None):
            """
            Send arpeggiated chord as CC14 triggers + CC11 pitch values.
            Non-blocking via threading.

            Args:
                midi_out: rtmidi MidiOut object
                midi_channel: MIDI channel (1-16)
                center_cc: Current D-pad CC11 value (pitch center)
                loop_state: Optional LoopState for recording
            """
            # Don't start new strum if one is already playing
            if self.strum_thread and self.strum_thread.is_alive():
                print("⚠️  Strum already in progress, skipping")
                return

            # Generate new chord around current pitch center
            chord = self.generate_chord(center_cc)

            # Random direction (50% up, 50% down)
            if random.random() < 0.5:
                chord.reverse()  # Strum down
                direction = "↓"
            else:
                direction = "↑"

            print(f"🎸 Strum {direction} (center={center_cc}): {chord}")

            # Launch in separate thread
            self.strum_thread = threading.Thread(
                target=self._send_strum,
                args=(midi_out, chord, midi_channel, loop_state)
            )
            self.strum_thread.daemon = True
            self.strum_thread.start()

    def _send_strum(self, midi_out, chord: List[int], channel: int, loop_state=None):
            """
            Internal: sends the actual MIDI messages.

            Sends CC11 pitch FIRST, then CC14 gate (with settling time).
            """
            status_byte = 0xB0 + (channel - 1)  # CC message

            for i, cc_pitch in enumerate(chord):
                # 1. CC11 = pitch value (SEND FIRST!)
                cc11_msg = [status_byte, 11, cc_pitch]
                midi_out.send_message(cc11_msg)

                # Record to loop if recording
                if loop_state and loop_state.recording:
                    loop_state.record_message(cc11_msg)

                # 2. SETTLE TIME - let the pitch CV stabilize (10ms)
                time.sleep(0.01)

                # 3. CC14 ON (trigger - now the pitch is ready!)
                cc14_on = [status_byte, 14, 127]
                midi_out.send_message(cc14_on)

                # Record to loop if recording
                if loop_state and loop_state.recording:
                    loop_state.record_message(cc14_on)

                # 4. Wait for strum delay
                time.sleep(self.strum_delay)

                # 5. CC14 OFF (trigger complete)
                cc14_off = [status_byte, 14, 0]
                midi_out.send_message(cc14_off)

                # Record to loop if recording
                if loop_state and loop_state.recording:
                    loop_state.record_message(cc14_off)
