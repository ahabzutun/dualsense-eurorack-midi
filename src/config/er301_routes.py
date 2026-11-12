"""
ER-301 routing inside the i2c2midi (DualSense → ER-301 directly, no Teletype script).

Written by DualSense Mapper on the Mac. You can still edit it by hand; the app reads it back.
Saving makes the MIDI hub (midi-passthrough service) send it to the i2c2midi within a
second; the i2c2midi keeps it across power cycles. Check by hand:

    python3 ~/dualsense-eurorack-midi/scripts/er301_push.py

Ports are the ER-301 SC.CV / SC.TR numbers (1–100) chosen as the source of a unit's input.
Entries are (control, port) on CHANNEL, or (channel, control, port) for another channel.
"""

CHANNEL = 2            # DualSense instrument channel

# CC → SC.CV   (0–127 → 0–10 V)
CV = [
    ('l2_trigger', 8),            # L2 → SC.CV 8
    ('right_stick_x', 4),         # Right stick x axis → SC.CV 4
    ('right_stick_y', 9),         # Right stick y axis → SC.CV 9
    (16, 80, 4),                  # ch 16 CC 80 → SC.CV 4
    (16, 81, 5),                  # ch 16 CC 81 → SC.CV 5
    (5, 5, 4),                    # ch 5 CC 5 → SC.CV 4
    (2, 105, 10),                 # ch 2 CC 105 → SC.CV 10
    (2, 109, 13),                 # ch 2 CC 109 → SC.CV 13
]

# 14-bit NRPN → SC.CV   (0–16383 → 0–10 V, full resolution)
NRPN_CV = [
    ('grain_slice', 3),           # Grain slice → SC.CV 3
]

# CC as gate (≥ 64 = high) → trigger port
TRIGGERS = [
    ('btn_south', 2),             # ✕ Cross → SC.CV 2
]

# Note held → trigger port. These notes do not change the pitch.
NOTE_TRIGGERS = [
    (2, 72, 12),                  # ch 2 note 72 → SC.CV 12
]

# Notes on CHANNEL → V/Oct. The octave control adds steps of semitones_per_step.
PITCH = dict(port=5, zero_note=60, octave_control='dpad_vertical', semitones_per_step=12)

# Trigger LFO: rate control 0 = stopped, 1–127 = min_hz … max_hz (exponential).
LFO = dict(port=2, rate_control='r2_trigger', reset_control='btn_west',
           min_hz=0.25, max_hz=40, pulse_ms=5)

# True: trigger ports are SC.CV ports (10 V pulse / gate). False: SC.TR ports.
TRIGGERS_AS_CV = True
