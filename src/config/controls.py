"""
What the DualSense buttons do. Read by the DualSense script; changes apply within a second.
Written by Kontrolleur on the Mac. You can still edit it by hand; the app reads it back.

tap  = short press   (loop actions fire when you let go)
hold = held for LONG_PRESS_DURATION (mappings.py) or longer, decided when you let go
'midi' = the button sends its own CC (127 pressed / 0 released): route it to the ER-301
         in er301_routes.py. Buttons: cross circle triangle square l1 r1 l3 r3 create
         options ps mic touchpad (= touchpad click).
Actions: none midi loop_record loop_play loop_clear motion block_up block_down block_reset
         section_up section_down strum freeze_left freeze_right quantize
"""

BUTTONS = {
    'cross':    dict(tap='midi'),
    'circle':   dict(tap='midi'),
    'triangle': dict(tap='midi'),
    'square':   dict(tap='midi'),
    'l1':       dict(tap='loop_clear'),
    'r1':       dict(tap='loop_play', hold='loop_record'),
    'l3':       dict(tap='freeze_left'),
    'r3':       dict(tap='freeze_right'),
    'create':   dict(tap='block_down'),
    'options':  dict(tap='block_up'),
    'ps':       dict(tap='strum'),
    'mic':      dict(tap='none'),
    'touchpad': dict(tap='quantize'),
}

# Two buttons pressed together. The buttons' own release actions are skipped then.
CHORDS = [
    ('l1', 'r1', 'motion'),
    ('create', 'options', 'block_reset'),
]

DPAD_HORIZONTAL = 'slice_section'   # 'slice_section' (16-slice sections) or 'midi' (CC steps)
LEFT_STICK_X = 'slice_fine'         # 'slice_fine' (slice within the section) or 'midi' (CC)

# MIDI from the other controllers -> the same functions: (device, channel, 'cc' | 'note', number, function).
# device: '16n', 'pedal' (MPC-20 via DOREMiDi) or 'ec4'; channel as the device sends it. A CC drives a
# continuous function with its value, or fires an action when it goes to 64 or above (notes: note on).
# Example: ('16n', 16, 'cc', 82, 'slice_section'), ('pedal', 2, 'cc', 109, 'loop_record')
MIDI_INPUTS = [
]
