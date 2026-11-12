"""
What the DualSense buttons do — loads and checks config/controls.py.

Used by the DualSense script (midi/button_router.py) and by scripts/er301_routes_tool.py
(the DualSense Mapper app on the Mac). A missing file means today's default behaviour.
"""
import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CONTROLS_PATH = os.path.join(HERE, "config", "controls.py")

# Button name -> CC_MAP key used when the button is set to 'midi' (and its fallback CC number)
BUTTON_CC = {
    "cross": ("btn_south", 14), "circle": ("btn_east", 15), "triangle": ("btn_north", 22), "square": ("btn_west", 23),
    "l1": ("btn_l1", 24), "r1": ("btn_r1", 25), "l3": ("btn_l3", 20), "r3": ("btn_r3", 21),
    "create": ("btn_create", 26), "options": ("btn_options", 27), "ps": ("btn_ps", 28), "mic": ("btn_mic", 29),
    "touchpad": ("btn_touchpad", 9),
}
BUTTONS = list(BUTTON_CC)

# Action -> (fires on 'press' or 'release', description)
ACTIONS = {
    "none":         ("press",   "nothing"),
    "midi":         ("press",   "send the button's own CC (127 pressed / 0 released)"),
    "loop_record":  ("release", "loop: start / stop recording"),
    "loop_play":    ("release", "loop: start / stop playback"),
    "loop_clear":   ("release", "loop: delete"),
    "motion":       ("press",   "motion control on / off"),
    "block_up":     ("press",   "grain slice: next block of 128"),
    "block_down":   ("press",   "grain slice: previous block of 128"),
    "block_reset":  ("press",   "grain slice: back to block 0"),
    "section_up":   ("press",   "grain slice: next section of 16"),
    "section_down": ("press",   "grain slice: previous section of 16"),
    "strum":        ("press",   "harmonic strum (notes around the D-pad step)"),
    "freeze_left":  ("press",   "freeze L2 / R2 / left stick"),
    "freeze_right": ("press",   "freeze right stick"),
    "quantize":     ("press",   "loop quantize on / off"),
}
DPAD_HORIZONTAL_MODES = ("slice_section", "midi")
LEFT_STICK_X_MODES = ("slice_fine", "midi")

# Axes -> function. Continuous axes give a position 0-127, step axes (D-pad) give -1 / +1.
#   midi           send the axis' own CC (motion: 14-bit NRPN) - route it to the ER-301 in er301_routes.py
#   none           nothing
#   slice_fine     grain slice: slice inside the section (16)        step axes: -
#   slice_section  grain slice: section of 16 inside the block (8)    step axes: previous / next section
#   slice_block    grain slice: block of 128 (8)                      step axes: previous / next block
#   loop_scrub     playing loop: playback position                     (continuous only)
#   quantize_grid  quantize subdivision 1/32 1/16 1/8 1/4              (continuous only)
AXIS_CC = {   # axis -> CC_MAP / NRPN_MAP key of its own MIDI output
    "left_x": "left_stick_x", "left_y": "left_stick_y", "right_x": "right_stick_x", "right_y": "right_stick_y",
    "l2": "l2_trigger", "r2": "r2_trigger", "touch_x": "touchpad_x", "touch_y": "touchpad_y",
    "dpad_x": "dpad_horizontal", "dpad_y": "dpad_vertical", "tilt_x": "tilt_x", "tilt_y": "tilt_y", "twist": "twist",
}
AXES = list(AXIS_CC)
STEP_AXES = ("dpad_x", "dpad_y")
AXIS_FUNCTIONS = {
    "midi":          "send its own MIDI (route it in er301_routes.py)",
    "none":          "nothing",
    "slice_fine":    "grain slice: slice inside the section",
    "slice_section": "grain slice: section of 16",
    "slice_block":   "grain slice: block of 128",
    "loop_scrub":    "loop: playback position",
    "quantize_grid": "loop: quantize subdivision",
}
STEP_FUNCTIONS = ("midi", "none", "slice_section", "slice_block")

# MIDI from the other controllers -> the same functions (Pi tab in Kontrolleur).
#   MIDI_INPUTS = [(device, channel, "cc" | "note", number, function), ...]
# device: a key of INPUT_DEVICES. Their MIDI still reaches the i2c2midi as before; these entries
# only add a Pi function on top. A CC can drive a continuous function (its value 0-127) or an
# action; an action fires when the value goes from below 64 to 64 or above (note on = 127,
# note off = 0), so footswitches, EC4 push buttons and notes work as buttons.
INPUT_DEVICES = {               # device -> ALSA port name contains one of these
    "16n":   ("16n",),
    "pedal": ("DOREMiDi", "CH345"),
    "ec4":   ("EC4",),
}
CONTINUOUS_FUNCTIONS = tuple(f for f in AXIS_FUNCTIONS if f not in ("midi", "none"))
INPUT_ACTIONS = tuple(a for a in ACTIONS if a not in ("midi", "none"))
INPUT_FUNCTIONS = CONTINUOUS_FUNCTIONS + INPUT_ACTIONS

DEFAULTS = {
    "buttons": {
        "cross": {"tap": "midi", "hold": None}, "circle": {"tap": "midi", "hold": None},
        "triangle": {"tap": "midi", "hold": None}, "square": {"tap": "midi", "hold": None},
        "l1": {"tap": "loop_clear", "hold": None},
        "r1": {"tap": "loop_play", "hold": "loop_record"},
        "l3": {"tap": "freeze_left", "hold": None}, "r3": {"tap": "freeze_right", "hold": None},
        "create": {"tap": "block_down", "hold": None}, "options": {"tap": "block_up", "hold": None},
        "ps": {"tap": "strum", "hold": None}, "mic": {"tap": "none", "hold": None},
        "touchpad": {"tap": "quantize", "hold": None},
    },
    "chords": [("l1", "r1", "motion"), ("create", "options", "block_reset")],
    "dpad_horizontal": "slice_section",
    "left_stick_x": "slice_fine",
    "axes": {"left_x": "slice_fine", "left_y": "midi", "right_x": "midi", "right_y": "midi", "l2": "midi", "r2": "midi",
             "touch_x": "loop_scrub", "touch_y": "quantize_grid", "dpad_x": "slice_section", "dpad_y": "midi",
             "tilt_x": "midi", "tilt_y": "midi", "twist": "midi"},
    "midi_inputs": [],
}


class ControlsError(Exception):
    pass


def _check(cond, msg):
    if not cond:
        raise ControlsError(msg)


def _action(value, where, allow_midi=True):
    if value is None:
        return None
    _check(isinstance(value, str) and value in ACTIONS,
           f"{where}: unknown action {value!r} (known: {', '.join(ACTIONS)})")
    _check(allow_midi or value != "midi", f"{where}: 'midi' only works as tap")
    return value


def _midi_inputs(raw):
    _check(isinstance(raw, (list, tuple)), "MIDI_INPUTS must be a list")
    out, seen = [], set()
    for e in raw:
        _check(isinstance(e, (tuple, list)) and len(e) == 5,
               f"MIDI_INPUTS: {e!r} must be (device, channel, 'cc' | 'note', number, function)")
        dev, ch, kind, num, fn = e
        _check(dev in INPUT_DEVICES, f"MIDI_INPUTS {e!r}: unknown device {dev!r} (known: {', '.join(INPUT_DEVICES)})")
        _check(isinstance(ch, int) and 1 <= ch <= 16, f"MIDI_INPUTS {e!r}: channel must be 1-16")
        _check(kind in ("cc", "note"), f"MIDI_INPUTS {e!r}: kind must be 'cc' or 'note'")
        _check(isinstance(num, int) and 0 <= num <= 127, f"MIDI_INPUTS {e!r}: number must be 0-127")
        _check(fn in INPUT_FUNCTIONS, f"MIDI_INPUTS {e!r}: unknown function {fn!r} (known: {', '.join(INPUT_FUNCTIONS)})")
        _check(kind == "cc" or fn in INPUT_ACTIONS, f"MIDI_INPUTS {e!r}: a note can only trigger an action")
        key = (dev, ch, kind, num)
        _check(key not in seen, f"MIDI_INPUTS: {dev} ch {ch} {kind} {num} is used twice")
        seen.add(key)
        out.append((dev, ch, kind, num, fn))
    return out


def load(path=CONTROLS_PATH):
    """Return the checked controls as a dict (see DEFAULTS). Raises ControlsError."""
    if not os.path.exists(path):
        d = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULTS.items()}
        d["buttons"] = {n: dict(v) for n, v in DEFAULTS["buttons"].items()}
        d["midi_inputs"] = []
        return d
    try:
        spec = importlib.util.spec_from_file_location("controls_live", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    except Exception as e:
        raise ControlsError(f"{os.path.basename(path)}: {type(e).__name__}: {e}")

    buttons = {name: dict(cfg) for name, cfg in DEFAULTS["buttons"].items()}
    raw = getattr(mod, "BUTTONS", {}) or {}
    _check(isinstance(raw, dict), "BUTTONS must be a dict")
    for name, cfg in raw.items():
        _check(name in BUTTON_CC, f"BUTTONS: unknown button {name!r} (known: {', '.join(BUTTONS)})")
        _check(isinstance(cfg, dict), f"BUTTONS[{name!r}] must be dict(tap=..., hold=...)")
        extra = set(cfg) - {"tap", "hold"}
        _check(not extra, f"BUTTONS[{name!r}]: unknown key(s) {', '.join(sorted(extra))}")
        tap = _action(cfg.get("tap", "none"), f"BUTTONS[{name!r}] tap") or "none"
        hold = _action(cfg.get("hold"), f"BUTTONS[{name!r}] hold", allow_midi=False)
        buttons[name] = {"tap": tap, "hold": None if hold == "none" else hold}

    chords = []
    for c in getattr(mod, "CHORDS", DEFAULTS["chords"]) or []:
        _check(isinstance(c, (tuple, list)) and len(c) == 3, f"CHORDS: {c!r} must be (button, button, action)")
        a, b, act = c
        _check(a in BUTTON_CC and b in BUTTON_CC, f"CHORDS: {c!r} uses an unknown button")
        _check(a != b, f"CHORDS: {c!r} needs two different buttons")
        _action(act, f"CHORDS {a}+{b}", allow_midi=False)
        chords.append((a, b, act))

    dh = getattr(mod, "DPAD_HORIZONTAL", DEFAULTS["dpad_horizontal"])
    _check(dh in DPAD_HORIZONTAL_MODES, f"DPAD_HORIZONTAL must be one of {DPAD_HORIZONTAL_MODES}")
    lx = getattr(mod, "LEFT_STICK_X", DEFAULTS["left_stick_x"])
    _check(lx in LEFT_STICK_X_MODES, f"LEFT_STICK_X must be one of {LEFT_STICK_X_MODES}")

    # AXES (new); files without it: DPAD_HORIZONTAL / LEFT_STICK_X as before
    axes = dict(DEFAULTS["axes"])
    axes["dpad_x"], axes["left_x"] = dh, lx
    raw_axes = getattr(mod, "AXES", None)
    if raw_axes is not None:
        _check(isinstance(raw_axes, dict), "AXES must be a dict")
        for name, fn in raw_axes.items():
            _check(name in AXIS_CC, f"AXES: unknown axis {name!r} (known: {', '.join(AXES)})")
            _check(fn in AXIS_FUNCTIONS, f"AXES[{name!r}]: unknown function {fn!r} (known: {', '.join(AXIS_FUNCTIONS)})")
            _check(name not in STEP_AXES or fn in STEP_FUNCTIONS,
                   f"AXES[{name!r}]: the D-pad can do {', '.join(STEP_FUNCTIONS)}")
            axes[name] = fn
    dh = "slice_section" if axes["dpad_x"] == "slice_section" else "midi"
    lx = "slice_fine" if axes["left_x"] == "slice_fine" else "midi"
    midi_inputs = _midi_inputs(getattr(mod, "MIDI_INPUTS", []) or [])
    return {"buttons": buttons, "chords": chords, "dpad_horizontal": dh, "left_stick_x": lx, "axes": axes,
            "midi_inputs": midi_inputs}


def mtime(path=CONTROLS_PATH):
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None
