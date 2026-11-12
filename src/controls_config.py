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


def load(path=CONTROLS_PATH):
    """Return the checked controls as a dict (see DEFAULTS). Raises ControlsError."""
    if not os.path.exists(path):
        return {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULTS.items()}
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
    return {"buttons": buttons, "chords": chords, "dpad_horizontal": dh, "left_stick_x": lx}


def mtime(path=CONTROLS_PATH):
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None
