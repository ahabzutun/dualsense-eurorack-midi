"""
DualSense routing and button functions: what the app edits, and the two files it writes on the
SSH host (src/config/er301_routes.py and src/config/controls.py). Port of the Mac app's Model.swift;
the generated files are byte-identical to the Mac app's.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ── ER-301 routing roles (what the i2c2midi does with a control's MIDI) ─────────────────────────

OFF, CV, TRIGGER, NOTE_TRIGGER, LFO_RATE, LFO_RESET, OCTAVE = (
    "off", "cv", "trigger", "noteTrigger", "lfoRate", "lfoReset", "octave")
ROLES = [OFF, CV, TRIGGER, NOTE_TRIGGER, LFO_RATE, LFO_RESET, OCTAVE]
ROLE_TITLE = {OFF: "Off", CV: "CV", TRIGGER: "Trigger", NOTE_TRIGGER: "Trigger",
              LFO_RATE: "LFO rate", LFO_RESET: "LFO reset", OCTAVE: "Octave"}


def role_uses_port(r: str) -> bool:
    return r in (CV, TRIGGER, NOTE_TRIGGER)


def role_exclusive(r: str) -> bool:
    return r in (LFO_RATE, LFO_RESET, OCTAVE)


# ── Button functions (config/controls.py) ───────────────────────────────────────────────────────

ACTIONS = ["none", "midi", "loop_record", "loop_play", "loop_clear", "motion", "block_up", "block_down",
           "block_reset", "section_up", "section_down", "strum", "freeze_left", "freeze_right", "quantize"]
ACTION_TITLE = {
    "none": "Nothing", "midi": "MIDI → ER-301", "loop_record": "Loop: record start / stop",
    "loop_play": "Loop: play / stop", "loop_clear": "Loop: delete", "motion": "Motion control on / off",
    "block_up": "Slice block +128", "block_down": "Slice block −128", "block_reset": "Slice block → 0",
    "section_up": "Slice section +16", "section_down": "Slice section −16", "strum": "Harmonic strum",
    "freeze_left": "Freeze L2 / R2 / left stick", "freeze_right": "Freeze right stick",
    "quantize": "Loop quantize on / off",
}
ACTION_TOKEN = {
    "none": "—", "midi": "MIDI", "loop_record": "REC", "loop_play": "PLAY", "loop_clear": "DELETE",
    "motion": "MOTION", "block_up": "BLOCK +", "block_down": "BLOCK −", "block_reset": "BLOCK 0",
    "section_up": "SECTION +", "section_down": "SECTION −", "strum": "STRUM", "freeze_left": "FREEZE L",
    "freeze_right": "FREEZE R", "quantize": "QUANTIZE",
}
HOLD_CHOICES = [a for a in ACTIONS if a != "midi"]
CHORD_CHOICES = [a for a in ACTIONS if a not in ("midi", "none")]


# Axes (config/controls.py AXES): what each analog / step axis does
AXIS_OF = {   # input id -> AXES name on the SSH host
    "left_stick_x": "left_x", "left_stick_y": "left_y", "right_stick_x": "right_x", "right_stick_y": "right_y",
    "l2_trigger": "l2", "r2_trigger": "r2", "touchpad_x": "touch_x", "touchpad_y": "touch_y",
    "dpad_horizontal": "dpad_x", "dpad_vertical": "dpad_y", "nrpn_tilt_x": "tilt_x", "nrpn_tilt_y": "tilt_y",
    "nrpn_twist": "twist",
}
AXES = list(AXIS_OF.values())
STEP_AXES = ("dpad_x", "dpad_y")
DEFAULT_AXES = {"left_x": "slice_fine", "left_y": "midi", "right_x": "midi", "right_y": "midi", "l2": "midi", "r2": "midi",
                "touch_x": "loop_scrub", "touch_y": "quantize_grid", "dpad_x": "slice_section", "dpad_y": "midi",
                "tilt_x": "midi", "tilt_y": "midi", "twist": "midi"}
AXIS_FUNCS = ["midi", "none", "slice_fine", "slice_section", "slice_block", "loop_scrub", "quantize_grid"]
STEP_FUNCS = ["midi", "none", "slice_section", "slice_block"]
AXIS_TOKEN = {"slice_fine": "SLICE", "slice_section": "SECTION", "slice_block": "BLOCK", "loop_scrub": "SCRUB",
              "quantize_grid": "Q-GRID"}

# One list of functions per control (what the inspector offers). 'er_*' = MIDI routed to the ER-301.
FUNC_TITLE = {
    "none": "Nothing", "er_cv": "ER-301 CV", "er_trigger": "ER-301 trigger", "er_lfo_rate": "LFO rate",
    "er_lfo_reset": "LFO reset", "er_octave": "Octave (pitch CV)", "midi": "MIDI only (not to the ER-301)",
    "slice_fine": "Grain slice: slice in section", "slice_section": "Grain slice: section", "slice_block": "Grain slice: block",
    "loop_scrub": "Loop: playback position", "quantize_grid": "Loop: quantize grid",
}
ER_ROLE = {"er_cv": CV, "er_trigger": TRIGGER, "er_lfo_rate": LFO_RATE, "er_lfo_reset": LFO_RESET, "er_octave": OCTAVE,
           "midi": OFF}
ROLE_ER = {v: k for k, v in ER_ROLE.items()}


# ── MIDI from the other controllers → Pi functions (config/controls.py MIDI_INPUTS) ──────────────

PI_DEVICES = [("16n", "16n"), ("pedal", "Pedals"), ("ec4", "EC4")]
PI_DEVICE_TITLE = dict(PI_DEVICES)
CONT_FUNCS = ["slice_fine", "slice_section", "slice_block", "loop_scrub", "quantize_grid"]
INPUT_ACTIONS = [a for a in ACTIONS if a not in ("none", "midi")]
LFO_FUNCS = ["er_lfo_rate", "er_lfo_reset"]


@dataclass
class MidiInput:
    device: str          # '16n' | 'pedal' | 'ec4'
    channel: int         # as the device sends it (before the MIDI hub)
    kind: str            # 'cc' | 'note'
    number: int
    function: str        # CONT_FUNCS or INPUT_ACTIONS
    id: str = field(default_factory=lambda: uuid.uuid4().hex)

    @property
    def key(self) -> tuple:
        return (self.device, self.channel, self.kind, self.number)


def _src_text(src) -> str:
    return f"{src[0]}:{src[1]}"


def _src_parse(t, ch: int, num: int) -> Optional[tuple]:
    """'16n:16' → ('16n', 16, num); unknown / missing: a 16n on that channel."""
    try:
        dev, c = str(t).split(":")
        if dev in PI_DEVICE_TITLE:
            return (dev, int(c), num)
    except (ValueError, TypeError):
        pass
    return ("16n", ch, num)


def fires_on_release(a: str) -> bool:
    return a in ("loop_record", "loop_play", "loop_clear")


@dataclass
class ButtonFunc:
    tap: str
    hold: str = "none"


@dataclass
class Chord:
    a: str
    b: str
    action: str
    id: str = field(default_factory=lambda: uuid.uuid4().hex)


BUTTONS: List[Tuple[str, str]] = [
    ("cross", "✕"), ("circle", "○"), ("triangle", "△"), ("square", "□"),
    ("l1", "L1"), ("r1", "R1"), ("l3", "L3"), ("r3", "R3"),
    ("create", "Create"), ("options", "Options"), ("ps", "PS"), ("mic", "Mic"), ("touchpad", "Touchpad click"),
]


def button_label(n: str) -> str:
    for name, label in BUTTONS:
        if name == n:
            return label
    return n


def default_funcs() -> Dict[str, ButtonFunc]:
    return {
        "cross": ButtonFunc("midi"), "circle": ButtonFunc("midi"), "triangle": ButtonFunc("midi"),
        "square": ButtonFunc("midi"), "l1": ButtonFunc("loop_clear"), "r1": ButtonFunc("loop_play", "loop_record"),
        "l3": ButtonFunc("freeze_left"), "r3": ButtonFunc("freeze_right"), "create": ButtonFunc("block_down"),
        "options": ButtonFunc("block_up"), "ps": ButtonFunc("strum"), "mic": ButtonFunc("none"),
        "touchpad": ButtonFunc("quantize"),
    }


def default_chords() -> List[Chord]:
    return [Chord("l1", "r1", "motion"), Chord("create", "options", "block_reset")]


# ── The controller ──────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class InputDef:
    id: str              # CC_MAP / NRPN_MAP name (nrpn_ prefix for NRPNs)
    label: str
    kind: str            # cc nrpn note
    name: Optional[str]  # key in CC_MAP / NRPN_MAP on the SSH host
    fallback: int        # number if the name is unknown there
    roles: Tuple[str, ...]
    button: Optional[str] = None   # controls.py button name for digital buttons

    @property
    def short(self) -> str:
        if self.id in ("left_stick_x", "right_stick_x", "touchpad_x"):
            return "X"
        if self.id in ("left_stick_y", "right_stick_y", "touchpad_y"):
            return "Y"
        if self.id in ("btn_touchpad", "btn_l3", "btn_r3"):
            return "CLICK"
        if self.id == "dpad_vertical":
            return "↕"
        if self.id == "dpad_horizontal":
            return "↔"
        return self.label.upper()


@dataclass(frozen=True)
class GroupDef:
    id: str
    title: str
    hint: str
    inputs: Tuple[InputDef, ...]


AXIS = (OFF, CV, TRIGGER, LFO_RATE)
STEP = (OFF, CV, OCTAVE)
BUTTON = (OFF, TRIGGER, CV, LFO_RESET)


def _cc(name, n, label, roles):
    return InputDef(name, label, "cc", name, n, tuple(roles))


def _btn(name, n, button, label="Button"):
    return InputDef(name, label, "cc", name, n, BUTTON, button)


def _nrpn(name, n, label):
    return InputDef("nrpn_" + name, label, "nrpn", name, n, (OFF, CV))


BUTTON_HINT = "Pick what a tap does, and optionally something else when it's held."
STICK_HINT = "Springs back to the centre (64 ≈ 5 V) when released."

GROUPS: List[GroupDef] = [
    GroupDef("l2", "L2", "Analog trigger, 0–127. As LFO rate: released = stopped, fully pressed = fastest.",
             (_cc("l2_trigger", 7, "Pressure", AXIS),)),
    GroupDef("l1", "L1", BUTTON_HINT, (_btn("btn_l1", 24, "l1"),)),
    GroupDef("r2", "R2", "Analog trigger, 0–127. As LFO rate: released = stopped, fully pressed = fastest.",
             (_cc("r2_trigger", 10, "Pressure", AXIS),)),
    GroupDef("r1", "R1", BUTTON_HINT, (_btn("btn_r1", 25, "r1"),)),
    GroupDef("dpad", "D-pad", "Steps up / down. As MIDI each direction pair sends 8 steps; grain slice functions step one section / block.",
             (_cc("dpad_vertical", 11, "Up / down", STEP),
              _cc("dpad_horizontal", 13, "Left / right", STEP))),
    GroupDef("leftStick", "Left stick", STICK_HINT,
             (_cc("left_stick_x", 1, "X axis", AXIS), _cc("left_stick_y", 2, "Y axis", AXIS),
              _btn("btn_l3", 20, "l3", "Click (L3)"))),
    GroupDef("rightStick", "Right stick", STICK_HINT,
             (_cc("right_stick_x", 87, "X axis", AXIS), _cc("right_stick_y", 88, "Y axis", AXIS),
              _btn("btn_r3", 21, "r3", "Click (R3)"))),
    GroupDef("touchpad", "Touchpad", "Works while a finger is on it; keeps its value when you lift the finger.",
             (_cc("touchpad_x", 30, "X position", AXIS), _cc("touchpad_y", 31, "Y position", AXIS),
              _btn("btn_touchpad", 9, "touchpad", "Click"))),
    GroupDef("create", "Create", BUTTON_HINT, (_btn("btn_create", 26, "create"),)),
    GroupDef("options", "Options", BUTTON_HINT, (_btn("btn_options", 27, "options"),)),
    GroupDef("ps", "PS button", BUTTON_HINT, (_btn("btn_ps", 28, "ps"),)),
    GroupDef("mic", "Mic button", BUTTON_HINT + " (Read from the controller's HID report.)", (_btn("btn_mic", 29, "mic"),)),
    GroupDef("triangle", "△ Triangle", BUTTON_HINT, (_btn("btn_north", 22, "triangle"),)),
    GroupDef("circle", "○ Circle", BUTTON_HINT, (_btn("btn_east", 15, "circle"),)),
    GroupDef("cross", "✕ Cross", BUTTON_HINT, (_btn("btn_south", 14, "cross"),)),
    GroupDef("square", "□ Square", BUTTON_HINT, (_btn("btn_west", 23, "square"),)),
    GroupDef("motion", "Motion", "Tilt and twist, active while motion control is on (give “Motion on / off” to a button or combo). As MIDI they are 14-bit.",
             (_nrpn("tilt_x", 0, "Tilt X"), _nrpn("tilt_y", 1, "Tilt Y"), _nrpn("twist", 2, "Twist"))),
    GroupDef("slice", "Grain slice", "Slice 0–1023 = block × 128 + section × 16 + slice, sent as one 14-bit CV. Give the grain slice functions to any controls, then pick its ER-301 port here.",
             (_nrpn("grain_slice", 3, "Slice 0–1023"),)),
]
GROUP = {g.id: g for g in GROUPS}
ALL_INPUTS: List[InputDef] = [i for g in GROUPS for i in g.inputs]
INPUT = {i.id: i for i in ALL_INPUTS}
_GROUP_OF = {i.id: g for g in GROUPS for i in g.inputs}


def group_of(i: InputDef) -> GroupDef:
    return _GROUP_OF[i.id]


def input_for_button(b: str) -> Optional[InputDef]:
    for i in ALL_INPUTS:
        if i.button == b:
            return i
    return None


def input_title(i: InputDef) -> str:
    """'Left stick x axis' / 'L2' (for lists and file comments)."""
    g = group_of(i)
    return f"{g.title} {i.label.lower()}" if len(g.inputs) > 1 else g.title


# ── Routes from other MIDI sources (pedals, EC4, 16n …) ─────────────────────────────────────────

EXT_CV, EXT_NRPN, EXT_TRIGGER, EXT_NOTE = "CC → CV", "NRPN → CV", "CC → Trigger", "Note → Trigger"
EXT_KINDS = [EXT_CV, EXT_NRPN, EXT_TRIGGER, EXT_NOTE]


@dataclass
class ExternalRoute:
    kind: str = EXT_CV
    channel: int = 2
    control: str = "1"       # number, or a name from CC_MAP / NRPN_MAP
    port: int = 1
    id: str = field(default_factory=lambda: uuid.uuid4().hex)


@dataclass
class PortUse:
    kind: str           # CV / TR
    port: int
    users: List[str] = field(default_factory=list)
    refs: List[tuple] = field(default_factory=list)   # ('dualsense', label, inputID, trigger) | ('builtin', name) | ('external', kind, ch, control)

    @property
    def id(self) -> str:
        return f"{self.kind}{self.port}"


def _strip(s: str) -> str:
    return s.strip(" \t")


def _int_or_none(s: str) -> Optional[int]:
    try:
        return int(_strip(s))
    except (TypeError, ValueError):
        return None


class Mapping:
    def __init__(self):
        self.cc_names: Dict[str, int] = {}
        self.nrpn_names: Dict[str, int] = {}
        self.long_press = 1.0
        self.controls_on_pi = True     # False: the SSH host has no controls support yet (routing only)
        self.load_defaults()

    def load_defaults(self):
        self.roles: Dict[str, str] = {"l2_trigger": CV, "nrpn_grain_slice": CV, "btn_south": TRIGGER,
                                      "dpad_vertical": OCTAVE, "r2_trigger": LFO_RATE, "btn_west": LFO_RESET}
        self.ports: Dict[str, int] = {"l2_trigger": 8, "nrpn_grain_slice": 3, "btn_south": 2}
        self.pitch_enabled = True
        self.pitch_port = 5
        self.zero_note = 60
        self.semis_per_step = 12
        self.lfo_port = 2
        self.lfo_min_hz = 0.25
        self.lfo_max_hz = 40.0
        self.lfo_pulse_ms = 5
        self.triggers_as_cv = True
        self.channel = 2
        self.externals: List[ExternalRoute] = []
        self.funcs: Dict[str, ButtonFunc] = default_funcs()
        self.chords: List[Chord] = default_chords()
        self.axes: Dict[str, str] = dict(DEFAULT_AXES)
        self.midi_inputs: List[MidiInput] = []
        self.lfo_rate_src: Optional[tuple] = None     # (device, channel as sent, cc) when the rate comes from another device
        self.lfo_reset_src: Optional[tuple] = None

    # ── other controllers → Pi functions ──
    def hub_ch(self, device: str, ch: int) -> int:
        """The channel the i2c2midi sees: the MIDI hub moves pedal channel 1 to the DualSense channel."""
        return self.channel if device == "pedal" and ch == 1 else ch

    def pi_choices(self, kind: str) -> List[str]:
        if kind == "note":
            return ["none"] + INPUT_ACTIONS
        return ["none"] + CONT_FUNCS + INPUT_ACTIONS + LFO_FUNCS

    def pi_fn(self, device: str, ch: int, kind: str, num: int) -> str:
        key = (device, ch, kind, num)
        for x in self.midi_inputs:
            if x.key == key:
                return x.function
        if kind == "cc":
            if self.lfo_rate_src == (device, ch, num):
                return "er_lfo_rate"
            if self.lfo_reset_src == (device, ch, num):
                return "er_lfo_reset"
        return "none"

    def set_pi_fn(self, device: str, ch: int, kind: str, num: int, f: str):
        key = (device, ch, kind, num)
        self.midi_inputs = [x for x in self.midi_inputs if x.key != key]
        if self.lfo_rate_src == (device, ch, num):
            self.lfo_rate_src = None
        if self.lfo_reset_src == (device, ch, num):
            self.lfo_reset_src = None
        if f in LFO_FUNCS and kind == "cc":
            role = LFO_RATE if f == "er_lfo_rate" else LFO_RESET
            cur = self.input_with(role)
            if cur is not None:
                self.set_choice(cur, "midi")
            if f == "er_lfo_rate":
                self.lfo_rate_src = (device, ch, num)
            else:
                self.lfo_reset_src = (device, ch, num)
        elif f in CONT_FUNCS + INPUT_ACTIONS and (kind == "cc" or f in INPUT_ACTIONS):
            self.midi_inputs.append(MidiInput(device, ch, kind, num, f))

    def move_pi(self, device: str, kind: str, old: Tuple[int, int], new: Tuple[int, int]):
        """A device control changed channel / number: its Pi function goes along."""
        if old == new:
            return
        f = self.pi_fn(device, old[0], kind, old[1])
        if f == "none" or self.pi_fn(device, new[0], kind, new[1]) != "none":
            return
        self.set_pi_fn(device, old[0], kind, old[1], "none")
        self.set_pi_fn(device, new[0], kind, new[1], f)

    def drop_pi(self, device: str, ch: int, kind: str, num: int):
        self.set_pi_fn(device, ch, kind, num, "none")

    # ── trigger LFO ──
    @property
    def lfo_on(self) -> bool:
        return self.lfo_rate_src is not None or self.input_with(LFO_RATE) is not None

    @property
    def lfo_channel(self) -> int:
        s = self.lfo_rate_src
        return self.hub_ch(s[0], s[1]) if s else self.channel

    def lfo_problem(self) -> Optional[str]:
        reset = self.lfo_reset_src
        if reset is None and self.input_with(LFO_RESET) is not None and self.lfo_channel != self.channel:
            return (f"LFO: the phase reset is on the DualSense (channel {self.channel}), the rate on channel {self.lfo_channel}. "
                    "The i2c2midi listens to both on one channel.")
        if reset is not None and self.lfo_on and self.hub_ch(reset[0], reset[1]) != self.lfo_channel:
            return (f"LFO: the phase reset comes on channel {self.hub_ch(reset[0], reset[1])}, the rate on channel "
                    f"{self.lfo_channel}. The i2c2midi listens to both on one channel.")
        return None

    # ── functions ──
    def fn(self, button: str) -> ButtonFunc:
        f = self.funcs.get(button)
        return ButtonFunc(f.tap, f.hold) if f else ButtonFunc("none", "none")

    def set_tap(self, a: str, button: str):
        f = self.fn(button)
        f.tap = a
        self.funcs[button] = f

    def set_hold(self, a: str, button: str):
        f = self.fn(button)
        f.hold = a
        self.funcs[button] = f

    # legacy names (controls.py DPAD_HORIZONTAL / LEFT_STICK_X for older DualSense scripts)
    @property
    def dpad_horizontal(self) -> str:
        return "slice_section" if self.axes.get("dpad_x") == "slice_section" else "midi"

    @property
    def left_stick_x(self) -> str:
        return "slice_fine" if self.axes.get("left_x") == "slice_fine" else "midi"

    def axis_fn(self, i: InputDef) -> Optional[str]:
        a = AXIS_OF.get(i.id)
        return self.axes.get(a, DEFAULT_AXES[a]) if a else None

    def sends_midi(self, i: InputDef) -> bool:
        if i.button:
            return self.fn(i.button).tap == "midi"
        a = AXIS_OF.get(i.id)
        if a:
            return self.axes.get(a, DEFAULT_AXES[a]) == "midi"
        return True

    # ── one function per control ──
    def choices(self, i: InputDef) -> List[str]:
        if i.button:
            return ["none"] + [ROLE_ER[r] for r in i.roles if r != OFF] + ["midi"] + \
                   [a for a in ACTIONS if a not in ("none", "midi")]
        a = AXIS_OF.get(i.id)
        if a is None:                                  # grain slice output
            return ["none", "er_cv"]
        er = [ROLE_ER[r] for r in i.roles if r != OFF]
        funcs = STEP_FUNCS if a in STEP_AXES else AXIS_FUNCS
        return ["none"] + er + ["midi"] + [f for f in funcs if f not in ("midi", "none")]

    def choice(self, i: InputDef) -> str:
        if i.button:
            f = self.fn(i.button)
            return ROLE_ER.get(self.role(i), "midi") if f.tap == "midi" else f.tap
        a = AXIS_OF.get(i.id)
        if a is None:
            return "er_cv" if self.role(i) == CV else "none"
        f = self.axes.get(a, DEFAULT_AXES[a])
        return ROLE_ER.get(self.role(i), "midi") if f == "midi" else f

    def set_choice(self, i: InputDef, key: str):
        a = AXIS_OF.get(i.id)
        if key in ER_ROLE:
            if i.button:
                self.set_tap("midi", i.button)
            elif a:
                self.axes[a] = "midi"
            self.set_role(ER_ROLE[key], i)
            return
        if key == "none" and not i.button and a is None:
            self.roles[i.id] = OFF                     # grain slice output off
            return
        self.roles[i.id] = OFF
        if i.button:
            self.set_tap(key, i.button)
        elif a:
            self.axes[a] = key

    def func_title(self, key: str) -> str:
        return FUNC_TITLE.get(key) or ACTION_TITLE.get(key, key)

    # ── routing access ──
    def role(self, i: InputDef) -> str:
        return self.roles.get(i.id, OFF)

    def effective_role(self, i: InputDef) -> str:
        return self.role(i) if self.sends_midi(i) else OFF

    def port(self, i: InputDef) -> int:
        return self.ports.get(i.id, 1)

    def set_role(self, r: str, i: InputDef):
        if r == LFO_RATE:
            self.lfo_rate_src = None
        elif r == LFO_RESET:
            self.lfo_reset_src = None
        if role_exclusive(r):
            for k, v in list(self.roles.items()):
                if v == r:
                    self.roles[k] = OFF
        self.roles[i.id] = r
        if role_uses_port(r) and i.id not in self.ports:
            self.ports[i.id] = self.first_free_port()

    def number(self, i: InputDef) -> int:
        if i.kind == "cc":
            return self.cc_names.get(i.name or "", i.fallback)
        if i.kind == "nrpn":
            return self.nrpn_names.get(i.name or "", i.fallback)
        return i.fallback

    def input_with(self, role: str) -> Optional[InputDef]:
        for i in ALL_INPUTS:
            if self.effective_role(i) == role:
                return i
        return None

    def first_free_port(self) -> int:
        used = {u.port for u in self.port_map()}
        for p in range(1, 100):
            if p not in used:
                return p
        return 1

    @property
    def trigger_port_name(self) -> str:
        return "SC.CV" if self.triggers_as_cv else "SC.TR"

    def route_token(self, i: InputDef) -> str:
        r = self.effective_role(i)
        if r == OFF:
            return "—"
        if r == CV:
            return f"CV {self.port(i)}"
        if r in (TRIGGER, NOTE_TRIGGER):
            return f"TRIG › CV {self.port(i)}" if self.triggers_as_cv else f"TR {self.port(i)}"
        return {LFO_RATE: "LFO RATE", LFO_RESET: "LFO RESET", OCTAVE: "OCTAVE"}[r]

    def _midi_token(self, i: InputDef) -> Tuple[str, tuple]:
        r = self.effective_role(i)
        if r != OFF:
            return self.route_token(i), ("route", r)
        n = self.number(i)
        return (f"NRPN {n}" if i.kind == "nrpn" else f"CC {n}"), ("midi",)

    def tokens(self, i: InputDef) -> List[Tuple[str, tuple]]:
        """What a control does, for the drawing: [(text, kind)], kind = ('route', role) | ('midi',) | ('function',).
        Empty when nothing is assigned."""
        if i.button:
            f = self.fn(i.button)
            out = []
            if f.tap == "midi":
                out.append(self._midi_token(i))
            elif f.tap != "none":
                out.append((ACTION_TOKEN[f.tap], ("function",)))
            if f.hold != "none":
                out.append(("HOLD " + ACTION_TOKEN[f.hold], ("function",)))
            return out
        a = AXIS_OF.get(i.id)
        if a is None:                                  # grain slice output
            return [self._midi_token(i)] if self.effective_role(i) != OFF else []
        f = self.axes.get(a, DEFAULT_AXES[a])
        if f == "none":
            return []
        if f == "midi":
            return [self._midi_token(i)]
        tok = AXIS_TOKEN[f] + (" ±" if a in STEP_AXES else "")
        return [(tok, ("function",))]

    def shown(self, i: InputDef) -> List[Tuple[str, tuple]]:
        """Tokens worth showing on the drawing: ER-301 routes and functions (not 'MIDI only')."""
        return [t for t in self.tokens(i) if t[1][0] != "midi"]

    def assigned(self, i: InputDef) -> bool:
        return bool(self.shown(i))

    def midi_only(self, i: InputDef) -> bool:
        return not self.assigned(i) and bool(self.tokens(i))

    # ── port overview ──
    def port_map(self) -> List[PortUse]:
        m: Dict[str, PortUse] = {}

        def add(kind, port, who, ref):
            k = f"{kind}{port}"
            if k not in m:
                m[k] = PortUse(kind, port)
            m[k].users.append(who)
            m[k].refs.append(ref)

        tk = "CV" if self.triggers_as_cv else "TR"
        for i in ALL_INPUTS:
            g = group_of(i)
            who = f"{g.title} {i.label}" if len(g.inputs) > 1 else g.title
            r = self.effective_role(i)
            if r == CV:
                add("CV", self.port(i), who, ("dualsense", who, i.id, False))
            elif r in (TRIGGER, NOTE_TRIGGER):
                add(tk, self.port(i), who + " (trigger)", ("dualsense", who, i.id, True))
        if self.pitch_enabled:
            add("CV", self.pitch_port, "Pitch V/Oct", ("builtin", "Pitch V/Oct"))
        if self.lfo_on:
            add(tk, self.lfo_port, "LFO", ("builtin", "LFO"))
        for e in self.externals:
            what = "note" if e.kind == EXT_NOTE else ("NRPN" if e.kind == EXT_NRPN else "CC")
            who = f"ch {e.channel} {what} {e.control}"
            k = "note" if e.kind == EXT_NOTE else "nrpn" if e.kind == EXT_NRPN else "trigger" if e.kind == EXT_TRIGGER else "cc"
            add("CV" if e.kind in (EXT_CV, EXT_NRPN) else tk, e.port, who, ("external", k, e.channel, _strip(e.control)))
        return sorted(m.values(), key=lambda u: (u.kind, u.port))

    # ── load from the SSH host (JSON from scripts/er301_routes_tool.py export) ──
    def apply(self, e: dict):
        if e.get("cc_names") is not None:
            self.cc_names = e["cc_names"]
        if e.get("nrpn_names") is not None:
            self.nrpn_names = e["nrpn_names"]
        if e.get("long_press") is not None:
            self.long_press = float(e["long_press"])
        self.channel = e.get("channel") or 2
        self.roles, self.ports, self.externals = {}, {}, []
        self.lfo_rate_src = self.lfo_reset_src = None

        def resolve(c, kind):
            if isinstance(c, int):
                return c
            return (self.nrpn_names if kind == "nrpn" else self.cc_names).get(c)

        def text(c):
            return str(c)

        def claim(entry, kind, r) -> bool:
            if entry["channel"] != self.channel:
                return False
            num = resolve(entry["control"], kind)
            if num is None:
                return False
            for i in ALL_INPUTS:
                if i.kind == kind and self.number(i) == num and self.role(i) == OFF and r in i.roles:
                    self.roles[i.id] = r
                    if role_uses_port(r):
                        self.ports[i.id] = entry["port"]
                    return True
            return False

        for x in e.get("cv") or []:
            if not claim(x, "cc", CV):
                self.externals.append(ExternalRoute(EXT_CV, x["channel"], text(x["control"]), x["port"]))
        for x in e.get("nrpn_cv") or []:
            if not claim(x, "nrpn", CV):
                self.externals.append(ExternalRoute(EXT_NRPN, x["channel"], text(x["control"]), x["port"]))
        for x in e.get("triggers") or []:
            if not claim(x, "cc", TRIGGER):
                self.externals.append(ExternalRoute(EXT_TRIGGER, x["channel"], text(x["control"]), x["port"]))
        for x in e.get("note_triggers") or []:
            self.externals.append(ExternalRoute(EXT_NOTE, x["channel"], text(x["control"]), x["port"]))
        p = e.get("pitch")
        if p:
            self.pitch_enabled = True
            self.pitch_port = p["port"]
            self.zero_note = p.get("zero_note") if p.get("zero_note") is not None else 60
            self.semis_per_step = p.get("semitones_per_step") if p.get("semitones_per_step") is not None else 12
            if p.get("octave_control") is not None:
                claim({"channel": self.channel, "control": p["octave_control"], "port": 0}, "cc", OCTAVE)
        else:
            self.pitch_enabled = False
        lf = e.get("lfo")
        if lf:
            self.lfo_port = lf["port"]
            self.lfo_min_hz = float(lf["min_hz"]) if lf.get("min_hz") is not None else 0.25
            self.lfo_max_hz = float(lf["max_hz"]) if lf.get("max_hz") is not None else 40.0
            self.lfo_pulse_ms = lf.get("pulse_ms") if lf.get("pulse_ms") is not None else 5
            lch = lf.get("channel") or self.channel
            for k, role, src_key, attr in (("rate_control", LFO_RATE, "source", "lfo_rate_src"),
                                           ("reset_control", LFO_RESET, "reset_source", "lfo_reset_src")):
                c = lf.get(k)
                if c is None:
                    continue
                if lf.get(src_key) is None and lch == self.channel and \
                        claim({"channel": self.channel, "control": c, "port": 0}, "cc", role):
                    continue
                num = resolve(c, "cc")
                if num is not None:
                    setattr(self, attr, _src_parse(lf.get(src_key), lch, num))
        self.triggers_as_cv = e.get("triggers_as_cv", True) if e.get("triggers_as_cv") is not None else True

        c = e.get("controls")
        if c:
            self.controls_on_pi = True
            f = default_funcs()
            for name, v in (c.get("buttons") or {}).items():
                tap = v.get("tap")
                hold = v.get("hold")
                f[name] = ButtonFunc(tap if tap in ACTIONS else "none", hold if hold in ACTIONS else "none")
            self.funcs = f
            self.chords = [Chord(x[0], x[1], x[2]) for x in c.get("chords") or [] if len(x) == 3 and x[2] in ACTIONS]
            axes = dict(DEFAULT_AXES)
            axes["dpad_x"] = c.get("dpad_horizontal", "slice_section")
            axes["left_x"] = c.get("left_stick_x", "slice_fine")
            for k, v in (c.get("axes") or {}).items():
                if k in axes and v in AXIS_FUNCS:
                    axes[k] = v
            self.axes = axes
            self.midi_inputs = []
            for x in c.get("midi_inputs") or []:
                if len(x) == 5 and x[0] in PI_DEVICE_TITLE and x[2] in ("cc", "note") and x[4] in CONT_FUNCS + INPUT_ACTIONS:
                    self.midi_inputs.append(MidiInput(x[0], int(x[1]), x[2], int(x[3]), x[4]))
        else:
            self.controls_on_pi = e.get("controls_error") is not None
            self.midi_inputs = []
            self.funcs, self.chords = default_funcs(), default_chords()
            self.axes = dict(DEFAULT_AXES)

    # ── write er301_routes.py ──
    def _ctl(self, i: InputDef) -> str:
        names = self.nrpn_names if i.kind == "nrpn" else self.cc_names
        if i.name and i.name in names:
            return f"'{i.name}'"
        return str(self.number(i))

    @staticmethod
    def _ext_literal(s: str) -> str:
        t = _strip(s)
        n = _int_or_none(t)
        if n is not None and t.lstrip("+-").isdigit():
            return str(n)
        clean = "".join(ch for ch in t if ch.isalnum() or ch == "_")
        return f"'{clean or 'unknown'}'"

    @staticmethod
    def _line(body: str, comment: str) -> str:
        pad = max(1, 30 - len(body))
        return "    " + body + " " * pad + "# " + comment + "\n"

    def python_source(self, notes: Optional[Dict[str, str]] = None) -> str:
        notes = notes or {}

        def noted(c, key, port_key=None):
            n = notes.get(key)
            c = c if n is None else c + " · " + n.replace("\n", " ")
            d = notes.get("er:" + port_key) if port_key else None
            return c if d is None else c + " » " + d.replace("\n", " ")

        s = (
            '"""\n'
            "ER-301 routing inside the i2c2midi (DualSense → ER-301 directly, no Teletype script).\n"
            "\n"
            "Written by Kontrolleur. You can still edit it by hand; the app reads it back.\n"
            "Saving makes the MIDI hub (midi-passthrough service) send it to the i2c2midi within a\n"
            "second; the i2c2midi keeps it across power cycles. Check by hand:\n"
            "\n"
            "    python3 ~/dualsense-eurorack-midi/scripts/er301_push.py\n"
            "\n"
            "Ports are the ER-301 SC.CV / SC.TR numbers (1–99) chosen as the source of a unit's input.\n"
            "Entries are (control, port) on CHANNEL, or (channel, control, port) for another channel.\n"
            '"""\n'
            "\n"
            f"CHANNEL = {self.channel}            # DualSense instrument channel\n"
            "\n"
        )
        tp = self.trigger_port_name

        def section(header, name, rows):
            nonlocal s
            s += header + f"\n{name} = [\n" + "".join(rows) + "]\n\n"

        def ext_key(e):
            return f"ext:{e.channel}:{_strip(e.control)}"

        L, eff = self._line, self.effective_role
        tk = "CV" if self.triggers_as_cv else "TR"
        rows = [L(f"({self._ctl(i)}, {self.port(i)}),", noted(f"{input_title(i)} → SC.CV {self.port(i)}", "ds:" + i.id, f"CV{self.port(i)}"))
                for i in ALL_INPUTS if i.kind == "cc" and eff(i) == CV]
        rows += [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} CC {e.control} → SC.CV {e.port}", ext_key(e), f"CV{e.port}"))
                 for e in self.externals if e.kind == EXT_CV]
        section("# CC → SC.CV   (0–127 → 0–10 V)", "CV", rows)

        rows = [L(f"({self._ctl(i)}, {self.port(i)}),", noted(f"{input_title(i)} → SC.CV {self.port(i)}", "ds:" + i.id, f"CV{self.port(i)}"))
                for i in ALL_INPUTS if i.kind == "nrpn" and eff(i) == CV]
        rows += [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} NRPN {e.control} → SC.CV {e.port}", ext_key(e), f"CV{e.port}"))
                 for e in self.externals if e.kind == EXT_NRPN]
        section("# 14-bit NRPN → SC.CV   (0–16383 → 0–10 V, full resolution)", "NRPN_CV", rows)

        rows = [L(f"({self._ctl(i)}, {self.port(i)}),", noted(f"{input_title(i)} → {tp} {self.port(i)}", "ds:" + i.id, tk + str(self.port(i))))
                for i in ALL_INPUTS if i.kind == "cc" and eff(i) == TRIGGER]
        rows += [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} CC {e.control} → {tp} {e.port}", ext_key(e), tk + str(e.port)))
                 for e in self.externals if e.kind == EXT_TRIGGER]
        section("# CC as gate (≥ 64 = high) → trigger port", "TRIGGERS", rows)

        rows = [L(f"({e.channel}, {self._ext_literal(e.control)}, {e.port}),", noted(f"ch {e.channel} note {e.control} → {tp} {e.port}", ext_key(e), tk + str(e.port)))
                for e in self.externals if e.kind == EXT_NOTE]
        section("# Note held → trigger port. These notes do not change the pitch.", "NOTE_TRIGGERS", rows)

        s += "# Notes on CHANNEL → V/Oct. The octave control adds steps of semitones_per_step.\n"
        if self.pitch_enabled:
            oc = self.input_with(OCTAVE)
            ocs = self._ctl(oc) if oc else "None"
            s += f"PITCH = dict(port={self.pitch_port}, zero_note={self.zero_note}, octave_control={ocs}, semitones_per_step={self.semis_per_step})\n\n"
        else:
            s += "PITCH = None\n\n"

        s += "# Trigger LFO: rate control 0 = stopped, 1–127 = min_hz … max_hz (exponential).\n"
        rate = self.input_with(LFO_RATE)
        if rate or self.lfo_rate_src:
            reset = self.input_with(LFO_RESET)
            rs = str(self.lfo_reset_src[2]) if self.lfo_reset_src else (self._ctl(reset) if reset else "None")
            rc = str(self.lfo_rate_src[2]) if self.lfo_rate_src else self._ctl(rate)
            s += f"LFO = dict(port={self.lfo_port}, rate_control={rc}, reset_control={rs},\n"
            if self.lfo_rate_src or self.lfo_reset_src:
                extra = [f"channel={self.lfo_channel}"]
                if self.lfo_rate_src:
                    extra.append(f"source='{_src_text(self.lfo_rate_src)}'")
                if self.lfo_reset_src:
                    extra.append(f"reset_source='{_src_text(self.lfo_reset_src)}'")
                s += "           " + ", ".join(extra) + ",   # rate / reset from another controller\n"
            s += f"           min_hz={'%g' % self.lfo_min_hz}, max_hz={'%g' % self.lfo_max_hz}, pulse_ms={self.lfo_pulse_ms})\n\n"
        else:
            s += "LFO = None\n\n"

        s += "# True: trigger ports are SC.CV ports (10 V pulse / gate). False: SC.TR ports.\n"
        s += f"TRIGGERS_AS_CV = {'True' if self.triggers_as_cv else 'False'}\n"
        return s

    # ── write controls.py ──
    def controls_source(self) -> str:
        s = (
            '"""\n'
            "What the DualSense buttons do. Read by the DualSense script; changes apply within a second.\n"
            "Written by Kontrolleur. You can still edit it by hand; the app reads it back.\n"
            "\n"
            "tap  = short press   (loop actions fire when you let go)\n"
            "hold = held for LONG_PRESS_DURATION (mappings.py) or longer, decided when you let go\n"
            "'midi' = the button sends its own CC (127 pressed / 0 released): route it to the ER-301\n"
            "         in er301_routes.py. Buttons: cross circle triangle square l1 r1 l3 r3 create\n"
            "         options ps mic touchpad (= touchpad click).\n"
            "Actions: none midi loop_record loop_play loop_clear motion block_up block_down block_reset\n"
            "         section_up section_down strum freeze_left freeze_right quantize\n"
            '"""\n'
            "\n"
            "BUTTONS = {\n"
        )
        for name, _ in BUTTONS:
            f = self.fn(name)
            key = f"'{name}':"
            body = f"dict(tap='{f.tap}'"
            if f.hold != "none":
                body += f", hold='{f.hold}'"
            body += "),"
            s += "    " + key + " " * max(1, 12 - len(key)) + body + "\n"
        s += "}\n\n# Two buttons pressed together. The buttons' own release actions are skipped then.\nCHORDS = [\n"
        for c in self.chords:
            s += f"    ('{c.a}', '{c.b}', '{c.action}'),\n"
        s += "]\n\n"
        s += ("# What each axis does: 'midi' (its own CC / NRPN, route it in er301_routes.py), 'none', 'slice_fine',\n"
              "# 'slice_section', 'slice_block' (grain slice), 'loop_scrub' (playing loop position), 'quantize_grid'.\n"
              "# The D-pad steps: 'midi', 'none', 'slice_section', 'slice_block'. Tilt / twist only while motion is on.\n"
              "AXES = {\n")
        for a in AXES:
            key = f"'{a}':"
            s += "    " + key + " " * max(1, 11 - len(key)) + f"'{self.axes.get(a, DEFAULT_AXES[a])}',\n"
        s += "}\n\n"
        s += "# For DualSense scripts from before AXES:\n"
        s += f"DPAD_HORIZONTAL = '{self.dpad_horizontal}'\n"
        s += f"LEFT_STICK_X = '{self.left_stick_x}'\n"
        s += ("\n# MIDI from the other controllers → the same functions: (device, channel, 'cc' | 'note', number, function).\n"
              "# device: '16n', 'pedal' (MPC-20 via DOREMiDi) or 'ec4'; channel as the device sends it. A CC drives a\n"
              "# continuous function with its value, or fires an action when it goes to 64 or above (notes: note on).\n"
              "MIDI_INPUTS = [\n")
        for x in sorted(self.midi_inputs, key=lambda x: ([d for d, _ in PI_DEVICES].index(x.device), x.channel, x.kind, x.number)):
            s += f"    ('{x.device}', {x.channel}, '{x.kind}', {x.number}, '{x.function}'),\n"
        s += "]\n"
        return s

    # ── CC → CV routes of other devices (16n faders, pedals, EC4 encoders) ──
    def _find_cc_route(self, channel: int, cc: int) -> int:
        for k, e in enumerate(self.externals):
            if e.kind == EXT_CV and e.channel == channel and _int_or_none(e.control) == cc:
                return k
        return -1

    def fader_port(self, channel: int, cc: int) -> Optional[int]:
        k = self._find_cc_route(channel, cc)
        return self.externals[k].port if k >= 0 else None

    def set_fader_port(self, channel: int, cc: int, port: Optional[int]):
        k = self._find_cc_route(channel, cc)
        if k >= 0:
            if port is not None:
                self.externals[k].port = port
            else:
                del self.externals[k]
        elif port is not None:
            self.externals.append(ExternalRoute(EXT_CV, channel, str(cc), port))

    # ── routes of other devices: CV, trigger (CC ≥ 64 = high) or note → trigger ──
    def ext_route(self, channel: int, number: int, note: bool = False) -> Optional[ExternalRoute]:
        kinds = (EXT_NOTE,) if note else (EXT_CV, EXT_TRIGGER)
        for e in self.externals:
            if e.kind in kinds and e.channel == channel and _int_or_none(e.control) == number:
                return e
        return None

    def set_ext_route(self, channel: int, number: int, kind: Optional[str], port: Optional[int] = None, note: bool = False):
        """kind None removes the route; otherwise sets kind (and port, or keeps / picks a free one)."""
        e = self.ext_route(channel, number, note)
        if kind is None:
            if e is not None:
                self.externals.remove(e)
            return
        if e is None:
            self.externals.append(ExternalRoute(kind, channel, str(number), port or self.first_free_port()))
        else:
            e.kind = kind
            if port is not None:
                e.port = port

    def move_ext_route(self, old: Tuple[int, int], new: Tuple[int, int], note: bool = False):
        """A device control changed channel / number: its route goes along."""
        if old == new:
            return
        e = self.ext_route(old[0], old[1], note)
        if e is not None and self.ext_route(new[0], new[1], note) is None:
            e.channel, e.control = new[0], str(new[1])

    def route_label(self, e: ExternalRoute) -> str:
        if e.kind in (EXT_CV, EXT_NRPN):
            return f"CV {e.port}"
        return f"TRIG › CV {e.port}" if self.triggers_as_cv else f"TR {e.port}"

    def problems(self) -> List[str]:
        p = []
        if self.lfo_min_hz < 0.01 or self.lfo_max_hz > 163.83 or self.lfo_min_hz > self.lfo_max_hz:
            p.append("LFO range must be 0.01 ≤ min ≤ max ≤ 163.83 Hz.")
        lp = self.lfo_problem()
        if lp:
            p.append(lp)
        for c in self.chords:
            if c.a == c.b:
                p.append(f"Combo {button_label(c.a)} + {button_label(c.b)} needs two different buttons.")
        for e in self.externals:
            t = _strip(e.control)
            n = _int_or_none(t)
            if n is not None:
                mx = 16383 if e.kind == EXT_NRPN else 127
                if n < 0 or n > mx:
                    p.append(f"{e.kind} ch {e.channel}: {n} is out of range 0–{mx}.")
            else:
                names = self.nrpn_names if e.kind == EXT_NRPN else self.cc_names
                if e.kind == EXT_NOTE or (names and t not in names):
                    what = "note number" if e.kind == EXT_NOTE else "control"
                    p.append(f"{e.kind} ch {e.channel}: “{t}” is not a known {what}.")
        return p

    # ── configuration files (same JSON as the Mac app's MappingSnapshot) ──
    def snapshot(self) -> dict:
        return {
            "channel": self.channel, "roles": dict(self.roles), "ports": dict(self.ports),
            "pitchEnabled": self.pitch_enabled, "pitchPort": self.pitch_port, "zeroNote": self.zero_note,
            "semisPerStep": self.semis_per_step, "lfoPort": self.lfo_port, "lfoMinHz": self.lfo_min_hz,
            "lfoMaxHz": self.lfo_max_hz, "lfoPulseMs": self.lfo_pulse_ms, "triggersAsCV": self.triggers_as_cv,
            "externals": [{"kind": e.kind, "channel": e.channel, "control": e.control, "port": e.port} for e in self.externals],
            "funcs": {k: {"tap": v.tap, "hold": v.hold} for k, v in self.funcs.items()},
            "chords": [{"a": c.a, "b": c.b, "action": c.action} for c in self.chords],
            "dpadHorizontal": self.dpad_horizontal, "leftStickX": self.left_stick_x, "longPress": self.long_press,
            "axes": dict(self.axes),
            "midiInputs": [[x.device, x.channel, x.kind, x.number, x.function] for x in self.midi_inputs],
            "lfoRateSrc": list(self.lfo_rate_src) if self.lfo_rate_src else None,
            "lfoResetSrc": list(self.lfo_reset_src) if self.lfo_reset_src else None,
        }

    def restore(self, s: dict):
        self.channel = s["channel"]
        self.roles = {k: v for k, v in s["roles"].items() if v in ROLES}
        self.ports = dict(s["ports"])
        self.pitch_enabled, self.pitch_port = s["pitchEnabled"], s["pitchPort"]
        self.zero_note, self.semis_per_step = s["zeroNote"], s["semisPerStep"]
        self.lfo_port, self.lfo_min_hz, self.lfo_max_hz = s["lfoPort"], float(s["lfoMinHz"]), float(s["lfoMaxHz"])
        self.lfo_pulse_ms, self.triggers_as_cv = s["lfoPulseMs"], s["triggersAsCV"]
        self.externals = [ExternalRoute(e["kind"], e["channel"], e["control"], e["port"])
                          for e in s["externals"] if e["kind"] in EXT_KINDS]
        f = default_funcs()
        for k, v in s["funcs"].items():
            if v["tap"] in ACTIONS and v["hold"] in ACTIONS:
                f[k] = ButtonFunc(v["tap"], v["hold"])
        self.funcs = f
        self.chords = [Chord(c["a"], c["b"], c["action"]) for c in s["chords"] if c["action"] in ACTIONS]
        axes = dict(DEFAULT_AXES)
        axes["dpad_x"], axes["left_x"] = s.get("dpadHorizontal", "slice_section"), s.get("leftStickX", "slice_fine")
        for k, v in (s.get("axes") or {}).items():
            if k in axes and v in AXIS_FUNCS:
                axes[k] = v
        self.axes = axes
        self.long_press = float(s["longPress"])
        self.midi_inputs = [MidiInput(x[0], x[1], x[2], x[3], x[4]) for x in s.get("midiInputs") or []
                            if len(x) == 5 and x[0] in PI_DEVICE_TITLE and x[4] in CONT_FUNCS + INPUT_ACTIONS]
        self.lfo_rate_src = tuple(s["lfoRateSrc"]) if s.get("lfoRateSrc") else None
        self.lfo_reset_src = tuple(s["lfoResetSrc"]) if s.get("lfoResetSrc") else None
