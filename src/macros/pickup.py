"""
Pickup ("soft takeover") for controls with a physical position, like the 16n faders.

After a snapshot sets a parameter, the fader that drives it is somewhere else. Until the fader passes the
parameter's current value (or comes within `window` of it), its messages are held back, so touching it
doesn't make the parameter jump. The target follows every value the macros send for that parameter, so a
fader can also catch a value while it is still gliding.

Used twice with the same rules: by the hub (which drops the held-back messages) and by the macro service
(which needs to know when a hand has taken a value over).
"""


class Pickup:
    def __init__(self, window=2):
        self.window = window
        self.targets = {}      # key -> current value of the parameter
        self.side = {}         # key -> -1 / +1: which side of the target the control was last on

    def arm(self, key, target):
        self.targets[key] = target
        self.side.pop(key, None)

    def update_target(self, key, target):
        """The parameter moved (a glide step). Only matters while the key is armed."""
        if key in self.targets:
            self.targets[key] = target

    def armed(self, key) -> bool:
        return key in self.targets

    def disarm(self, key):
        self.targets.pop(key, None)
        self.side.pop(key, None)

    def clear(self):
        self.targets.clear()
        self.side.clear()

    def accept(self, key, value) -> bool:
        """True: the control drives the parameter (now or from now on). False: hold the message back."""
        target = self.targets.get(key)
        if target is None:
            return True
        diff = value - target
        if abs(diff) <= self.window:
            self.disarm(key)
            return True
        side = 1 if diff > 0 else -1
        before = self.side.get(key)
        self.side[key] = side
        if before is not None and before != side:     # crossed over the target between two messages
            self.disarm(key)
            return True
        return False
