"""
The glide engine: moves values from where they are to where a snapshot wants them, over a time.

A track is either
  - a glide: a number moving from start to target (sent whenever its integer value changes), or
  - a jump:  something that can't glide (a slice, freeze, a scene) and is set in one go, at the start or
             the end of the glide.
Loading another snapshot during a glide simply starts new tracks from the values reached so far
(the providers know them, because every value sent goes through them).
"""
import math

CURVES = {
    "linear": lambda p: p,
    "smooth": lambda p: p * p * (3 - 2 * p),           # ease in and out
    "exp": lambda p: (math.exp(4 * p) - 1) / (math.exp(4) - 1),   # slow start, for long fades
}


class Track:
    __slots__ = ("start", "target", "t0", "duration", "send", "jump", "last", "done")

    def __init__(self, start, target, t0, duration, send, jump):
        self.start = start
        self.target = target
        self.t0 = t0
        self.duration = duration
        self.send = send
        self.jump = jump          # None for a glide, "start" / "end" for a jump
        self.last = None
        self.done = False


class Glide:
    def __init__(self, curve="linear"):
        self.tracks = {}
        self.curve = CURVES.get(curve, CURVES["linear"])
        self.t_end = 0.0

    def set_curve(self, name):
        self.curve = CURVES.get(name, CURVES["linear"])

    def glide(self, key, start, target, duration, now, send):
        """Move key from start to target over duration seconds. start None (value unknown) = set at once."""
        if start is None or duration <= 0 or start == target:
            self.tracks.pop(key, None)
            send(target)
            return
        tr = Track(start, target, now, duration, send, None)
        tr.last = start
        self.tracks[key] = tr
        self.t_end = max(self.t_end, now + duration)

    def jump(self, key, value, duration, now, send, at="end"):
        """Set something that can't glide, at the start or at the end of the glide."""
        if duration <= 0 or at == "start":
            self.tracks.pop(key, None)
            send(value)
            return
        self.tracks[key] = Track(None, value, now, duration, send, "end")
        self.t_end = max(self.t_end, now + duration)

    def cancel(self, key):
        """A hand took over this value: stop moving it."""
        return self.tracks.pop(key, None) is not None

    def cancel_all(self):
        self.tracks.clear()
        self.t_end = 0.0

    def gliding(self, key) -> bool:
        return key in self.tracks

    @property
    def active(self) -> bool:
        return bool(self.tracks)

    def tick(self, now):
        """Advance every track; returns the number of values sent."""
        sent = 0
        for key, tr in list(self.tracks.items()):
            p = (now - tr.t0) / tr.duration if tr.duration > 0 else 1.0
            if tr.jump is not None:
                if p >= 1.0:
                    tr.send(tr.target)
                    sent += 1
                    del self.tracks[key]
                continue
            if p >= 1.0:
                if tr.last != tr.target:
                    tr.send(tr.target)
                    sent += 1
                del self.tracks[key]
                continue
            v = round(tr.start + (tr.target - tr.start) * self.curve(max(0.0, p)))
            if v != tr.last:
                tr.last = v
                tr.send(v)
                sent += 1
        if not self.tracks:
            self.t_end = 0.0
        return sent
