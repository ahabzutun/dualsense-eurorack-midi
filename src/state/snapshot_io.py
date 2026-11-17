"""
Snapshots of the DualSense script's own state, for the grid macros (src/grid_macros.py).

What a snapshot holds: the grain slice position (sample block, section, fine slice), freeze of the left
group (L2, R2, left stick) and of the right stick with their held values, and motion control on/off —
all for the current instrument.

The grid macros ask for it and set it over UDP on 127.0.0.1:47122 (JSON):
    {"cmd": "capture"}                 → {"ok": true, "state": {…}}
    {"cmd": "apply", "state": {…}}     → {"ok": true}
The socket is polled from the script's main loop, so applying never races the controller handling.
"""
import json
import socket

from config.mappings import CC_MAP

PORT = 47122


def capture(controller, channel_mgr) -> dict:
    fs = channel_mgr.get_current_freeze_state()
    idx = channel_mgr.current_channel - 1
    return {
        "instrument": channel_mgr.current_channel,
        "block": controller.sample_block,
        "section": controller.slice_bank,
        "fine": controller.current_fine_slice,
        "slice": controller.compute_grain_slice(),
        "freeze": {
            "left": bool(fs.left_stick_frozen),
            "l2": fs.l2_value, "r2": fs.r2_value,
            "left_x": fs.left_x_value, "left_y": fs.left_y_value,
            "right": bool(fs.right_stick_frozen),
            "right_x": fs.right_x_value, "right_y": fs.right_y_value,
        },
        "motion": bool(controller.motion_enabled[idx]),
    }


def _clamp(v, lo, hi, default):
    try:
        return max(lo, min(hi, int(v)))
    except (TypeError, ValueError):
        return default


def apply(controller, channel_mgr, state, midiout) -> None:
    """Set the script to a captured state and send what changed. Missing fields are left as they are."""
    if not isinstance(state, dict):
        return
    c = controller
    # grain slice position
    if any(k in state for k in ("block", "section", "fine")):
        c.sample_block = _clamp(state.get("block"), 0, c.max_blocks, c.sample_block)
        c.slice_bank = _clamp(state.get("section"), 0, c.max_banks, c.slice_bank)
        c.current_fine_slice = _clamp(state.get("fine"), 0, c.slices_per_bank - 1, c.current_fine_slice)
        c.last_grain_slice = -1                       # send even if it happens to be the same slice
        c.send_grain_slice(midiout)
        try:
            c.show_block()
        except Exception:
            pass
    # freeze (values are stored as sent: MIDI 0-127)
    fr = state.get("freeze")
    if isinstance(fr, dict):
        fs = channel_mgr.get_current_freeze_state()
        if "left" in fr:
            on = bool(fr["left"])
            fs.l2_frozen = fs.r2_frozen = fs.left_stick_frozen = on
            if on:
                fs.l2_value = _clamp(fr.get("l2"), 0, 127, fs.l2_value)
                fs.r2_value = _clamp(fr.get("r2"), 0, 127, fs.r2_value)
                fs.left_x_value = _clamp(fr.get("left_x"), 0, 127, fs.left_x_value)
                fs.left_y_value = _clamp(fr.get("left_y"), 0, 127, fs.left_y_value)
        if "right" in fr:
            on = bool(fr["right"])
            fs.right_stick_frozen = on
            if on:
                fs.right_x_value = _clamp(fr.get("right_x"), 0, 127, fs.right_x_value)
                fs.right_y_value = _clamp(fr.get("right_y"), 0, 127, fs.right_y_value)
        channel_mgr.send_frozen_values_on_channel_switch(midiout, c, CC_MAP)
    # motion control
    if "motion" in state:
        idx = channel_mgr.current_channel - 1
        on = bool(state["motion"])
        if bool(c.motion_enabled[idx]) != on:
            c.motion_enabled[idx] = on
            try:
                c.update_led_color()
            except Exception:
                pass
    print(f"🎛️  Grid macro → DualSense state: slice {c.compute_grain_slice()}")


class SnapshotServer:
    """Non-blocking UDP endpoint, polled from the main loop. One per process (main() may restart)."""

    def __init__(self, port=PORT):
        self.sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", port))
            s.setblocking(False)
            self.sock = s
        except OSError as e:
            print(f"⚠️  Snapshot socket not available ({e}); grid macros can't reach the DualSense state")

    def poll(self, controller, channel_mgr, midiout) -> int:
        """Answer everything waiting. Returns the number of requests handled."""
        if self.sock is None:
            return 0
        n = 0
        while True:
            try:
                data, addr = self.sock.recvfrom(65536)
            except (BlockingIOError, InterruptedError):
                return n
            except OSError:
                return n
            n += 1
            try:
                req = json.loads(data.decode())
                cmd = req.get("cmd")
                if cmd == "capture":
                    reply = {"ok": True, "state": capture(controller, channel_mgr)}
                elif cmd == "apply":
                    apply(controller, channel_mgr, req.get("state"), midiout)
                    reply = {"ok": True}
                else:
                    reply = {"ok": False, "error": f"unknown command {cmd!r}"}
            except Exception as e:
                reply = {"ok": False, "error": str(e)}
            try:
                self.sock.sendto(json.dumps(reply).encode(), addr)
            except OSError:
                pass


_server = None


def server():
    global _server
    if _server is None:
        _server = SnapshotServer()
    return _server
