"""
MIDI Passthrough Hub for DualSense Eurorack system
Routes MIDI between the controllers, the NerdSEQ and the MIDI bus.

THE BUS: every XIAO link (mysteryman, jumpingman, nadine … see ~/xiao-uart/links.json) and every MIDI
device plugged into the Pi that isn't one of the known controllers is a bus member. Members are
interchangeable — it doesn't matter what is plugged into which link: each one gets the whole MIDI flow
(controllers + the other members) and what it sends goes to everyone else, never back to itself.
Echoes (a device sending back what it just received, MIDI thru) are dropped, so two echoing devices
can't loop. CC / pressure / bend floods to members are thinned (latest value every 10 ms), and the
ER-301 routing SysEx goes to all members: the i2c2midi answers from wherever it is plugged in.
Controllers (INPUT_ONLY) only send into the hub. loglady is not MIDI (Teletype grid + keyboard).

FIX: rtmidi MidiIn objects were not having their callbacks cancelled before
     close_port() and del, creating a reference cycle (MidiIn → callback closure
     → self.outputs → MidiIn) that prevented the C++ ALSA client destructor from
     running. With a 2-second rescan cycle and any brief disconnections, leaked
     ALSA clients accumulate until the system-wide cap of 64 is hit and all
     MIDI creation fails.

     Fix: call midiin.cancel_callback() before close_port(), then explicitly
     del the object so Python releases it immediately.
"""

#!/usr/bin/env python3
import rtmidi
import gc
try:
    import osc_out          # OpenMW OSC tap (adds no MIDI clients)
except Exception as _e:
    osc_out = None
    print(f"osc_out tap not loaded: {_e}")
try:
    from macros.hub_pickup import HubPickup, MACRO_PORT_NAME   # grid macros: 16n pickup after a load
except Exception as _e:
    HubPickup = None
    MACRO_PORT_NAME = "Grid_Macros"
    print(f"grid macro pickup not loaded: {_e}")
import time
import threading
import collections
from xiao_links import link_names, client_of
try:
    import er301_config     # ER-301 routing for the i2c2midi (config/er301_routes.py)
except Exception as _e:
    er301_config = None
    print(f"er301_config not loaded: {_e}")


# Controllers: they send into the hub, nothing is sent to them (not bus members).
INPUT_ONLY = ("16n", "Faderfox", "EC4", "CH345", "DOREMiDi", "DOReMiDi", "MPC-20", "iii grid", "monome",
              "Midi Through", "DualSense", "RtMidi", "f_midi", MACRO_PORT_NAME)


def is_nerdseq_port(port_name):
    return "NerdSEQ" in port_name or "USB MIDI" in port_name


def bus_member(port_name):
    """'link' for a XIAO link, 'device' for unknown MIDI hardware on the Pi, None otherwise."""
    client, num = client_of(port_name)
    if client in link_names():
        return "link"
    if num < 128 and not is_nerdseq_port(port_name) and not any(k in port_name for k in INPUT_ONLY):
        return "device"
    return None


def bus_key(port_name):
    return "bus:" + client_of(port_name)[0]


class MIDIHub:
    def __init__(self):
        self.outputs = {}
        self.inputs = {}
        self.skip_devices = ["f_midi", "Midi Through"]
        # echo guard: what was sent to each bus member lately (a member sending it back = echo, dropped)
        self.ECHO_WINDOW = 0.25
        self._recent_sent = {}
        self._echo_lock = threading.Lock()
        self._out_port_names = {}   # output key -> ALSA port name currently open
        self.running = True
        self.lock = threading.Lock()
        self.pickup = HubPickup() if HubPickup is not None else None

        # ── Diagnostic mode ──────────────────────────────────────────────────
        # Set to True to print raw 16n values WITHOUT forwarding.
        # Set to False for normal operation.
        self.DIAGNOSTIC_MODE = False
        self.DIAGNOSTIC_FILTER_CHANNEL = 16
        self.DIAGNOSTIC_FILTER_CC_MIN  = 80
        self.DIAGNOSTIC_FILTER_CC_MAX  = 95
        # ─────────────────────────────────────────────────────────────────────

        # ── 16n → SSP rescaling ───────────────────────────────────────────────
        # The SSP maps CC 0-127 to its internal 0.0-1.0 range, but PMIX/ATTN
        # treat 1.0 as maximum gain (far above unity), so full fader travel
        # slams the output immediately. We rescale 16n fader values (ch16,
        # CC 80-95) before forwarding to SSP only, leaving NerdSEQ untouched.
        #
        # TUNING: raise SSP_CC_MAX if the faders feel too restricted,
        #         lower it if they still max out too early.
        #         Start at 80 (~63% of full scale) and adjust by feel.
        #
        self.RESCALE_ENABLED  = True
        self.RESCALE_CHANNEL  = 16          # only rescale ch16 (the 16n)
        self.RESCALE_CC_MIN   = 80          # first fader CC
        self.RESCALE_CC_MAX   = 95          # last fader CC
        self.SSP_CC_MAX       = 127         # 127 = no compression; full fader = 1.0 (0dB on PMIX)
                                            # Lower this only if modulation adds on top of a non-zero base
        # ─────────────────────────────────────────────────────────────────────

        # ── Teletype (i2c2midi) output coalescing ─────────────────────────────
        # i2c2midi only stores the latest value per CC and Teletype polls it every
        # few tens of ms, so CC / channel-pressure / pitch-bend floods (16n sweeps,
        # DualSense gyro) are reduced to the latest value per controller and
        # flushed every TT_FLUSH_INTERVAL. Notes, clock etc. are sent immediately.
        # Keeps the 115200-baud XIAO link clear. SSP/NerdSEQ are not affected.
        self.TT_COALESCE_ENABLED = True
        self.TT_FLUSH_INTERVAL   = 0.010     # seconds
        # Channels that go straight through (no coalescing): ch 2 is the DualSense
        # instrument that i2c2midi turns into ER-301 gates/triggers — timing-critical.
        self.TT_COALESCE_EXEMPT_CHANNELS = {2}
        self._tt_pending = {}
        self._tt_lock = threading.Lock()
        # ER-301 routing (config/er301_routes.py) is sent to the i2c2midi as SysEx
        # whenever the XIAO-TT output (re)connects or the file changes.
        self._er301_mtime = None
        self._er301_push_pending = False
        # …and confirmed: the i2c2midi answers every routing with accepted / rejected and its route counts
        # (SysEx on the XIAO-TT input). Without an answer (rack off) it is sent again until one comes;
        # once confirmed, a query every ER301_CHECK_EVERY s notices a rack that was off in between.
        # The state goes to live_config.STATUS for the routes tool / Kontrolleur.
        self.ER301_ANSWER_WITHIN = 3.0
        self.ER301_RETRY = (5, 10, 20, 40, 60)       # seconds between tries while nothing answers
        self.ER301_CHECK_EVERY = 60.0
        self._er301 = {"state": "unknown", "expected": None, "summary": "", "sent_at": 0.0, "kind": None,
                       "tries": 0, "next": 0.0, "file_mtime": None, "reply": None, "checked": 0.0}
        self._er301_reply = None                       # (time, dict) — set by the XIAO-TT callback
        # ─────────────────────────────────────────────────────────────────────

        # Create scanners ONCE and reuse them (never re-create these)
        self.output_scanner = rtmidi.MidiOut()
        self.input_scanner = rtmidi.MidiIn()

        # Track connected sets to suppress redundant log spam
        self._last_output_names = set()
        self._last_input_names = set()

        # ── DOReMIDI channel remapping ────────────────────────────────────────
        # The CH345/DOReMIDI pedals send on channel 1. We remap them to
        # follow the current DualSense instrument channel (written to
        # /tmp/dualsense_channel on every channel switch).
        self.PEDAL_REMAP_ENABLED  = True
        self.PEDAL_SOURCE_CHANNEL = 1     # what the DOReMIDI sends on
        self._channel_file        = '/tmp/dualsense_channel'
        self._cached_target_channel = 1  # in-memory cache — updated by monitor loop
        # ─────────────────────────────────────────────────────────────────────

    def scan_and_update_outputs(self):
        """Find and open all output devices"""
        try:
            available = self.output_scanner.get_ports()
        except Exception as e:
            print(f"⚠️  Cannot scan outputs: {e} — clearing stale outputs for clean reconnect")
            # Force-close all current outputs so the next successful scan
            # reconnects cleanly rather than leaving broken ports open.
            with self.lock:
                for name in list(self.outputs.keys()):
                    midiout = self.outputs.pop(name)
                    try:
                        midiout.close_port()
                    except Exception:
                        pass
                    del midiout
            self._last_output_names = set()
            return

        found_outputs = {}

        for i, port_name in enumerate(available):
            if "NerdSEQ" in port_name:
                found_outputs['NerdSEQ'] = (i, port_name)
            elif "USB MIDI" in port_name and 'NerdSEQ' not in found_outputs:
                # NerdSEQ sometimes enumerates with generic "USB MIDI" name
                # instead of "XOR NerdSEQ" after reboot. Accept as fallback
                # only if a proper NerdSEQ port hasn't already been found.
                found_outputs['NerdSEQ'] = (i, port_name)
            elif bus_member(port_name):
                found_outputs[bus_key(port_name)] = (i, port_name)

        current_names = set(found_outputs.keys())
        changed = current_names != self._last_output_names

        with self.lock:
            # Close outputs that are no longer available, or whose port changed
            for name in list(self.outputs.keys()):
                moved = (name in found_outputs and
                         self._out_port_names.get(name) != found_outputs[name][1])
                if name not in found_outputs or moved:
                    if moved:
                        print(f"🔀 Output {name} moving to {found_outputs[name][1]}")
                        changed = True
                    else:
                        print(f"❌ Output disconnected: {name}")
                    self._out_port_names.pop(name, None)
                    midiout = self.outputs.pop(name)
                    try:
                        midiout.close_port()
                    except Exception:
                        pass
                    del midiout   # ensure C++ destructor runs now

            # Open new outputs
            for name, (port_num, port_name) in found_outputs.items():
                if name not in self.outputs:
                    try:
                        midiout = rtmidi.MidiOut()
                    except Exception as e:
                        print(f"⚠️  Cannot create {port_name} client: {e}")
                        continue

                    try:
                        midiout.open_port(port_num)
                        self.outputs[name] = midiout
                        self._out_port_names[name] = port_name
                        print(f"✅ Output connected: {port_name}")
                        if name.startswith('bus:'):
                            self._er301_push_pending = True    # wherever the i2c2midi is plugged in
                    except Exception as e:
                        print(f"⚠️  Failed to open {port_name}: {e}")
                        del midiout   # don't leak the client on failure either

        if changed:
            self._last_output_names = current_names
            print(f"🔊 Active outputs: {list(self.outputs.keys())}")

    def scan_and_update_inputs(self):
        """Find and open all input devices"""
        try:
            available = self.input_scanner.get_ports()
        except Exception as e:
            print(f"⚠️  Cannot scan inputs: {e} — clearing stale inputs for clean reconnect")
            with self.lock:
                for port_name in list(self.inputs.keys()):
                    midiin = self.inputs.pop(port_name)
                    try:
                        midiin.cancel_callback()
                        midiin.close_port()
                    except Exception:
                        pass
                    del midiin
            self._last_input_names = set()
            return

        found_inputs = {}

        for i, port_name in enumerate(available):
            should_skip = any(skip in port_name for skip in self.skip_devices)
            if ("RtMidi output" in port_name and "DualSense_Controller" not in port_name
                    and MACRO_PORT_NAME not in port_name):
                should_skip = True
            if not should_skip:
                found_inputs[port_name] = i

        current_names = set(found_inputs.keys())
        changed = current_names != self._last_input_names

        with self.lock:
            # Close inputs that are no longer available
            for port_name in list(self.inputs.keys()):
                if port_name not in found_inputs:
                    print(f"❌ Input disconnected: {port_name}")
                    midiin = self.inputs.pop(port_name)
                    try:
                        midiin.cancel_callback()
                        midiin.close_port()
                    except Exception:
                        pass
                    del midiin

            # Open new inputs
            for port_name, port_num in found_inputs.items():
                if port_name not in self.inputs:
                    try:
                        midiin = rtmidi.MidiIn()
                    except Exception as e:
                        print(f"⚠️  Cannot create {port_name} client: {e}")
                        continue

                    try:
                        midiin.open_port(port_num)
                        if bus_member(port_name):
                            midiin.ignore_types(sysex=False, timing=True, active_sense=True)   # i2c2midi answers
                        midiin.set_callback(self.make_callback(port_name))
                        self.inputs[port_name] = midiin
                        print(f"✅ Input connected: {port_name}")
                    except Exception as e:
                        print(f"⚠️  Failed to open {port_name}: {e}")
                        try:
                            midiin.cancel_callback()
                        except Exception:
                            pass
                        del midiin

        if changed:
            self._last_input_names = current_names
            print(f"🎹 Active inputs:  {list(self.inputs.keys())}")

    def rescale_for_ssp(self, midi_message):
        """
        Rescale 16n fader CC values before sending to SSP.

        The SSP maps CC 0-127 → 0.0-1.0 internally, but PMIX/ATTN treat
        1.0 as maximum gain (well above unity), so even small CC values
        slam the output. We compress the range so full fader travel = SSP_CC_MAX
        instead of 127.

        Returns a new message list with the rescaled value, or the original
        if this message doesn't need rescaling.
        """
        if len(midi_message) < 3:
            return midi_message

        status   = midi_message[0]
        msg_type = status & 0xF0
        channel  = (status & 0x0F) + 1

        if (msg_type == 0xB0 and
                channel == self.RESCALE_CHANNEL and
                self.RESCALE_CC_MIN <= midi_message[1] <= self.RESCALE_CC_MAX):
            raw = midi_message[2]
            rescaled = round(raw * self.SSP_CC_MAX / 127)
            rescaled = max(0, min(127, rescaled))
            return [status, midi_message[1], rescaled]

        return midi_message

    def remap_pedal_channel(self, midi_message):
        """
        Remap CH345/DOReMIDI messages from their fixed channel to the current
        DualSense instrument channel, using an in-memory cache updated by the
        monitor loop every 2 seconds (avoids per-message file I/O latency).
        """
        if not self.PEDAL_REMAP_ENABLED or len(midi_message) < 1:
            return midi_message

        status   = midi_message[0]
        msg_type = status & 0xF0
        channel  = (status & 0x0F) + 1

        # Only remap voice messages on the pedal source channel
        if channel != self.PEDAL_SOURCE_CHANNEL or msg_type not in (0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0):
            return midi_message

        target_channel = self._cached_target_channel
        if target_channel == self.PEDAL_SOURCE_CHANNEL:
            return midi_message  # already on the right channel

        new_status = msg_type | ((target_channel - 1) & 0x0F)
        return [new_status] + list(midi_message[1:])

    def _note_sent(self, out_name, msg):
        with self._echo_lock:
            dq = self._recent_sent.get(out_name)
            if dq is None:
                dq = self._recent_sent[out_name] = collections.deque(maxlen=64)
            dq.append((tuple(msg), time.monotonic()))

    def _is_echo(self, out_name, msg):
        """True if this bus member just got exactly this message from us (MIDI thru / echo)."""
        m, t0 = tuple(msg), time.monotonic() - self.ECHO_WINDOW
        with self._echo_lock:
            dq = self._recent_sent.get(out_name)
            if not dq:
                return False
            for i, (mm, t) in enumerate(dq):
                if mm == m and t >= t0:
                    del dq[i]
                    return True
        return False

    def _tt_coalesce(self, out_name, msg):
        """Queue continuous messages for a bus member; return True if queued."""
        if not self.TT_COALESCE_ENABLED or len(msg) < 2:
            return False
        if (msg[0] & 0x0F) + 1 in self.TT_COALESCE_EXEMPT_CHANNELS:
            return False
        msg_type = msg[0] & 0xF0
        if msg_type == 0xB0 and len(msg) >= 3:
            key = (out_name, msg[0], msg[1])    # per output, channel + controller
        elif msg_type in (0xD0, 0xE0):
            key = (out_name, msg[0])            # per output, channel
        else:
            return False
        with self._tt_lock:
            self._tt_pending[key] = (out_name, list(msg))
        return True

    def tt_flush_loop(self):
        """Send the latest queued CC/pressure/bend values to the bus members."""
        while self.running:
            time.sleep(self.TT_FLUSH_INTERVAL)
            with self._tt_lock:
                if not self._tt_pending:
                    continue
                pending = list(self._tt_pending.values())
                self._tt_pending.clear()
            with self.lock:
                for out_name, m in pending:
                    out = self.outputs.get(out_name)
                    if out is None:
                        continue
                    try:
                        out.send_message(m)
                        self._note_sent(out_name, m)
                    except Exception as e:
                        print(f"⚠️  Failed to flush to {out_name}: {e}")

    def make_callback(self, port_name):
        """Create a callback that forwards MIDI to all outputs.

        Routing rules:
          NerdSEQ input  → all outputs except NerdSEQ (echo prevention)
          Bus member input → all outputs except itself; echoes of what it just got are dropped
          Everything else → all outputs (bus members + NerdSEQ if present)
          CC / pressure / pitch bend to bus members are coalesced (see TT_COALESCE_*)
        """
        is_nerdseq_source = "NerdSEQ" in port_name or "USB MIDI" in port_name
        is_grid_source    = "iii grid" in port_name
        is_pedal_source   = "CH345" in port_name or "DOREMiDi" in port_name
        src_bus = bus_key(port_name) if bus_member(port_name) else None
        is_macro_source = MACRO_PORT_NAME in port_name

        def callback(message, data):
            try:
                midi_message, deltatime = message
                if midi_message and midi_message[0] == 0xF0:      # SysEx (only bus members let it through)
                    if src_bus is not None and er301_config is not None:
                        r = er301_config.parse_reply(midi_message)
                        if r is not None:
                            r = dict(r, via=client_of(port_name)[0])   # the XIAO link it is plugged into
                            self._er301_reply = (time.time(), r)
                    return                                         # never forwarded (as before)
                if src_bus is not None and self._is_echo(src_bus, midi_message):
                    return                                         # MIDI thru of what we sent it: no loop
                if self.pickup is not None:    # grid macros: faders pick up a loaded value first
                    if is_macro_source:
                        self.pickup.note_macro(midi_message)
                    elif not self.pickup.allow(port_name, midi_message):
                        return
                if osc_out is not None:        # fire OpenMW OSC for every message
                    try:
                        osc_out.handle(midi_message)
                    except Exception:
                        pass

                # ── Diagnostic mode: log raw bytes, skip forwarding ──────────
                if self.DIAGNOSTIC_MODE and ("16n" in port_name or "fader" in port_name.lower()):
                    if len(midi_message) >= 3:
                        status   = midi_message[0]
                        msg_type = status & 0xF0
                        channel  = (status & 0x0F) + 1
                        if msg_type == 0xB0:
                            cc_num = midi_message[1]
                            cc_val = midi_message[2]
                            in_range = (
                                channel == self.DIAGNOSTIC_FILTER_CHANNEL and
                                self.DIAGNOSTIC_FILTER_CC_MIN <= cc_num <= self.DIAGNOSTIC_FILTER_CC_MAX
                            )
                            if in_range:
                                bar = "█" * int(cc_val / 127 * 20)
                                print(f"🔬 [16n] Ch{channel:2d} CC{cc_num:3d} = {cc_val:3d}/127  |{bar:<20}|  raw: {[hex(b) for b in midi_message]}")
                        else:
                            print(f"🔬 [16n] Non-CC msg: {[hex(b) for b in midi_message]}")
                    return  # do NOT forward in diagnostic mode

                # ── Normal mode: forward with routing rules ──────────────────
                with self.lock:
                    if not self.outputs:
                        print(f"⚠️  No outputs available to forward to!")
                    for output_name, output in self.outputs.items():
                        # NerdSEQ: echo prevention
                        if is_nerdseq_source and output_name == 'NerdSEQ':
                            continue

                        # bus member: never back to where it came from
                        if src_bus is not None and output_name == src_bus:
                            continue

                        # Grid ch1 block: grid_bridge owns LP Mini MK3 ch1 path.
                        # Any ch1 message from the grid via passthrough would
                        # double-trigger LP commands causing stray lit matrix buttons.
                        # Grid->NerdSEQ surgical filter:
                        # - Block ch1 (LP Mini MK3 control channel, grid_bridge owns it)
                        # - Block Note On/Off (button presses, grid_bridge handles via LP)
                        # - Allow CC ch2+ (faders and LFOs reach NerdSEQ)
                        # Stray LP LED state from NerdSEQ step indicators is handled
                        # by the STALE_TIMEOUT expiry in grid_bridge, not here.
                        if is_grid_source and output_name == 'NerdSEQ':
                            _g_st = midi_message[0] if midi_message else 0
                            _g_ch = (_g_st & 0x0F) + 1
                            _g_ty = _g_st & 0xF0
                            if _g_ch == 1 or _g_ty in (0x80, 0x90):
                                continue

                        try:
                            msg_to_send = midi_message

                            # Remap DOReMIDI/CH345 pedal channel to match DualSense
                            if is_pedal_source:
                                msg_to_send = self.remap_pedal_channel(msg_to_send)

                            # SSP gets rescaled 16n values; everything else gets originals
                            if self.RESCALE_ENABLED and output_name == 'SSP':
                                rescaled = self.rescale_for_ssp(msg_to_send)
                                if len(rescaled) > 2 and len(msg_to_send) > 2 and rescaled[2] != msg_to_send[2]:
                                    print(f"🎚️  [16n→SSP] CC{msg_to_send[1]} {msg_to_send[2]}→{rescaled[2]}")
                                msg_to_send = rescaled

                            is_bus_out = output_name.startswith('bus:')
                            if is_bus_out and self._tt_coalesce(output_name, msg_to_send):
                                continue

                            output.send_message(msg_to_send)
                            if is_bus_out:
                                self._note_sent(output_name, msg_to_send)
                        except Exception as e:
                            print(f"⚠️  Failed to send to {output_name}: {e} -- evicting")
                            stale = self.outputs.pop(output_name, None)
                            if stale is not None:
                                try: stale.close_port()
                                except Exception: pass
                                del stale
                            self._last_output_names.discard(output_name)
                            break
            except Exception as e:
                print(f"⚠️  Callback error for {port_name}: {e}")

        return callback

    def _refresh_channel_cache(self):
        """Read the DualSense channel file and update the in-memory cache."""
        try:
            with open(self._channel_file, 'r') as f:
                ch = int(f.read().strip())
                if ch != self._cached_target_channel:
                    print(f"🎛️  Pedal channel updated: {self._cached_target_channel} → {ch}")
                    self._cached_target_channel = ch
        except Exception:
            pass  # file not written yet, keep current cache

    def _er301_status(self, state, message=""):
        st = self._er301
        changed = state != st["state"]
        st["state"] = state
        try:
            import live_config
            live_config.write_json(live_config.STATUS, {
                "state": state, "message": message, "file_mtime": st["file_mtime"], "expected": st["expected"],
                "summary": st["summary"], "reply": st["reply"], "sent_at": st["sent_at"], "time": time.time()})
        except Exception:
            pass
        return changed

    def _er301_send(self, msgs) -> bool:
        """To every bus member: the i2c2midi answers from whichever link it is plugged into; others
        ignore the SysEx (non-commercial ID 0x7D)."""
        for msg in msgs:
            with self.lock:
                outs = [(n, o) for n, o in self.outputs.items() if n.startswith('bus:')]
                if not outs:
                    return False
                for n, out in outs:
                    try:
                        out.send_message(msg)
                    except Exception as e:
                        print(f"⚠️  ER-301 routing send to {n} failed: {e}")
            time.sleep(0.003)
        return True

    def _er301_tick(self):
        """Send the ER-301 routing to the i2c2midi on connect or when er301_routes.py changes, and make sure
        it arrived (see ER301_* above)."""
        if er301_config is None:
            return
        st, now = self._er301, time.time()
        m = er301_config.mtime()
        if m != self._er301_mtime:
            if self._er301_mtime is not None:
                print("📝 er301_routes.py changed")
            self._er301_mtime = m
            self._er301_push_pending = True
            st["tries"] = 0
        connected = any(n.startswith('bus:') for n in self.outputs)

        # an answer to what was sent last?
        rep = self._er301_reply
        if st["state"] == "waiting" and rep is not None and rep[0] >= st["sent_at"]:
            t, r = rep
            self._er301_reply = None
            st["reply"] = r
            got = {k: r[k] for k in ("cv", "nrpn", "triggers", "note_triggers")}
            if st["kind"] == "push" and r["kind"] == "commit":
                if r["status"] != 0:
                    self._er301_status("rejected", r["status_text"])
                    print(f"❌ i2c2midi rejected the ER-301 routing: {r['status_text']}")
                elif got != st["expected"]:
                    self._er301_status("mismatch", f"i2c2midi has {got}, sent {st['expected']}")
                    print(f"⚠️  i2c2midi took the routing but reports {got}, sent {st['expected']}")
                else:
                    if self._er301_status("confirmed"):
                        print(f"✅ i2c2midi confirmed the ER-301 routing ({r['source']}): {st['summary']}")
                st["tries"], st["checked"] = 0, now
            elif st["kind"] == "query" and r["kind"] == "query":
                st["checked"] = now
                if got != st["expected"] or r["source"] == er301_config.SOURCE[0]:
                    print(f"🔁 i2c2midi has another routing ({r['source']}, {got}): sending it again")
                    self._er301_status("outdated", f"i2c2midi had {r['source']}, {got}")
                    self._er301_push_pending = True
                else:
                    self._er301_status("confirmed")
        elif st["state"] == "waiting" and now - st["sent_at"] > self.ER301_ANSWER_WITHIN:
            st["tries"] += 1
            st["next"] = now + self.ER301_RETRY[min(st["tries"] - 1, len(self.ER301_RETRY) - 1)]
            if self._er301_status("no answer", "rack off, or the XIAO link to the i2c2midi is down"):
                print("⚠️  No answer from the i2c2midi (rack off?): the ER-301 routing will be sent again")

        if not connected:
            return
        due = self._er301_push_pending or (st["state"] == "no answer" and now >= st["next"])
        if not due and st["state"] == "confirmed" and now - st["checked"] > self.ER301_CHECK_EVERY:
            st["checked"] = now                        # health check: is it still the one we sent?
            st["kind"], st["sent_at"] = "query", now
            if self._er301_send([er301_config.query_message()]):
                self._er301_status("waiting")
            return
        if not due or st["state"] == "waiting":
            return
        self._er301_push_pending = False
        try:
            msgs, summary = er301_config.build()
        except er301_config.ConfigError as e:
            st["file_mtime"] = m
            self._er301_status("invalid", str(e))
            print(f"⚠️  ER-301 routing NOT sent, er301_routes.py: {e}")
            return
        st.update(expected=er301_config.counts(msgs), summary=summary, kind="push", file_mtime=m,
                  sent_at=time.time(), reply=None)
        self._er301_reply = None
        if not self._er301_send(msgs):
            self._er301_push_pending = True
            return
        self._er301_status("waiting")
        if st["tries"] == 0:
            print(f"🎛️  ER-301 routing sent to i2c2midi: {summary}")

    def monitor_loop(self):
        """Periodically rescan for device changes"""
        print("🔄 Hot-plug monitoring active (logging on change only)")
        while self.running:
            try:
                self._refresh_channel_cache()
                self.scan_and_update_outputs()
                self.scan_and_update_inputs()
                self._er301_tick()
                self._gc_tick = getattr(self, "_gc_tick", 0) + 1
                if self._gc_tick >= 5:   # every ~5s: reclaim GC-delayed rtmidi ALSA clients (load bursts)
                    self._gc_tick = 0
                    gc.collect()
                time.sleep(1)
            except Exception as e:
                print(f"⚠️  Monitor error: {e}")
                time.sleep(1)

    def run(self):
        """Main run loop"""
        print("🎹 MIDI Hub Starting (Hot-Plug Enabled)...")
        print("=" * 50)

        # Initial scan
        self.scan_and_update_outputs()
        self.scan_and_update_inputs()

        # Start monitoring thread
        monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        monitor_thread.start()

        tt_thread = threading.Thread(target=self.tt_flush_loop, daemon=True)
        tt_thread.start()

        print("\n🔄 Ready! Devices can be plugged/unplugged anytime.")
        print("Press Ctrl+C to stop\n")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n👋 Shutting down...")
            self.running = False

            with self.lock:
                # FIX: cancel callbacks before closing on shutdown too
                for midiin in self.inputs.values():
                    try:
                        midiin.cancel_callback()
                        midiin.close_port()
                    except Exception:
                        pass
                for midiout in self.outputs.values():
                    try:
                        midiout.close_port()
                    except Exception:
                        pass

            # Cleanup scanners
            del self.output_scanner
            del self.input_scanner

            print("✅ All ports closed")


def main():
    hub = MIDIHub()
    hub.run()


if __name__ == "__main__":
    main()
