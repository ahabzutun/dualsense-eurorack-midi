"""
MIDI Passthrough Hub for DualSense Eurorack system
Routes MIDI between DualSense virtual port, NerdSEQ, and Percussa SSP

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
import time
import threading


class MIDIHub:
    def __init__(self):
        self.outputs = {}
        self.inputs = {}
        self.skip_devices = ["f_midi", "Midi Through"]
        self.running = True
        self.lock = threading.Lock()

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
            print(f"⚠️  Cannot scan outputs: {e}")
            return

        found_outputs = {}

        for i, port_name in enumerate(available):
            if "f_midi" in port_name:
                found_outputs['SSP'] = (i, port_name)
            elif "NerdSEQ" in port_name:
                found_outputs['NerdSEQ'] = (i, port_name)
            elif "USB MIDI" in port_name and 'NerdSEQ' not in found_outputs:
                # NerdSEQ sometimes enumerates with generic "USB MIDI" name
                # instead of "XOR NerdSEQ" after reboot. Accept as fallback
                # only if a proper NerdSEQ port hasn't already been found.
                found_outputs['NerdSEQ'] = (i, port_name)

        current_names = set(found_outputs.keys())
        changed = current_names != self._last_output_names

        with self.lock:
            # Close outputs that are no longer available
            for name in list(self.outputs.keys()):
                if name not in found_outputs:
                    print(f"❌ Output disconnected: {name}")
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
                        print(f"✅ Output connected: {port_name}")
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
            print(f"⚠️  Cannot scan inputs: {e}")
            return

        found_inputs = {}

        for i, port_name in enumerate(available):
            should_skip = any(skip in port_name for skip in self.skip_devices)
            if "RtMidi output" in port_name and "DualSense_Controller" not in port_name:
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

    def make_callback(self, port_name):
        """Create a callback that forwards MIDI to all outputs.

        Routing rules:
          NerdSEQ input  → SSP only  (avoid echo loop back to NerdSEQ itself)
          Everything else → all outputs (SSP + NerdSEQ)
        """
        is_nerdseq_source = "NerdSEQ" in port_name or "USB MIDI" in port_name
        is_pedal_source   = "CH345" in port_name or "DOREMiDi" in port_name

        def callback(message, data):
            try:
                midi_message, deltatime = message            

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
                        # NerdSEQ input must NOT be echoed back to NerdSEQ output
                        if is_nerdseq_source and output_name == 'NerdSEQ':
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

                            output.send_message(msg_to_send)
                        except Exception as e:
                            print(f"⚠️  Failed to send to {output_name}: {e}")
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

    def monitor_loop(self):
        """Periodically rescan for device changes"""
        print("🔄 Hot-plug monitoring active (logging on change only)")
        while self.running:
            try:
                self._refresh_channel_cache()
                self.scan_and_update_outputs()
                self.scan_and_update_inputs()
                time.sleep(2)
            except Exception as e:
                print(f"⚠️  Monitor error: {e}")
                time.sleep(2)

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
