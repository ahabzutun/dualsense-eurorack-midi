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
        self.skip_devices = ["f_midi", "NerdSEQ", "Midi Through"]
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
        self.SSP_CC_MAX       = 80          # 0-127 in → 0-80 to SSP
        # ─────────────────────────────────────────────────────────────────────

        # Create scanners ONCE and reuse them (never re-create these)
        self.output_scanner = rtmidi.MidiOut()
        self.input_scanner = rtmidi.MidiIn()

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
                print(f"🔍 Found f_midi at port {i}: {port_name}")
            elif "NerdSEQ" in port_name:
                found_outputs['NerdSEQ'] = (i, port_name)
                print(f"🔍 Found NerdSEQ at port {i}: {port_name}")

        print(f"🎯 Current outputs in memory: {list(self.outputs.keys())}")
        print(f"🎯 Found outputs to open: {list(found_outputs.keys())}")

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
                print(f"🔍 Found input: {port_name}")

        with self.lock:
            # Close inputs that are no longer available
            for port_name in list(self.inputs.keys()):
                if port_name not in found_inputs:
                    print(f"❌ Input disconnected: {port_name}")
                    midiin = self.inputs.pop(port_name)
                    try:
                        # FIX: cancel_callback() FIRST to break the reference
                        # cycle (MidiIn → closure → self.outputs → MidiIn).
                        # Without this the C++ ALSA client is never freed.
                        midiin.cancel_callback()
                        midiin.close_port()
                    except Exception:
                        pass
                    del midiin   # allow C++ destructor to run immediately

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
                        # FIX: also cancel callback on failed open before del
                        try:
                            midiin.cancel_callback()
                        except Exception:
                            pass
                        del midiin

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

    def make_callback(self, port_name):
        """Create a callback that forwards MIDI to all outputs"""
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

                # ── Normal mode: forward to all outputs ──────────────────────
                with self.lock:
                    if not self.outputs:
                        print(f"⚠️  No outputs available to forward to!")
                    for output_name, output in self.outputs.items():
                        try:
                            # SSP gets rescaled 16n values; NerdSEQ gets originals
                            if self.RESCALE_ENABLED and output_name == 'SSP':
                                msg_to_send = self.rescale_for_ssp(midi_message)
                                if msg_to_send[2] != midi_message[2]:
                                    print(f"🎚️  [16n→SSP] CC{midi_message[1]} {midi_message[2]}→{msg_to_send[2]}")
                            else:
                                msg_to_send = midi_message

                            output.send_message(msg_to_send)
                        except Exception as e:
                            print(f"⚠️  Failed to send to {output_name}: {e}")
            except Exception as e:
                print(f"⚠️  Callback error for {port_name}: {e}")

        return callback

    def monitor_loop(self):
        """Periodically rescan for device changes"""
        print("🔄 Starting hot-plug monitor (rescanning every 2 seconds)...")
        while self.running:
            try:
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
