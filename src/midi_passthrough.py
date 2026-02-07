#!/usr/bin/env python3
import rtmidi
import time
import threading

class MIDIHub:
    def __init__(self):
        self.outputs = {}  # {port_name: MidiOut object}
        self.inputs = {}   # {port_name: MidiIn object}
        self.skip_devices = ["f_midi", "NerdSEQ", "Midi Through"]
        self.running = True
        self.lock = threading.Lock()

    def scan_and_update_outputs(self):
        """Find and open all output devices"""
        scanner = rtmidi.MidiOut()
        available = scanner.get_ports()
        del scanner  # Fix memory leak!

        # Find f_midi and NerdSEQ
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
                    try:
                        self.outputs[name].close_port()
                    except:
                        pass
                    del self.outputs[name]

            # Open new outputs
            for name, (port_num, port_name) in found_outputs.items():
                if name not in self.outputs:
                    try:
                        midiout = rtmidi.MidiOut()
                        midiout.open_port(port_num)
                        self.outputs[name] = midiout
                        print(f"✅ Output connected: {port_name}")
                    except Exception as e:
                        print(f"⚠️  Failed to open {port_name}: {e}")

    def scan_and_update_inputs(self):
        """Find and open all input devices"""
        scanner = rtmidi.MidiIn()
        available = scanner.get_ports()
        del scanner  # Fix memory leak!

        # Find devices we should listen to
        found_inputs = {}

        for i, port_name in enumerate(available):
            # Skip output devices and system ports
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
                    try:
                        self.inputs[port_name].close_port()
                    except:
                        pass
                    del self.inputs[port_name]

            # Open new inputs
            for port_name, port_num in found_inputs.items():
                if port_name not in self.inputs:
                    try:
                        midiin = rtmidi.MidiIn()
                        midiin.open_port(port_num)
                        midiin.set_callback(self.make_callback(port_name))
                        self.inputs[port_name] = midiin
                        print(f"✅ Input connected: {port_name}")
                    except Exception as e:
                        print(f"⚠️  Failed to open {port_name}: {e}")

    def make_callback(self, port_name):
        """Create a callback that forwards MIDI to all outputs"""
        def callback(message, data):
            try:
                midi_message, deltatime = message
                print(f"📨 [{port_name}] → {midi_message}")

                with self.lock:
                    if not self.outputs:
                        print(f"⚠️  No outputs available to forward to!")
                    for output_name, output in self.outputs.items():
                        try:
                            output.send_message(midi_message)
                            print(f"   ✓ Sent to {output_name}")
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
                time.sleep(2)  # Check every 2 seconds
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
                for midiin in self.inputs.values():
                    try:
                        midiin.close_port()
                    except:
                        pass
                for midiout in self.outputs.values():
                    try:
                        midiout.close_port()
                    except:
                        pass

            print("✅ All ports closed")

def main():
    hub = MIDIHub()
    hub.run()

if __name__ == "__main__":
    main()
