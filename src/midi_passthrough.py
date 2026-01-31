#!/usr/bin/env python3
import rtmidi
import time

def main():
    print("🎹 MIDI Hub Starting...")
    print("=" * 50)

    # ===== STEP 1: Find and open the OUTPUT (f_midi USB gadget) =====
    midiout = rtmidi.MidiOut()
    ports_out = midiout.get_ports()

    f_midi_port = None
    for i, port in enumerate(ports_out):
        if "f_midi" in port:
            f_midi_port = i
            break

    if f_midi_port is None:
        print("❌ f_midi (USB gadget) not found!")
        print("Available ports:", ports_out)
        return

    midiout.open_port(f_midi_port)
    print(f"✅ Output: {ports_out[f_midi_port]} (USB Gadget)\n")

    # ===== STEP 2: Find ALL available INPUTS =====
    scanner = rtmidi.MidiIn()
    available_ports = scanner.get_ports()

    print("🔍 Scanning for MIDI inputs...")
    for i, port in enumerate(available_ports):
        print(f"   {i}: {port}")

# ===== STEP 3: Filter which ports to listen to =====
    ports_to_listen = []

    # TODO: GADGET MODE - When Pi is in gadget mode with multiple outputs (SSP, NerdSEQ, etc.):
    # - Remove this NerdSEQ input filter
    # - Implement routing logic to send different inputs to different outputs
    # - Example: NerdSEQ output → SSP, DualSense → both NerdSEQ and SSP

    # Current simple mode: Avoid feedback loop by skipping output devices as inputs
    output_device_name = "f_midi"  # TODO: Make this configurable

    for i, port in enumerate(available_ports):
        # Skip "Midi Through" system port (always skip this)
        if "Midi Through" in port:
            print(f"⏭️  Skipping: {port}")
            continue

        # Skip output device to avoid feedback (TODO: Remove when implementing routing)
        if output_device_name in port:
            print(f"⏭️  Skipping: {port} (output device - avoiding feedback)")
            continue

        # Skip generic RtMidi outputs (self-created ports) but allow DualSense_Controller
        if "RtMidi output" in port and "DualSense_Controller" not in port:
            print(f"⏭️  Skipping: {port} (internal RtMidi port)")
            continue

        # Add everything else to our listen list
        ports_to_listen.append((i, port))
        print(f"✅ Will listen to: {port}")

    print()  # Blank line

    # ===== STEP 4: Create MidiIn object for EACH port =====
    input_ports = []  # Store all MidiIn objects here

    for port_num, port_name in ports_to_listen:
        # Create a NEW MidiIn for each input port
        midiin = rtmidi.MidiIn()
        midiin.open_port(port_num)

        # Create callback that forwards to NerdSEQ
        def make_callback(name):
            def callback(message, data):
                midi_message, deltatime = message
                print(f"📨 [{name}] → {midi_message}")
                midiout.send_message(midi_message)
            return callback

        midiin.set_callback(make_callback(port_name))

        # IMPORTANT: Store this MidiIn object!
        input_ports.append(midiin)

    print(f"🔄 Listening to {len(input_ports)} inputs, forwarding to f_midi (USB Gadget)...")
    print("Press Ctrl+C to stop\n")

    # ===== STEP 5: Keep running =====
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down...")
    finally:
        # Clean up all ports
        for midiin in input_ports:
            midiin.close_port()
        midiout.close_port()
        print("✅ All ports closed")

if __name__ == "__main__":
    main()
