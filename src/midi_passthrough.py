#!/usr/bin/env python3
import rtmidi
import time
import sys



def main():
    print("🎹 MIDI Passthrough Starting...")
    print("=" * 50)

    # Create virtual MIDI input port
    midiin = rtmidi.MidiIn()
    midiin.open_virtual_port("MIDI_Passthrough_In")
    print("✅ Created virtual input: MIDI_Passthrough_In")

    # Find and open NerdSEQ output
    midiout = rtmidi.MidiOut()
    ports = midiout.get_ports()

    nerdseq_port = None
    for i, port in enumerate(ports):
        if "NerdSEQ" in port:
            nerdseq_port = i
            break

    if nerdseq_port is None:
        print("❌ NerdSEQ not found! Available ports:")
        for port in ports:
            print(f"   - {port}")
        sys.exit(1)

    midiout.open_port(nerdseq_port)
    print(f"✅ Output to: {ports[nerdseq_port]}")

    print("\n🔄 Forwarding MIDI messages...")
    print("Press Ctrl+C to stop\n")

    # Callback function that runs when MIDI is received
    def midi_callback(message, data):
        midi_message, deltatime = message
        print(f"📨 Forwarding: {midi_message}")  # Add this line!
        midiout.send_message(midi_message)

    # Attach the callback to our input
    midiin.set_callback(midi_callback)

    try:
        # Keep running until Ctrl+C
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n👋 Shutting down MIDI passthrough...")
    finally:
        # Clean up
        midiin.close_port()
        midiout.close_port()
        print("✅ Ports closed cleanly")

if __name__ == "__main__":
    main()
