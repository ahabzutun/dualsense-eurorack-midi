#!/usr/bin/env python3
import rtmidi

# List available MIDI outputs
midiout = rtmidi.MidiOut()
available_ports = midiout.get_ports()

print("Available MIDI outputs:")
for i, port in enumerate(available_ports):
    print(f"  {i}: {port}")

if not available_ports:
    print("\nNo MIDI devices found!")
    exit(1)

# Find NerdSEQ port
nerdseq_port = None
for i, port in enumerate(available_ports):
    if "NerdSEQ" in port:
        nerdseq_port = i
        break

if nerdseq_port is None:
    print("\nNerdSEQ not found! Using port 0 instead.")
    nerdseq_port = 0

print(f"\nOpening: {available_ports[nerdseq_port]}")
midiout.open_port(nerdseq_port)

# Send a test note (Middle C, velocity 100)
print("Sending MIDI note C4 (60)...")
note_on = [0x90, 60, 100]  # Note On, channel 1, note 60, velocity 100
midiout.send_message(note_on)

input("Press Enter to stop the note...")

note_off = [0x80, 60, 0]   # Note Off
midiout.send_message(note_off)

print("Note stopped!")
del midiout
