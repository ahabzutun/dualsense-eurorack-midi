# XIAO MIDI host-to-host bridge

Connects two USB **hosts** over MIDI (e.g. the Raspberry Pi and i2c2midi, or the Pi and
the Percussa SSP). Two Seeed XIAO SAMD21 boards, each plugged into one host as a
class-compliant USB MIDI device, talk to each other over UART.

Based on [marchingband/usb-midi-host2host](https://github.com/marchingband/usb-midi-host2host), hardened:

- every 4-byte USB-MIDI packet is validated before it is forwarded; invalid bytes are
  dropped one at a time so the framing resyncs by itself
- a partial packet older than 1 ms is discarded
- internal pull-up on RX, so an unpowered partner board can't inject noise
- 250000 baud UART
- channel voice, SysEx (any length), song position/select, MTC quarter frame,
  tune request and clock/start/continue/stop/active sensing pass through
- LED pulses once per forwarded packet

## Wiring

```
XIAO A  TX (D6) ──► RX (D7)  XIAO B
XIAO A  RX (D7) ◄── TX (D6)  XIAO B
XIAO A  GND ─────── GND      XIAO B
```

Both boards are powered by the host they are plugged into. Use USB cables that carry data.

## Build / flash

Both boards of a pair need the same environment (baud rate and USB name must match).

| Env        | USB name   | Link                         |
|------------|------------|------------------------------|
| `xiao_tt`  | `XIAO-TT`  | Pi ⇄ i2c2midi (Teletype)     |
| `xiao_ssp` | `XIAO-SSP` | Pi ⇄ Percussa SSP            |

```
pio run -e xiao_tt  -t upload --upload-port /dev/cu.usbmodemXXXX
pio run -e xiao_ssp -t upload --upload-port /dev/cu.usbmodemXXXX
```

`midi_passthrough.py` routes by these USB names.
