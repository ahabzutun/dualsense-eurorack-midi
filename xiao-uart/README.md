# XIAO links over the Pi's UARTs

Four XIAO SAMD21 boards hang directly off the Pi 4's UARTs instead of USB.

| XIAO        | Pi device      | GPIO (TX/RX) | Role                                   |
|-------------|----------------|--------------|----------------------------------------|
| mysteryman  | /dev/ttyAMA0   | 14 / 15      | MIDI bus link (any device)             |
| jumpingman  | /dev/ttyAMA3   | 4 / 5        | MIDI bus link (any device)             |
| nadine      | /dev/ttyAMA4   | 8 / 9        | MIDI bus link (any device)             |
| loglady     | /dev/ttyAMA5   | 12 / 13      | Teletype only: grid + keyboard over USB |

Wiring: XIAO D6 (TX) → Pi RX, XIAO D7 (RX) → Pi TX, plus GND.

## Boot config

`/boot/firmware/config.txt`:

```
dtoverlay=miniuart-bt
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5
```

Remove `console=serial0,115200` from `/boot/firmware/cmdline.txt`.

## Files

- `xiao_uart_bridge.py`: one ALSA MIDI port per enabled link in `links.json` (250000 baud,
  4-byte USB-MIDI packets). The ALSA client name is the XIAO's name.
- `links.json`: the links. loglady is `enabled: false` here because her own service owns ttyAMA5.
- `gpio_gate.py`: keeps the Pi's TX pin from powering an unplugged XIAO. When RX stays low for
  0.3 s, TX becomes an input with pull-down until the XIAO is back.
- `loglady_teletype.py`: COBS + CRC8 frames at 1 Mbaud to loglady
  (firmware: `firmware/loglady-teletype`). Gives `/run/loglady/grid` (pty for grid-macros),
  `/run/loglady/kbd` (fifo for keyboard-forward), `/run/loglady/ctl` (`replug`, `status`) and
  writes `/run/loglady/status.json`.
- `systemd/`: the two services plus the drop-ins that point grid-macros and keyboard-forward at
  loglady, switch off the old USB gadget and tie midi-passthrough to the bridge.

The MIDI hub (`src/midi_passthrough.py`) treats every link as a bus member: whatever arrives on
one link or controller goes to all the others, never back to where it came from.

## Install on the Pi

```
mkdir -p ~/xiao-uart && cp *.py links.json ~/xiao-uart/
sudo cp -r systemd/* /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl disable --now teletype-gadget teletype-hid-module
sudo systemctl enable --now xiao-uart-bridge loglady-teletype
sudo systemctl restart midi-passthrough grid-macros keyboard-forward
```

Firmware: `firmware/xiao-midi-bridge` (envs `mysteryman`, `jumpingman`, `nadine`) and
`firmware/loglady-teletype` (env `loglady_tt`).
