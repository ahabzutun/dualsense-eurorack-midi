# DualSense → Eurorack MIDI

A Raspberry Pi 4 sits in the middle of a eurorack rig and turns a PS5 DualSense (on a USB cable) into an
instrument, adds a few performance functions (grain-slice selection, a loop recorder, freeze, motion control,
strum) and acts as a MIDI hub for the other controllers (16n faderbank, Faderfox EC4, DOREMiDi MPC-20 pedals).
The MIDI goes to USB-host modules through small XIAO bridges — so far an **i2c2midi** (→ ER-301 and Teletype), a
**Percussa SSP** and a **NerdSEQ**; none of them is required. **Kontrolleur**, a desktop app for macOS and Linux,
configures all of it from one place and shows where every signal goes.

```
                         ┌──────────────── Raspberry Pi 4 ("ahab") ────────────────┐
 DualSense ─ USB cable ─▶│ dualsense_midi.py ──▶ port DualSense_Controller ──┐     │
 16n ──────── USB ───────┤        ▲ (own ports for the Pi functions)          ▼     │
 MPC-20 ─── USB (CH345) ─┤────────┴──────────────────────────────▶ midi_passthrough.py (MIDI hub)
 EC4 ──────── USB ───────┘   (each controller optional)               │       │        │
                                                                      │       │        └─▶ NerdSEQ (optional)
                         XIAO-TT ⇄ XIAO ─▶ i2c2midi ─ I2C ─▶ ER-301 ◀─┘       │
                                              ▲   (Teletype leads the bus)     │
                                              └─ Teletype I2M ch 17–32 ─▶ back into the hub
                         XIAO-SSP ⇄ XIAO ─▶ Percussa SSP ◀──────────────────────┘
                         (each target optional; XIAO pairs = modified usb-midi-host2host)

 Kontrolleur (Mac / Fedora) ── SSH ──▶ Pi: controls.py, er301_routes.py, device tools
                            ── SSH ──▶ SSP: reads the MIDI mappings of its patches
```

## Contents

1. [What you need](#what-you-need)
1. [What runs where](#what-runs-where)
2. [The repository](#the-repository)
3. [The DualSense script](#the-dualsense-script)
4. [The MIDI hub](#the-midi-hub)
5. [ER-301 routing through the i2c2midi](#er-301-routing-through-the-i2c2midi)
6. [Kontrolleur](#kontrolleur)
7. [Device tools](#device-tools)
8. [Percussa SSP](#percussa-ssp)
9. [Firmware](#firmware)
10. [Setting up the Pi](#setting-up-the-pi)
11. [Day to day](#day-to-day)
12. [Things worth knowing](#things-worth-knowing)
13. [History](#history)

## What you need

| | Needed? | Notes |
|---|---|---|
| Raspberry Pi 4 | yes | Raspberry Pi OS Lite 64-bit (Debian 13 "trixie" runs here) |
| PS5 DualSense + USB data cable | yes | plugged into the Pi; no Bluetooth pairing |
| A computer for Kontrolleur | recommended | macOS or Linux; everything can also be edited by hand |
| Powered USB hub | with several devices | the Pi's ports don't power many devices at once |
| A pair of Seeed XIAO SAMD21 per USB-host target | per target | the i2c2midi and the SSP are USB *hosts* like the Pi, so each link is two XIAOs joined by UART ([firmware](#firmware)) |
| i2c2midi MK2 with *ER-301 direct* + ER-301 | optional | for the ER-301 routing and the trigger LFO; a Teletype on the same bus is optional too |
| Percussa SSP | optional | gets the hub's MIDI through its own XIAO pair |
| NerdSEQ | optional | gets the hub's MIDI when plugged into the Pi |
| 16n · Faderfox EC4 · MPC-20 pedals (via DOREMiDi) | optional | extra controllers; any can drive the Pi functions or the targets |

No power/data splitter is needed: that was for the old USB-gadget setup (see [History](#history)).

## What runs where

| Where | What | Started by |
|---|---|---|
| Raspberry Pi | `src/dualsense_midi.py` — reads the DualSense, runs the Pi functions, sends MIDI on its own port `DualSense_Controller` | `dualsense-midi.service` |
| Raspberry Pi | `src/midi_passthrough.py` — the MIDI hub between every USB MIDI device | `midi-passthrough.service` |
| XIAO SAMD21 pairs | `firmware/xiao-midi-bridge`, a modified [usb-midi-host2host](https://github.com/marchingband/usb-midi-host2host) — USB host ⇄ USB host bridges Pi ⇄ i2c2midi (`XIAO-TT`) and Pi ⇄ SSP (`XIAO-SSP`) | power from the hosts |
| i2c2midi (Teletype expander) | i2c2midi firmware with *ER-301 direct* (not in this repo): MIDI → SC.CV / SC.TR straight over its own I2C bus, plus a trigger LFO | — |
| Percussa SSP | `scripts/ssp_mdns.py` (`ssp.local`) and `scripts/ssp_synthor_log.py` (which patch is loaded) | the SSP's init scripts |
| Mac / Linux | `kontrolleur/` — the configuration app | you |

Both Pi services restart themselves when they fail (`Restart=always`) and handle devices being plugged in and
out while they run. Their settings files are re-read while running, so a change from Kontrolleur needs no restart.

## The repository

| Path | What it is |
|---|---|
| `src/dualsense_midi.py` | the DualSense script (main loop, evdev reading, MIDI out) |
| `src/midi/` | `button_router.py` (tap / hold / chords → actions), `axis_router.py` (axes → functions or own CC), `device_inputs.py` (16n / EC4 / pedals → the same functions), `controller.py` (MIDI, grain slice, LEDs), `dualsense_hid.py` (light bar, dots, rumble over hidraw), `harmonic_strummer.py` |
| `src/state/` | `channel_manager.py` (instruments 1–3, freeze), `loop.py` (loop recorder), `freeze.py`, `clock_source.py` (tempo for the loop recorder: a NerdSEQ's MIDI clock if one is plugged in, else 120 BPM) |
| `src/controls_config.py` | reads and checks `controls.py` |
| `~/.config/dualsense-eurorack-midi/controls.py` (on the Pi) | what each DualSense button / axis does, and which 16n / pedal / EC4 messages run Pi functions — written by Kontrolleur |
| `~/.config/dualsense-eurorack-midi/er301_routes.py` (on the Pi) | which MIDI goes to which ER-301 port, pitch and LFO settings — written by Kontrolleur |
| `src/config/controls.py`, `src/config/er301_routes.py` | the defaults of a fresh install; moved to `~/.config/…` the first time (`src/live_config.py`), so `git pull` never touches your mappings |
| `src/config/mappings.py` | CC / NRPN numbers of the DualSense controls, instrument channels, timing constants |
| `src/midi_passthrough.py` | the MIDI hub |
| `src/er301_config.py` | turns `er301_routes.py` into the SysEx the i2c2midi stores |
| `src/osc_out.py`, `src/osc_map.conf` | OSC tap of the hub: mapped MIDI → OpenMW on the Mac (UDP) |
| `scripts/` | device tools Kontrolleur runs over SSH (or locally on a cable), SSP helpers, `er301_push.py` |
| `kontrolleur/` | the desktop app ([its own README](kontrolleur/README.md)) |
| `firmware/xiao-midi-bridge/` | firmware of the XIAO USB host-to-host bridges (modified [usb-midi-host2host](https://github.com/marchingband/usb-midi-host2host)) |
| `install-pi.sh` | sets up a Pi: packages, udev rules, groups, both services for your user and checkout |
| `systemd/`, `udev/` | service files and udev rules for the Pi (`install-pi.sh` installs them) |
| `test/` | tests of the routers, device inputs and passthrough (`python3 test/test_*.py`) plus a few interactive helpers |
| `backups/` | configuration dumps of the 16n, EC4 and MPC-20 |
| `src/doremidi_bridge.py`, `DOREMIDI_DEBUG_NOTES.md` | an earlier libusb bridge for the MPC-20, not used any more (see [History](#history)) |

## The DualSense script

`dualsense_midi.py` reads the DualSense's three input devices (gamepad, touchpad, motion sensors) over the USB
cable with evdev and writes light bar, player dots and rumble straight to its hidraw device. It waits for the
controller and picks it up again after it is unplugged. Everything it sends leaves through one
ALSA port, `DualSense_Controller`, which the hub picks up.

**How a control is handled.** A button goes to the button router: tap, hold (≥ 1 s) or a two-button chord runs an
action, or the button sends its own CC (127 pressed / 0 released). Sticks, triggers, touchpad, d-pad and motion go
to the axis router (sticks and triggers through the freeze first): a function, or the control's own CC (motion and
the grain slice as 14-bit NRPN). 16n, EC4 and pedal messages listed in `MIDI_INPUTS` reach the same functions
through the script's own input ports (`DS 16n In`, `DS EC4 In`, `DS Pedal In`): a CC drives a continuous function
with its value or fires an action when it reaches 64; a note on fires an action.

**Layout** (as used here; change it in Kontrolleur's DualSense tab):

| Control | Does |
|---|---|
| ✕ ○ △ □ | their own CC |
| L1 · R1 · R1 hold | loop delete · loop play / stop · loop record |
| L1 + R1 | motion control on / off (tilt and twist as 14-bit NRPN) |
| L3 · R3 | freeze L2, R2 and the left stick · freeze the right stick |
| Create · Options · both | sample block − · block + · back to block 1 |
| Left stick X · d-pad ↔ | slice in the section · section − / + |
| Touchpad X | loop scrub |
| PS | harmonic strum (notes around the d-pad step) |
| everything else | its own CC |

**Grain slice.** Slice 0–1023 = block × 128 + section × 16 + slice, sent as one 14-bit value (NRPN 3).

**Instruments.** The script knows three instruments with their own freeze and loop; instrument 1 sends on MIDI
ch 2, 2 on ch 13, 3 on ch 14 (`INSTRUMENT_MIDI_CHANNELS`). No control switches between them at the moment. The
current channel is written to `/tmp/dualsense_channel` for the hub.

**Feedback on the controller.**

| | Shows |
|---|---|
| Light bar | dim white when idle · red pulse while recording · purple pulse while overdubbing · white pulse while a loop plays · blue while motion control is on |
| Player dots (under the touchpad) | the sample block: a dot walking from left to right, two neighbours in between; the first and last block light a single outer dot |
| Short rumble | block − (left) / block + (right) · section − / + on the d-pad · freezing (both motors for L3, right for R3) |
| No rumble | Create / Options or the d-pad past the first or last block: nothing changes, nothing is sent |
| Rumble while tilting | with motion control on, near the ends of the tilt range |

## The MIDI hub

`midi_passthrough.py` subscribes to every MIDI input it finds (the DualSense script, 16n, EC4, the pedals, the
XIAO-TT, a NerdSEQ, an iii grid) and sends each message to every output that is plugged in: **XIAO-TT**
(→ i2c2midi), **XIAO-SSP** (→ SSP), a **NerdSEQ**. Whatever is missing is skipped; devices can come and go while
it runs. On the way:

| Rule | Why |
|---|---|
| Nothing goes back to where it came from (XIAO-TT input ↛ XIAO-TT, NerdSEQ ↛ NerdSEQ) | no loops through the i2c2midi's MIDI-in store |
| Pedal messages on ch 1 move to the DualSense's channel (`/tmp/dualsense_channel`) | the pedals play the current instrument |
| CC / pressure / pitch bend to the XIAO-TT are thinned to the latest value per controller every 10 ms; ch 2 and notes go straight through | keeps the XIAO serial link and the i2c2midi clear; ch 2 is the timing-critical DualSense instrument |
| `er301_routes.py` is sent to the i2c2midi as SysEx whenever the XIAO-TT connects or the file changes; the hub waits for the i2c2midi's answer (accepted + route counts), sends it again until one comes (rack off), and checks every minute that it's still the one sent | the i2c2midi keeps the routing (EEPROM); its last answer is in `~/.config/dualsense-eurorack-midi/i2c2midi_status.json` and shown in Kontrolleur |
| 16n → SSP: values can be scaled down (`SSP_CC_MAX`) | at 127 (now) nothing changes |
| iii grid → NerdSEQ: no ch 1, no notes | a separate grid bridge handles those |
| Every message also goes to `osc_out.py` → OpenMW (`osc_map.conf`, hot-reloaded) | adds no MIDI port |

What the i2c2midi sends on its USB channels (Teletype `I2M.CH 17–32`, i.e. USB ch 1–16) comes back in through the
XIAO-TT and reaches the SSP and the NerdSEQ.

## ER-301 routing through the i2c2midi

The ER-301 isn't driven by Teletype scripts for this: the i2c2midi runs a firmware with *ER-301 direct*, which
turns incoming USB MIDI into SC.CV / SC.TR values on its own I2C bus. `er301_routes.py` says what goes
where:

| List | Meaning |
|---|---|
| `CV` | a DualSense control or `(channel, CC, port)` → SC.CV, 0–127 → 0–10 V |
| `NRPN_CV` | 14-bit NRPN → SC.CV in full resolution (e.g. the grain slice) |
| `TRIGGERS` / `NOTE_TRIGGERS` | CC ≥ 64 or a held note → gate on a port |
| `PITCH` | notes on the DualSense channel → V/Oct on one port |
| `LFO` | the i2c2midi's trigger LFO: port, rate and reset controls, Hz range |

A route belongs to a channel + CC, not to a device: every controller that sends that channel and CC drives the
port (Kontrolleur's ER-301 tab lists them all and links to each). `scripts/er301_push.py` sends the routing by hand
(`--query` asks the i2c2midi what it's using). The Teletype stays the I2C leader for its own scripts.

## Kontrolleur

The desktop app in `kontrolleur/` (Python + PySide6, macOS and Linux) with one tab per device: DualSense, 16n,
Pedals, EC4, Routing, ER-301 and Teletype. It loads the current state from the Pi (and from the devices, over SSH or on a
cable), lets you change it and uploads it. The **Routing** tab draws the whole rig — controllers, the Pi with the
DualSense script and the hub (*Scripts* and *Stages* open them up), the i2c2midi, ER-301, Teletype and the SSP's
loaded patch — and traces a single control through all of it, hop by hop. The **Teletype** tab reads the
Teletype's scenes from a USB stick (its WRITE TO USB), shows what each script line sends through the i2c2midi and
where it lands, flags mistakes like a TRS channel, lets you edit scripts and write them back for READ FROM USB,
and shows live what actually arrives from the i2c2midi at the Pi.

Install and details: [kontrolleur/README.md](kontrolleur/README.md). In short: `kontrolleur/install-mac.sh` or
`kontrolleur/install-linux.sh`, then set the SSH host (`ahab@ahab.local`) in the app.

## Device tools

Kontrolleur runs these on the Pi over SSH (from the Pi's checkout, `python3 ~/dualsense-eurorack-midi/scripts/…`),
locally when the device is on a cable to the computer, and `ssp_tool.py` on the SSP. They also work on their own.

| Script | Device | Does |
|---|---|---|
| `er301_routes_tool.py` | Pi | exports / writes `controls.py` and `er301_routes.py`, checks them; refuses to overwrite a file that changed since it was loaded (`--expect`); keeps the last EC4 dump for every computer (`shared-get` / `shared-put`) |
| `er301_push.py` | i2c2midi | sends the ER-301 routing as SysEx, shows the i2c2midi's answer |
| `fader16_tool.py` | 16n | reads / writes channels, CCs, 14-bit mode, curves and output ranges (SysEx) |
| `mpc20_tool.py` | MPC-20 | reads / writes the pedal messages (pyusb, needs `udev/99-doremidi.rules`) |
| `ec4_tool.py` | EC4 | receives / sends setups as SysEx dumps (transfers are started on the EC4) |
| `ssp_tool.py` | SSP | scans the presets: MIDI modules, MIDI learn mappings, device names, the loaded patch |
| `midi_monitor.py` | Pi | streams what arrives on one MIDI input (default `XIAO-TT`) as JSON lines, alongside the hub (Teletype tab → Live) |
| `rtmidi_compat.py` | — | the bit of python-rtmidi the tools need, on top of alsa-midi (for Pythons without python-rtmidi) |
| `deploy_controls_to_pi.sh` | Pi | the one-off rollout of the button router (kept for reference; updates now go through git) |

## Percussa SSP

The SSP gets its MIDI from the hub through the XIAO-SSP bridge (its patches select **XIAO-SSP** as MIDI device).
Kontrolleur reads the SSP's presets over SSH (`scripts/ssp_tool.py`) to show which module listens to what and which
controller reaches it. Two helpers run on the SSP itself:

- `ssp_mdns.py` (as `/root/ssp_mdns.py`, started by `/etc/init.d/S85mdns`) answers `ssp.local`, so the SSP is
  reachable over Wi-Fi as well as Ethernet.
- `ssp_synthor_log.py` (as `/root/synthor_log.py`, started from `/etc/init.d/S90synthor` instead of
  `synthor.sh`) runs SYNTHOR in a pty and keeps its preset lines in `/tmp/synthor-presets.log`, so Kontrolleur
  can follow the loaded patch.

Good to know: TheTechnobear's modules (PMIX, MMX4 …) match a MIDI mapping by CC number only; the channel they
listen on is the module's own MIDI channel setting (0 = every channel).

## Firmware

- **XIAO MIDI bridge** (`firmware/xiao-midi-bridge`, PlatformIO, Seeed XIAO SAMD21): a modified version of
  [marchingband/usb-midi-host2host](https://github.com/marchingband/usb-midi-host2host) — packet validation and
  resync, 250 000 baud, own USB names. Two boards per link, each a USB MIDI device on one host, joined by UART
  (TX↔RX, GND). Environments `xiao_tt` (`XIAO-TT`) and `xiao_ssp` (`XIAO-SSP`); the hub routes by these names.
  Wiring and flashing in [its README](firmware/xiao-midi-bridge/README.md).
- **i2c2midi** (MK2, Teensy 4.1) with *ER-301 direct* — needed for the ER-301 routing and the LFO; not in this
  repo.
- **16n** (16nLC) with per-fader curves and output ranges — needed for those two settings in Kontrolleur; not in
  this repo. A stock 16n works, Kontrolleur then skips curves and ranges.

## Setting up the Pi

1. **OS.** Flash Raspberry Pi OS Lite (64-bit) with the Raspberry Pi Imager; set a hostname, a user and SSH there.
2. **Clone and install.** On the Pi:
   ```bash
   sudo apt install -y git
   git clone https://github.com/ahabzutun/dualsense-eurorack-midi.git ~/dualsense-eurorack-midi
   cd ~/dualsense-eurorack-midi && ./install-pi.sh
   ```
   `install-pi.sh` installs the packages (`python3-rtmidi`, `python3-evdev`, `python3-usb`, `alsa-utils`), the
   udev rules, adds you to the `input`, `audio` and `plugdev` groups and installs both services for your user and
   this checkout — the unit files in `systemd/` name user `ahab`, the script fills in yours. `--dry-run` shows
   what it would do; `--no-services` stops before the services. Running it again is safe.
3. **Plug things in.** The DualSense on a USB data cable, the XIAO bridges and the controllers you have (through
   a powered hub). The services find everything by name whenever it appears.
4. **Kontrolleur.** On the computer: `ssh-copy-id <user>@<hostname>.local`, then install Kontrolleur
   ([its README](kontrolleur/README.md)) and set that host in the app.

<details><summary>The same by hand</summary>

```bash
sudo apt update && sudo apt install -y git alsa-utils python3-rtmidi python3-evdev python3-usb
git clone https://github.com/ahabzutun/dualsense-eurorack-midi.git ~/dualsense-eurorack-midi
cd ~/dualsense-eurorack-midi
sudo cp udev/70-dualsense.rules udev/99-mpc20-midi.rules udev/99-doremidi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG input,audio,plugdev "$USER"
# edit User= and the paths in systemd/*.service if you aren't ahab in /home/ahab, then:
sudo cp systemd/dualsense-midi.service systemd/midi-passthrough.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now dualsense-midi midi-passthrough
```
</details>

## Day to day

```bash
# update the Pi (here: user ahab on ahab.local)
ssh ahab@ahab.local 'cd ~/dualsense-eurorack-midi && git pull && sudo systemctl restart dualsense-midi midi-passthrough'

# logs
journalctl -u dualsense-midi -f
journalctl -u midi-passthrough -f

# what is connected to what
aconnect -l
aseqdump -p XIAO-TT        # watch what arrives from the i2c2midi

# tests (on the Pi or any computer with the packages)
python3 test/test_button_router.py && python3 test/test_axis_router.py && python3 test/test_device_inputs.py
```

Kontrolleur: `git pull` in the repository and restart the app (the macOS app and the Linux launcher run the code
from the repository).

## Things worth knowing

- **Teletype → SSP.** `I2M.CH 17–32` sends on the i2c2midi's USB port (17 = USB ch 1). `I2M.CH` is kept by the
  Teletype until changed; set in the I script it only takes effect once the I script has run (scene load or F10).
  `I2M.N# ch …` / `I2M.CC# ch …` carry the channel in the command.
- **Channel clashes.** ER-301 routes and SSP mappings match channel + CC. Two controllers sending the same
  pair (e.g. a pedal on ch 1, moved to ch 2 by the hub, and an EC4 encoder on ch 2) both drive the target.
- **XIAO link.** The bridges run at 250 000 baud with packet validation (`xiao_tt_115k` is a 115 200-baud
  fallback for a marginal wire). If nothing arrives from the i2c2midi, `er301_push.py --query` tests the link in
  both directions: it asks the i2c2midi for its routing and waits for the answer.
- **MPC-20.** It shows up as a CH345 USB MIDI device; `99-mpc20-midi.rules` makes Linux create its MIDI port
  reliably. The hub moves its channel 1 to the DualSense channel.

## History

Earlier versions of the project worked differently; older notes and commits describe these setups:

- **NerdSEQ as the centre.** The project started with a NerdSEQ as central sequencer and its MIDI clock driving
  a quantized loop recorder. Today the Teletype (with a monome grid) leads, and the NerdSEQ is just one optional
  target of the hub.
- **The Pi as a USB MIDI gadget.** The Pi used to appear to the SSP as a USB MIDI device (gadget mode, with a
  power/data splitter on the Pi's USB-C port). Its MIDI device disappeared from the SSP whenever the Pi
  rebooted. The Pi is a normal USB host now, and the XIAO host-to-host bridges link it to the SSP and the
  i2c2midi.
- **Bluetooth.** The DualSense used to connect over Bluetooth; it is on a USB cable now.
- **Software.** `pydualsense` was replaced by the small hidraw writer in `src/midi/dualsense_hid.py`, the fixed
  button layout by `config/controls.py`, and the SwiftUI mapper app by Kontrolleur.
- **MPC-20 over libusb.** `src/doremidi_bridge.py` and `DOREMIDI_DEBUG_NOTES.md` document an attempt to read the
  MPC-20 through libusb before the kernel route worked; `doremidi-bridge.service`, `doremidi-powercycle.service`,
  `udev/99-doremidi-reset.rules` and `scripts/reset-doremidi.sh` date from that time and aren't needed.
