# Kontrolleur

Configures the whole rig of this repository from one place. Runs on **Linux** (Fedora, Debian/Ubuntu,
Raspberry Pi OS) and **macOS**; Python + Qt (PySide6).

| Tab | Device | How it talks to it |
|---|---|---|
| DualSense | PS5 controller on the SSH host | `er301_routes.py` + `controls.py` over SSH (`scripts/er301_routes_tool.py`) |
| Pi | the Pi's own functions (Slice-Selekteur, Loop-Rekordeur, LFO, DualSense extras) | patched from the DualSense, 16n, pedals or EC4: `controls.py` (`MIDI_INPUTS`) + `er301_routes.py` |
| 16n | 16n faderbank (fork with curves) | `scripts/fader16_tool.py`, over SSH or on a cable to this computer |
| Pedals | DOREMiDi MPC-20 | `scripts/mpc20_tool.py`, over SSH or on a cable |
| EC4 | Faderfox EC4 | `scripts/ec4_tool.py`, over SSH or on a cable (transfers are started on the EC4) |
| ER-301 | i2c2midi → ER-301 | port overview, MIDI routes, pitch & LFO |

Also: notes per control (also written into `er301_routes.py` comments), saved configurations
(`~/Documents/Kontrolleur/*.dsmconfig`), adjustable colours / fonts / text size, zoomable drawings
(pinch, scroll wheel, Ctrl +/−/0, drag).

## Install

**Linux**

```sh
git clone -b kontrolleur-and-device-tools https://github.com/ahabzutun/dualsense-eurorack-midi.git ~/dualsense-eurorack-midi
cd ~/dualsense-eurorack-midi/kontrolleur
./install-linux.sh           # venv with PySide6-Essentials, alsa-midi, pyusb; launcher + menu entry
./install-linux.sh --udev    # same, plus access to an MPC-20 plugged into this computer (sudo)
```

Start it from the app menu or with `kontrolleur`. Needs Python ≥ 3.9, `git` and `ssh` (all on Fedora
by default). MIDI to a 16n or EC4 on a cable uses python-rtmidi where it can be installed and
`scripts/rtmidi_compat.py` (alsa-midi) where it can't, e.g. Fedora 44.

**macOS**

```sh
cd kontrolleur
./install-mac.sh             # venv + ~/Applications/Kontrolleur.app
```

Or run it straight from the repository: `python3 kontrolleur.py` (with PySide6 or PySide6-Essentials installed).

## Where things are kept

| | Linux | macOS |
|---|---|---|
| Settings (host, routes, appearance) | `~/.config/Kontrolleur/Kontrolleur.conf` | `~/Library/Preferences/com.kontrolleur.Kontrolleur.plist` |
| Notes, last EC4 dump | `~/.local/share/kontrolleur/` | `~/Library/Application Support/Kontrolleur/` |
| Configurations | `~/Documents/Kontrolleur/` | `~/Documents/Kontrolleur/` |

Configuration files are the same on both systems.

## Checking the UI without devices

`KONTROLLEUR_OFFLINE=<folder with export.json, fader16.json, mpc20.json, ec4-last.json>` loads data from
files instead of the devices; `KONTROLLEUR_SNAPSHOT=out.png` (+ `KONTROLLEUR_MODE`, `_TAB`, `_SIZE`, `_SCALE`,
`_FONT`, `_PRESET`) renders the window to a PNG and quits. Works with `QT_QPA_PLATFORM=offscreen`.

`scripts/` holds the app icon (`make_icon.py` draws it). The SwiftUI version this replaced is in the git
history (removed after commit f1ca7b6).
