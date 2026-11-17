# Kontrolleur

Configures the whole rig of this repository from one place. Runs on **Linux** (Fedora, Debian/Ubuntu,
Raspberry Pi OS) and **macOS**; Python + Qt (PySide6).

| Tab | Device | How it talks to it |
|---|---|---|
| DualSense | PS5 controller on the SSH host | `er301_routes.py` + `controls.py` over SSH (`scripts/er301_routes_tool.py`) |
| Routing | where the MIDI goes: the controllers, the Raspberry Pi (DualSense script + MIDI hub; *Scripts* shows how the two services work together, *Stages* opens up the DualSense script), the i2c2midi with its LFO and ER-301 routes, the ER-301 and Teletype on I2C, the Percussa SSP's patch. Click a controller's row to trace it hop by hop. Boxes can be dragged and folded | patched from the DualSense, 16n, pedals or EC4: `controls.py` (`MIDI_INPUTS`) + `er301_routes.py`; SSP: `scripts/ssp_tool.py` over ssh to the SSP (reads the MIDI learn / MIDI modules of its presets) |
| 16n | 16n faderbank (fork with curves and output ranges) | `scripts/fader16_tool.py`, over SSH or on a cable to this computer |
| Pedals | DOREMiDi MPC-20 | `scripts/mpc20_tool.py`, over SSH or on a cable |
| EC4 | Faderfox EC4 | `scripts/ec4_tool.py`, over SSH or on a cable (transfers are started on the EC4) |
| ER-301 | i2c2midi → ER-301 | port overview (each source links to its control), MIDI routes, pitch & LFO |
| Macros | the grid macros: the monome grid's left half as the Pi runs it — 8 banks × 7 slots. Each slot's contents in this rig's words (which control a value came from, which ER-301 port / SSP input it reaches); names for slots and banks, slew, copy / move / clear, take single values out, what a bank saves, the settings (slew times, curve, which controllers are remembered, pickup, send back); backups as files; **▶ Load this slot now** loads it on the Pi as if pressed on the grid | `grid_macros.json` over SSH (`scripts/grid_macros_tool.py`); Upload refuses when a slot was saved on the grid since the Load. Routing shows the grid as a controller: left half → GRID MACROS input of the MIDI hub, right half → Teletype over the Pi's USB-C |
| Teletype | the scene's scripts: what each line sends through the i2c2midi (USB channel, CC / note) and which SSP input it reaches, ER-301 ports it writes directly, mistakes (TRS channels, `I2M.CH` only in Init, unknown ops); edit scripts; **Live** shows what really arrives from the i2c2midi at the Pi | scenes by USB stick: the Teletype's WRITE TO USB → `↓ From stick` reads `tt00s.txt …`; `↑ To stick` writes `tt##.txt` for READ FROM USB. Live: `scripts/midi_monitor.py` piped over SSH to the Pi |

**Keeping things in sync.** Upload only after a Load in the same session; it refuses when the files on the Pi changed
since (another computer, or by hand) and asks whether to load them or upload anyway. After writing it reads
everything back and reports what the i2c2midi answered (confirmed / not answering: rack off / rejected). An edit made
while a load is still running is kept. The last 16n and MPC-20 settings are kept on this computer and refreshed in the
background, the last EC4 setups on the Pi too, so the Routing and ER-301 tabs show the right sources from the start.
Routing › Raspberry Pi lists **Checks**: ports several routes write to, routes nothing sends, patched messages no
control sends, old device data. Every connection in Routing links to its control (↗), and a cable can be clicked.

Also: notes per control (also written into `er301_routes.py` comments), saved configurations
(`~/Documents/Kontrolleur/*.dsmconfig`), adjustable colours / fonts / text size, zoomable drawings
(pinch, scroll wheel, Ctrl +/−/0, drag).

## Install

**Linux**

```sh
git clone https://github.com/ahabzutun/dualsense-eurorack-midi.git ~/dualsense-eurorack-midi
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
To update: `git pull` in the repository and restart the app. `packaging/build-mac-dmg.sh` builds a standalone
`.dmg` (PyInstaller) for a Mac without the repository.

## Where things are kept

| | Linux | macOS |
|---|---|---|
| Settings (host, routes, appearance) | `~/.config/Kontrolleur/Kontrolleur.conf` | `~/Library/Preferences/com.kontrolleur.Kontrolleur.plist` |
| Notes, last EC4 dump, Teletype scenes (`teletype/`) | `~/.local/share/kontrolleur/` | `~/Library/Application Support/Kontrolleur/` |
| Configurations | `~/Documents/Kontrolleur/` | `~/Documents/Kontrolleur/` |

Configuration files are the same on both systems.

## Checking the UI without devices

`KONTROLLEUR_DEBUG_WINDOWS=1` logs every window shown besides the main one to `windows.log` in the data folder.

`KONTROLLEUR_OFFLINE=<folder with export.json, fader16.json, mpc20.json, ec4-last.json, grid-macros.json>` loads data from
files instead of the devices; `KONTROLLEUR_SNAPSHOT=out.png` (+ `KONTROLLEUR_MODE`, `_TAB`, `_SIZE`, `_SCALE`,
`_FONT`, `_PRESET`, `_TT_SCENE`, `_MAC_BANK`, `_MAC_SLOT`) renders the window to a PNG and quits. Works with `QT_QPA_PLATFORM=offscreen`.

`scripts/` holds the app icon (`make_icon.py` draws it). The SwiftUI version this replaced is in the git
history (removed after commit f1ca7b6).
