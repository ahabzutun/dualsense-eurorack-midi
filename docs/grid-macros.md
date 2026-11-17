# Grid macros

The monome grid goes into the Pi instead of into Teletype. The Pi keeps the **left 8×8** for itself, as a
snapshot / morph controller for the whole rig, and passes the **right 8×8** on to Teletype, which still
sees a normal 16×8 grid. The Pi's USB-C port is Teletype's USB device: a grid, and (if Teletype takes
both) a keyboard at the same time, so scripts can be written without swapping cables.

```
 monome grid ── USB-A ──▶ ┌──────── Pi ("ahab") ────────┐
 USB keyboard ─ USB-A ──▶ │ grid_macros.py   left 8×8 ──┼──▶ macros: DualSense script, MIDI hub
                          │                  right 8×8 ─┼─┐
                          │ keyboard_forward.py ────────┼─┤
                          └─────────────────────────────┘ │  USB-C (gadget: serial grid + keyboard)
                                                          └──▶ power/data splitter ──▶ Teletype USB
```

Status: **built and tested without hardware** (`python3 test/test_grid_macros.py`, plus the simulator).
Nothing here has run against the real grid or Teletype yet — see [First test on the hardware](#first-test-on-the-hardware).

## The left half

| Where | What |
|---|---|
| column 1 | 8 banks, one per row |
| columns 2–8 | 7 macro slots per bank |
| top row | **save**: hold ½ s to save the rig's state into the slot |
| rows 2–7 | **slew fader**: how slowly a load moves there — row 7 (just above load) instant, row 2 slowest |
| bottom row | **load**: tap to go to the slot's state |
| save held + load | clear the slot |

The slew steps are 0 · 0.1 · 0.5 · 2 · 8 · 30 s by default (`glide_times`). The fader works on empty
slots too; it is stored per slot.

LEDs: current bank bright, banks with saved slots dim. Saved slots show their save and load keys; the slot
loaded last in a bank is bright, and pulses while it glides in. The save key brightens while it is held
and flashes when the save happens.

## What a snapshot holds

| Part | Captured | Restored |
|---|---|---|
| `dualsense` | sample block, section, fine slice; freeze of the left group (L2, R2, left stick) and the right stick with their held values; motion control on/off (current instrument) | at once, at the end (or start) of the glide (`jump_at`): a slice can't glide without scanning through every slice in between |
| `midi` | the last value of every CC and NRPN that came from the remembered controllers (`midi_sources`, default 16n and EC4) | glides from where each value is now, through the macros' own MIDI port `Grid_Macros` into the hub (so SSP rescaling, Teletype thinning etc. apply as usual) |
| `teletype` | what Teletype showed on the right half (its LEDs) | nothing yet — kept so a snapshot remembers it; restoring needs a look at what Grid Control mode can do on the real grid |

While a load glides:

- **A hand takes over.** Move a control that is gliding and it stops gliding: the hand wins.
- **16n faders pick up** (`pickup_sources`). After a load, a fader no longer matches its value. The hub holds
  its messages back until it passes the value (or comes within `pickup_window`), so nothing jumps. The
  value it has to pass is the one the glide has reached, so a fader can also catch a moving value. Faders
  whose value didn't change aren't held back.
- **The EC4 gets its values back** (`feedback_sources`), so its encoders continue from the loaded value.
- **Loading during a glide** starts from wherever the values are at that moment.

Only what goes through the Pi can be remembered. A knob turned on a module itself isn't in a snapshot.

## Settings: `~/.config/dualsense-eurorack-midi/grid_macros.json`

Snapshots, slew settings, names and settings live in one JSON file, next to `controls.py` and
`er301_routes.py`. The service creates it with the defaults, saves into it, and reloads it when it changes.
Kontrolleur's Macros tab edits it (`scripts/grid_macros_tool.py` over SSH); by hand works too.

```jsonc
"settings": {
  "glide_times": [0.0, 0.1, 0.5, 2.0, 8.0, 30.0],
  "curve": "linear",                      // linear · smooth · exp
  "hold_to_save": 0.5,
  "jump_at": "end",                       // when the DualSense state changes during a glide: start / end
  "tick_hz": 50,
  "providers": ["dualsense", "midi", "teletype"],
  "bank_providers": {"2": ["midi"]},      // optional: bank 3 only saves / loads MIDI values
  "midi_sources": ["16n", "EC4"],         // inputs (by port name) whose values are remembered
  "pickup_sources": ["16n"],
  "pickup_window": 2,
  "feedback_sources": ["EC4"]
}
```

## Kontrolleur

The **Macros** tab shows the left half as the grid shows it, with the selected bank's slots underneath.

- **Slot:** name, slew, when it was saved. What it holds in this rig's words: the DualSense state (block,
  section, slice, freeze, motion), each MIDI value named by its control ("16n Fader 3", "EC4 S1 G2
  encoder 4") with where it lands (ER-301 port, SSP input of the patch in use). ✕ takes a single value or
  part out of the slot. Copy / move (a move swaps with what's at the target) / clear.
  **▶ Load this slot now** makes the Pi load it, as if its load key were pressed.
- **Bank:** name, its slots, what the bank saves and loads (e.g. a bank of mixer scenes that leaves the
  slice alone), copy the whole bank.
- **Settings:** slew times and curve, when the slice / freeze change (start / end), hold time for save,
  which controllers are remembered / pick up / get values sent back. Backups of everything as a file.

↓ Load / ↑ Upload work like in the other tabs. Upload refuses when the file on the Pi changed since the Load,
which usually means a slot was saved on the grid; then Load first, or upload anyway. The Pi keeps the last
10 versions in `~/.config/dualsense-eurorack-midi/backups/`.

**Routing** shows the grid as a controller once the grid macros are on the Pi. Its left half runs into the
MIDI hub's GRID MACROS input, its right half goes around to Teletype. Its panel lists everything the slots
touch, and where each value lands.

## Trying it on a computer

```bash
python3 src/grid_macros_sim.py          # opens http://localhost:8301
```

The real engine, with stand-ins: a 16n (8 faders), an EC4 (4 encoders), the DualSense script's slice and
freeze, and Teletype's half (click it to change what it shows). Hold a save key, move things, load. With a
mouse you can't hold save and press load at once, so shift-click a load key to clear. Snapshots go to
`grid_macros_sim.json`, separate from the Pi's file.

## On the Pi

```bash
./install-pi.sh --grid-macros       # then reboot once
```

On top of the usual setup this adds:

- `dtoverlay=dwc2,dr_mode=peripheral` in `/boot/firmware/config.txt`: the USB-C port becomes a USB
  device. The four USB-A ports are a separate controller and stay hosts (DualSense, hub, grid, keyboard).
- `teletype-gadget.service` (`scripts/teletype_gadget.sh both`): the USB-C port appears as one device
  with a CDC serial interface (the grid, `/dev/ttyGS0`) and a boot keyboard (`/dev/hidg0`).
- `grid-macros.service` (`src/grid_macros.py`): grid proxy and macros.
- `keyboard-forward.service` (`src/keyboard_forward.py`): a USB keyboard on the Pi types into Teletype.
- a patched gadget keyboard module (`scripts/build_teletype_hid.sh`, installed to
  `/lib/modules/<kernel>/updates/`) and `teletype-hid-module.service`, which builds it again at boot after a
  kernel update (needs the kernel headers and internet once; takes about a minute).
- the `dialout` group for the serial ports.

Cabling: the Pi's USB-C goes through the power/data splitter, with power from the Pi's supply and data to
Teletype's USB port. No VBUS from the Pi towards Teletype. The grid has to be in **serial (monome) mode**,
not iii mode — as it is now on Teletype.

Modes of the gadget (each switch looks to Teletype like unplugging one device and plugging in another):

```bash
sudo scripts/teletype_gadget.sh both       # grid + keyboard together (default)
sudo scripts/teletype_gadget.sh grid       # grid only
sudo scripts/teletype_gadget.sh keyboard   # keyboard only
sudo scripts/teletype_gadget.sh status
```

### What Teletype needs (found on the hardware, Nov 2026)

- **Keyboard first, grid second.** `uhi_cdc_install` (libavr32) takes the device only when the *last*
  interface is the CDC data interface; `teletype_gadget.sh both` links the keyboard first for that reason.
  Vendor/product IDs and names aren't checked.
- **The grid's serial port must be open when Teletype connects.** Teletype sends the size query (`0x05`)
  about 0.5 s after connecting and only takes the grid if it gets an answer. `grid-macros.service` keeps the
  port open and re-plugs the gadget 2 s after it starts (`teletype_gadget.sh rebind`), so the boot order
  doesn't matter. The journal says "🔌 Teletype connected the grid" when it worked.
- **The kernel's gadget keyboard doesn't work with Teletype.** Teletype collects the first key report and
  then stops. Two things in `usb_f_hid` together: the endpoint interval (4/10 → 1, the Pi's dwc2 otherwise
  schedules reports in frames Teletype's host doesn't poll) and the zero-length packet after each report
  (added to the kernel in 2025, removed again). `scripts/build_teletype_hid.sh` patches exactly those lines
  and refuses to build if a newer kernel source no longer has them.
- **Grid Control mode** is off after every power-up; the front button toggles it. It draws on columns 8–15.
- **What Teletype sends the grid:** only `0x05` and whole-quad level maps (`0x1A`, quad x = 0 or 8). Its
  parser stops at the first byte it doesn't know, so the Pi passes it nothing but key events and the size
  answer.

### Checking it

```bash
scripts/teletype_gadget.sh status            # /dev/ttyGS0 and /dev/hidg0 there, functions linked
scripts/build_teletype_hid.sh --check && echo "patched keyboard module installed"
journalctl -u grid-macros -f                 # grid connects, saves, loads
journalctl -u keyboard-forward -f            # keyboards found; a ⚠️ line means reports weren't collected
```

## Known limits / open points

- **Teletype restore** isn't there yet (its LEDs are only remembered). Two routes once the hardware shows
  what's possible: the Pi presses keys on the right half (no script lines), or CCs through the i2c2midi
  that `M.ACT` / `SCENE` lines act on (every scene you switch to needs those lines).
- **NRPN towards Teletype**: the hub thins CCs to Teletype per controller number. On channels other than 2,
  that can mix up the 4 messages of NRPNs to different parameters. This was already the case before the
  macros; the DualSense's NRPNs on ch 2 aren't thinned.
- **Fine slice and the left stick**: a loaded fine slice stays until the left stick moves again (unless the
  left group is frozen).
- **Single instrument**: the DualSense part covers the current instrument (no control switches instruments today).
- **Pedals as macro controls** (load next / previous slot by foot) aren't there yet.
