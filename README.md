# DualSense → Eurorack MIDI

A Raspberry Pi 4 bridge that turns a PS5 DualSense controller into a full-featured MIDI performance instrument for eurorack synthesizers. The system runs as two background services — one translating controller input to MIDI, one routing MIDI between all connected hardware — and is designed to be reliable enough for live use.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [How It All Connects](#3-how-it-all-connects)
4. [Raspberry Pi 4 Setup — From Scratch](#4-raspberry-pi-4-setup--from-scratch)
5. [USB Gadget Mode (Pi as MIDI Device)](#5-usb-gadget-mode-pi-as-midi-device)
6. [Pairing the DualSense Controller](#6-pairing-the-dualsense-controller)
7. [Installing the Project](#7-installing-the-project)
8. [Installing & Enabling the Services](#8-installing--enabling-the-services)
9. [Complete Control Reference](#9-complete-control-reference)
10. [MIDI Routing Reference](#10-midi-routing-reference)
11. [LED & Haptic Feedback Reference](#11-led--haptic-feedback-reference)
12. [Service Management & Troubleshooting](#12-service-management--troubleshooting)
13. [Development Workflow](#13-development-workflow)

---

## 1. System Overview

The project consists of two systemd services that start automatically on boot:

**`dualsense-midi.service`** — Reads the DualSense controller via Bluetooth (evdev), translates buttons, sticks, triggers, touchpad, and motion sensors into MIDI messages, and exposes a virtual MIDI port (`DualSense_Controller`).

**`midi-passthrough.service`** — Acts as a MIDI hub. It watches for the DualSense virtual port, the NerdSEQ sequencer (USB), and the 16n faderbank (USB), and routes messages to both the Percussa SSP (USB gadget) and NerdSEQ, with per-destination rescaling where needed. Supports hot-plug: devices can be connected or disconnected at any time without restarting the service.

```
DualSense (Bluetooth)
        │
        ▼
[dualsense-midi.service]
        │  virtual port: DualSense_Controller
        ▼
[midi-passthrough.service]  ◄── NerdSEQ (USB host)
        │                   ◄── 16n faderbank (USB host)
        ├──────────────────► Percussa SSP (USB gadget / f_midi)
        └──────────────────► NerdSEQ (USB host, DualSense + 16n only)
```


---

## 2. Hardware Requirements

| Component | Notes |
|---|---|
| Raspberry Pi 4 (any RAM) | Must be Pi 4 — USB-C port supports gadget mode via dwc2 |
| PS5 DualSense controller | Connected via Bluetooth |
| Percussa SSP | Connected via USB-C (Pi in gadget mode → SSP host slot) |
| NerdSEQ | Connected via USB-A host port on Pi |
| 16n faderbank | Connected via USB-A host port on Pi |
| USB hub (optional) | For connecting NerdSEQ + 16n to the same USB-A port |
| USB-C power/data splitter | Allows Pi USB-C to carry both power and MIDI data to SSP |
| MicroSD card (16GB+) | For Raspberry Pi OS Lite |

> **Important USB architecture note:** The Pi's USB-C port operates in gadget mode (Pi acts as a USB device to the SSP). This port cannot be split to multiple hosts. Use the USB-A ports for NerdSEQ, 16n, and any other USB host connections.

---

## 3. How It All Connects

```
                    ┌─────────────────────────────┐
                    │      Raspberry Pi 4          │
                    │                              │
  Percussa SSP ◄────┤ USB-C (gadget/device mode)  │
                    │                              │
  NerdSEQ      ────►│ USB-A port 1 (host mode)    │
                    │                              │
  16n faderbank────►│ USB-A port 2 (host mode)    │
                    │                              │
  DualSense    ◄───►│ Bluetooth                    │
                    └─────────────────────────────┘
```

The Pi presents itself to the SSP as a standard USB MIDI device (class-compliant, no driver needed). From the SSP's perspective it sees a device called `midid Gadget` appear in its host slots — exactly the same as any other USB MIDI controller.

---

## 4. Raspberry Pi 4 Setup — From Scratch

### 4.1 Flash Raspberry Pi OS Lite

Download and flash **Raspberry Pi OS Lite (64-bit)** using [Raspberry Pi Imager](https://www.raspberrypi.com/software/). In the imager's advanced settings before flashing:

- Set hostname (e.g. `ahab`)
- Enable SSH
- Set username and password
- Configure Wi-Fi (optional — you can also use ethernet)

### 4.2 First Boot & Updates

```bash
ssh youruser@ahab.local
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

### 4.3 Install System Dependencies

```bash
sudo apt install -y \
    python3-pip \
    python3-dev \
    libasound2-dev \
    librtmidi-dev \
    bluetooth \
    bluez \
    python3-bluez \
    git
```

### 4.4 Install Python Dependencies

```bash
pip3 install --break-system-packages \
    pydualsense \
    python-rtmidi \
    evdev
```

> **Note:** `pydualsense` requires hidapi access. Add your user to the `input` and `plugdev` groups:
> ```bash
> sudo usermod -aG input,plugdev $USER
> # Log out and back in for this to take effect
> ```

### 4.5 Allow Non-Root Access to DualSense HID

Create a udev rule so the service can access the DualSense without root:

```bash
sudo nano /etc/udev/rules.d/99-dualsense.rules
```

Add this line:
```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", MODE="0666"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```


---

## 5. USB Gadget Mode (Pi as MIDI Device)

This makes the Pi appear as a USB MIDI device to the Percussa SSP. All commands below must be run as root (or via sudo).

### 5.1 Enable the dwc2 USB Controller

```bash
echo "dtoverlay=dwc2" | sudo tee -a /boot/firmware/config.txt
echo "dwc2" | sudo tee -a /etc/modules
echo "g_midi" | sudo tee -a /etc/modules
```

> On older Raspberry Pi OS versions the config file may be at `/boot/config.txt` instead of `/boot/firmware/config.txt`.

### 5.2 Configure a Persistent USB Gadget Identity

The SSP (like most USB hosts) identifies devices by Vendor ID, Product ID, and **serial number**. If the serial number changes between reboots, the SSP treats it as a brand-new device and discards any saved MIDI bindings. Setting a fixed serial number keeps bindings intact across Pi reboots.

Create the gadget setup script:

```bash
sudo nano /usr/local/bin/setup-midi-gadget.sh
```

```bash
#!/bin/bash
# Set up USB MIDI gadget with fixed identity so SSP retains bindings

modprobe libcomposite

cd /sys/kernel/config/usb_gadget/
mkdir -p midi_gadget
cd midi_gadget

echo 0x1d6b > idVendor          # Linux Foundation
echo 0x0104 > idProduct         # Multifunction Composite Gadget
echo 0x0100 > bcdDevice
echo 0x0200 > bcdUSB

mkdir -p strings/0x409
echo "PercussaMIDIGadget" > strings/0x409/product
echo "MyCompany"          > strings/0x409/manufacturer
echo "00000001"           > strings/0x409/serialnumber   # FIXED — never changes

mkdir -p configs/c.1/strings/0x409
echo "MIDI Config" > configs/c.1/strings/0x409/configuration
echo 120            > configs/c.1/MaxPower

mkdir -p functions/midi.usb0
echo 64   > functions/midi.usb0/buflen
echo "DualSense MIDI" > functions/midi.usb0/id

ln -s functions/midi.usb0 configs/c.1/

# Bind to the first available UDC
ls /sys/class/udc > UDC
```

```bash
sudo chmod +x /usr/local/bin/setup-midi-gadget.sh
```

### 5.3 Run the Gadget Script at Boot

```bash
sudo nano /etc/systemd/system/midi-gadget.service
```

```ini
[Unit]
Description=USB MIDI Gadget Setup
After=local-fs.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup-midi-gadget.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable midi-gadget.service
sudo reboot
```

After reboot, verify with:
```bash
ls /sys/class/udc           # Should show something like "fe980000.usb"
cat /sys/kernel/config/usb_gadget/midi_gadget/UDC   # Should not be empty
```

The SSP should show `midid Gadget connected` when the Pi boots.

---

## 6. Pairing the DualSense Controller

### 6.1 Pair via bluetoothctl

```bash
sudo bluetoothctl
```

Inside the bluetoothctl shell:
```
power on
agent on
scan on
```

Put the DualSense into pairing mode by holding **PS button + Create button** until the lightbar flashes rapidly. You should see its MAC address appear (usually `Sony...` or `Wireless Controller`).

```
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
scan off
exit
```

### 6.2 Verify Connection

```bash
evtest
```

You should see three DualSense devices listed:
- `Sony Interactive Entertainment DualSense Wireless Controller` (buttons/sticks)
- `Sony Interactive Entertainment DualSense Wireless Controller Motion Sensors`
- `Sony Interactive Entertainment DualSense Wireless Controller Touchpad`

### 6.3 Auto-Reconnect on Boot

The controller will automatically reconnect after pairing as long as Bluetooth is on and the controller is turned on. The `dualsense-midi.service` handles the case where the controller is not yet connected at boot — it keeps the virtual MIDI port alive and waits.


---

## 7. Installing the Project

```bash
ssh youruser@ahab.local
cd ~
git clone https://github.com/ahabzutun/dualsense-eurorack-midi.git
cd dualsense-eurorack-midi
```

The service files reference the username `ahab` and the home path `/home/ahab/`. If your username is different, update these paths in both service files:

```bash
nano systemd/dualsense-midi.service
nano systemd/midi-passthrough.service
# Change "ahab" to your actual username in User= and WorkingDirectory=
```

---

## 8. Installing & Enabling the Services

```bash
sudo cp systemd/dualsense-midi.service /etc/systemd/system/
sudo cp systemd/midi-passthrough.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable dualsense-midi.service
sudo systemctl enable midi-passthrough.service
sudo systemctl start dualsense-midi.service
sudo systemctl start midi-passthrough.service
```

Both services start automatically on every boot from this point. The passthrough service depends on the DualSense service and waits 3 seconds after it starts (to ensure the virtual MIDI port exists before connecting).

### Service Dependency Chain

```
midi-gadget.service         (USB gadget identity — runs first)
        ↓
dualsense-midi.service      (DualSense → virtual MIDI port)
        ↓
midi-passthrough.service    (MIDI hub → NerdSEQ + SSP)
```

---

## 9. Complete Control Reference

### Channel Switching

The controller operates on three independent MIDI channels, each with its own LED colour, freeze states, and loop.

| Gesture | Channel | LED |
|---|---|---|
| SELECT (Create) | Channel 1 | ⚪ White |
| START (Options) | Channel 2 | 🩵 Turquoise |
| SELECT + START together | Channel 3 | 🟡 Yellow |

---

### Analog Inputs → MIDI CC

| Control | CC | Notes |
|---|---|---|
| Left Stick X | CC 1 | With deadzone + slice banking |
| Left Stick Y | CC 2 | With deadzone |
| Right Stick X | CC 74 | Filter cutoff |
| Right Stick Y | CC 71 | Resonance |
| L2 Trigger | CC 7 | Volume (freeze-able) |
| R2 Trigger | CC 10 | Pan (freeze-able) |
| D-Pad ↑ / ↓ | CC 11 | 8-step quantised (Expression) |

All analog CCs are sent on the current channel and recorded into the loop while recording is active.

---

### Face Buttons → CC Triggers

All four face buttons behave identically: **press → CC 127, release → CC 0**, and repeat at 50ms intervals while held. This is designed for MIDI-learn workflows where the target module needs sustained signal to bind.

| Button | CC |
|---|---|
| ✕ (X) | CC 14 |
| ○ (Circle) | CC 15 |
| △ (Triangle) | CC 22 |
| □ (Square) | CC 23 |

---

### Motion Sensors → NRPN (14-bit)

Motion is disabled by default per channel. Enable with **L1 + R1** together. The controller calibrates its resting position at the moment of enabling.

Motion uses 14-bit NRPN messages (0–16383) for significantly higher resolution than standard 7-bit CC. Smoothing and deadzones are applied in 14-bit space.

| Motion | NRPN Parameter | Range |
|---|---|---|
| Tilt X (left/right) | NRPN 0 | 0–16383 |
| Tilt Y (forward/back) | NRPN 1 | 0–16383 |
| Twist (yaw/rotation) | NRPN 2 | 0–16383 |

Motion is per-channel — enabling it on channel 2 does not affect channels 1 or 3.

Haptic feedback (rumble) activates when tilt reaches extreme positions, giving physical resistance cues.

---

### Slice Banking (Left Stick X)

The left stick X axis controls a **slice selector** across up to 128 values, organised into 8 banks of 16 slices each.

| Control | Action |
|---|---|
| D-Pad ← | Previous bank (slices 1–16 / 17–32 / etc.) |
| D-Pad → | Next bank |
| Left Stick X | Select slice within current bank |

The slice bank is per-session (resets on service restart) and independent per channel.

---

### Freeze Feature

Freeze locks the current CC value for a group of controls, so moving those controls no longer sends MIDI. Useful for holding a parameter while switching focus.

| Button | Freezes |
|---|---|
| L3 (Left Stick Click) | L2 trigger + R2 trigger + Left Stick X/Y |
| R3 (Right Stick Click) | Right Stick X/Y |

Freeze states are saved **per channel** — freezing on channel 1 does not affect channel 2.

---

### Loop Recording

The looper records all MIDI output (CC, NRPN, notes) into a per-channel buffer and plays it back in a continuous loop. Quantization can be applied non-destructively to the playback timeline.

| Gesture | Action |
|---|---|
| R1 long press (>1s) | Start / stop recording |
| R1 short press | Play / stop loop |
| L1 | Clear loop |

**Loop states and LED colours:**

| State | LED |
|---|---|
| Recording | 🔴 Red pulse |
| Overdub (recording while playing) | 🟣 Purple pulse |
| Playing | 🟢 Green pulse |
| Idle | Dim channel colour |

---

### Touchpad

| Gesture | Action |
|---|---|
| Finger X position | Scrub playback start position within the loop |
| Finger Y position | Select quantization subdivision (while finger is down) |
| Physical click | Toggle quantization ON / OFF |
| Lift finger | Reset scrub position (quantize state unchanged) |

**Touchpad Y zones (top to bottom):**

| Zone | Subdivision |
|---|---|
| Top quarter | 1/32 |
| Second quarter | 1/16 |
| Third quarter | 1/8 |
| Bottom quarter | 1/4 |

Quantization is **non-destructive** — raw event timestamps are always preserved. Turning quantization on or off takes effect immediately on the next playback cycle. The last used subdivision is remembered and restored when you click to re-enable.

**Player indicator dots** (the 4 white dots below the touchpad) show the active subdivision at a glance:

| Dots | Subdivision |
|---|---|
| ○ ○ ○ ○ | OFF |
| ● ○ ○ ○ | 1/4 |
| ● ● ○ ○ | 1/8 |
| ● ● ● ○ | 1/16 |
| ● ● ● ● | 1/32 |

---

### Harmonic Strummer (PlayStation Button)

Pressing the PlayStation logo button triggers an **arpeggio strum** — a chord of 4 notes generated randomly around the current D-Pad pitch position, sent as Note ON/OFF messages with a 80ms delay between notes.

---


## 10. MIDI Routing Reference

The passthrough service acts as a hub and applies routing rules per source:

| Source | → NerdSEQ | → Percussa SSP | Notes |
|---|---|---|---|
| DualSense controller | ✅ | ✅ | All channels |
| 16n faderbank | ✅ (raw 0–127) | ✅ (rescaled) | See rescaling below |
| NerdSEQ | ❌ | ✅ | No echo back to NerdSEQ |

### 16n → SSP Rescaling

The SSP maps incoming CC 0–127 linearly to its internal 0.0–1.0 parameter range, where 1.0 represents maximum gain (well above unity for PMIX and ATTN modules). The passthrough rescales 16n fader values (channel 16, CC 80–95) before forwarding to the SSP.

The rescaling ceiling is set in `midi_passthrough.py`:

```python
self.SSP_CC_MAX = 127   # 127 = no compression; full fader travel = 1.0 on SSP
```

**Tuning guide:**
- `SSP_CC_MAX = 127` — full fader travel reaches 1.0 (0dB on PMIX) — use when SSP base parameter is at 0
- Lower `SSP_CC_MAX` only if modulation is added on top of a non-zero base parameter on the SSP side

NerdSEQ always receives unmodified 0–127 values.

### Clock Sync

`dualsense-midi.service` includes a `ClockSource` that listens for MIDI clock from NerdSEQ (24 ppqn). When detected, the looper's quantizer locks to the external tempo. If the external clock disappears for more than 2 seconds, it falls back to the internal 120 BPM clock automatically.

The LED bar indicates sync state when quantization is active:
- 🟢 Green blink — locked to NerdSEQ external clock
- Channel colour blink (⚪/🩵/🟡) — running on internal clock

---

## 11. LED & Haptic Feedback Reference

### LED Bar (Side Light Bars)

Priority order — higher entries override lower ones:

| Priority | State | Colour |
|---|---|---|
| 1 | Recording + playing (overdub) | 🟣 Purple pulse |
| 2 | Recording | 🔴 Red pulse |
| 3 | Quantize ON + external clock | 🟢 Green blink |
| 4 | Quantize ON + internal clock | ⚪/🩵/🟡 Channel blink |
| 5 | Playing | 🟢 Green pulse |
| 6 | Motion enabled | 🔵 Blue solid |
| 7 | Idle | Dim channel colour |

### Haptics

Rumble motors activate based on tilt angle:
- Left motor → activates when tilting left past threshold
- Right motor → activates when tilting right past threshold
- Both motors → activate together for extreme Y-axis (forward/back) tilt

Intensity scales with how far past the threshold the controller is tilted. A brief double-pulse fires on freeze toggle (both motors) and bank change (left or right motor).

---

## 12. Service Management & Troubleshooting

### Everyday Commands

```bash
# Check status
sudo systemctl status dualsense-midi.service
sudo systemctl status midi-passthrough.service

# View live logs
sudo journalctl -u dualsense-midi.service -f
sudo journalctl -u midi-passthrough.service -f

# Restart after code changes (preferred — does not break USB gadget connection)
sudo systemctl restart dualsense-midi.service
sudo systemctl restart midi-passthrough.service

# View recent log (last 50 lines)
sudo journalctl -u dualsense-midi.service -n 50 --no-pager
```

### Pulling Updates from Git

```bash
cd ~/dualsense-eurorack-midi
git fetch origin
git reset --hard origin/main
sudo systemctl restart dualsense-midi.service
sudo systemctl restart midi-passthrough.service
```

> **Always prefer `systemctl restart` over rebooting.** A full reboot causes a USB disconnect/reconnect cycle which may cause the SSP to forget its MIDI bindings. Service restarts keep the USB gadget connection alive.

### Checking ALSA MIDI Clients

```bash
cat /proc/asound/seq/clients   # Shows all registered ALSA clients
aconnect -l                    # Lists ALSA MIDI connections
aseqdump -p 0                  # Dump all MIDI traffic (useful for debugging)
```

### Common Problems

**SSP loses MIDI bindings after Pi reboot**

This is caused by the USB gadget presenting a different identity on each boot. Verify the serial number is fixed:
```bash
cat /sys/kernel/config/usb_gadget/midi_gadget/strings/0x409/serialnumber
# Should print: 00000001
```
If the gadget setup script isn't running at boot, check:
```bash
sudo systemctl status midi-gadget.service
```

**DualSense not detected**

```bash
bluetoothctl
# Inside bluetoothctl:
devices          # Check if controller is listed
connect XX:XX:XX:XX:XX:XX
```
Then check logs:
```bash
sudo journalctl -u dualsense-midi.service -n 30 --no-pager
```

**NerdSEQ or 16n not showing up in passthrough**

```bash
sudo journalctl -u midi-passthrough.service -n 30 --no-pager
```
The passthrough rescans every 2 seconds — connecting the device should trigger a `✅ Input connected` or `✅ Output connected` log entry within a few seconds.

**ALSA client limit reached**

If the passthrough was crashing and restarting repeatedly, leaked ALSA clients can exhaust the system cap of 64:
```bash
cat /proc/asound/seq/clients | grep "Client " | wc -l
```
If this is close to 64, reboot once to clear all clients, then the fixed passthrough code will prevent future leaks.

**Memory growth (RSS increasing over time)**

The DualSense motion sensors generate ~1500 evdev events per second. The service handles this with:
- Tightened GC thresholds (200/5/2 instead of Python's default 700/10/10)
- Rate-limited motion processing at 50Hz (raw kernel buffer drain between samples)
- `malloc_trim()` every 30 seconds to return freed pages to the OS

Expected steady-state memory growth: < 0.5 MB/minute. If you see faster growth, check `sudo journalctl -u dualsense-midi.service` for exception traces.

---

## 13. Development Workflow

### Recommended Cycle (no reboot needed)

1. Edit files on your Mac
2. Push to GitHub
3. On the Pi:
```bash
cd ~/dualsense-eurorack-midi
git fetch origin && git reset --hard origin/main
sudo systemctl restart dualsense-midi.service
sudo systemctl restart midi-passthrough.service
```

### Clearing Stale Python Bytecode

If you see unexpected behaviour after a code change (stale `.pyc` running instead of new source):
```bash
find ~/dualsense-eurorack-midi -name "*.pyc" -delete
find ~/dualsense-eurorack-midi -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null
sudo systemctl restart dualsense-midi.service
```

### Testing MIDI Output

```bash
aseqdump -p 0   # Watch all MIDI traffic in real time
aconnect -l     # See port connections
```

### Diagnostic Mode (16n → SSP rescaling)

In `midi_passthrough.py`, set `DIAGNOSTIC_MODE = True` to log raw 16n fader values without forwarding. This lets you verify what values are arriving from the faderbank independently of the SSP:

```python
self.DIAGNOSTIC_MODE = True   # prints raw values, does NOT forward
```

---

*Maintained by [@ahabzutun](https://github.com/ahabzutun)*
