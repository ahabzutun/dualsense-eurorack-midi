# DOREMiDi MPC-20 USB Bridge — Debug Session Notes
**Date:** 2026-03-07  
**Status:** Unresolved — parked for now  
**Branch context:** All changes were made directly on `main` during live debugging

---

## Device Profile

- **VID/PID:** 1a86:752d (CH34x family, STM32-based)
- **USB interfaces:**
  - 0: Audio Control (snd-usb-audio)
  - 1: MIDI Streaming (snd-usb-audio) — EP_IN 0x81
  - 2: CDC ACM control
  - 3: CDC Data
- **CDC union descriptor is malformed** — cdc_acm kernel driver always fails with errno -22
- **Key quirk:** The STM32 requires a CDC SET_LINE_CODING (31250 baud, 8N1) + SET_CONTROL_LINE_STATE (DTR+RTS = 0x03) handshake sent to interface 2 before its MIDI endpoint becomes active
- **Device is on powered USB hub, port 1-1.4** — backfeeds 5V during Pi reboots so STM32 never loses power unless hub port is explicitly cut

---

## What Worked

### Raw manual test (confirmed working)
With all services stopped and running as root:
1. The previous bridge session's libusb cleanup had sent `SET_CONFIGURATION(0)`, leaving the device in USB ADDRESS state (unconfigured)
2. Manual `set_configuration(1)` triggered a genuine STM32 firmware reinitialisation
3. `claim_interface(1)` succeeded because snd-usb-audio hadn't yet rebound
4. CDC handshake sent, MIDI data flowed, `[TX]` lines appeared in logs

This is the **gold standard** sequence. Everything else has been attempts to replicate it reliably from a systemd service.

### Passthrough already subscribes to snd-usb-audio ALSA card
The passthrough log confirmed `DOREMiDi MPC-20-30C4 MIDI 1` connecting and disconnecting. snd-usb-audio creates a working ALSA card from interface 1 — the passthrough routes it correctly when it's present. This suggests an alternative architecture where the bridge only sends CDC and snd-usb-audio does the actual MIDI reading.

### uhubctl power cycling
`sudo uhubctl -l 1-1 -p 4 -a off && sleep 3 && on && sleep 4` confirmed working — cuts real hub power, forces STM32 into true power-on reset. Used in `doremidi-powercycle.service`.

---

## What Was Tried and Failed

### 1. Kernel QUIRK_MIDI_FIXED_ENDPOINT patch
Early attempt to patch snd-usb-audio to handle the malformed CDC descriptor. Abandoned — too invasive, kernel rebuild required.

### 2. snd-usb-audio modprobe blacklist
`options snd-usb-audio vid=0x1a86 pid=0x752d enable=0` — only marks ALSA disabled, snd-usb-audio still probes and binds interfaces. Worse, it turned out snd-usb-audio probing interfaces 0+1 appears to be a **required initialisation step** for the STM32's USB audio topology. Blacklisting it broke everything.

### 3. udev unbind script
A udev rule firing a sysfs unbind script on device attachment. Fired asynchronously — sometimes after the bridge had already started its CDC handshake, resetting the device at exactly the wrong moment.

### 4. set_configuration(0) → sleep(0.5) → set_configuration(1)
The 0.5s sleep between unconfigure and reconfigure gave snd-usb-audio enough time to rebind to interface 1 before the bridge could claim it. Every attempt to use this sequence with a sleep resulted in the endless rebind loop seen in logs.

### 5. set_configuration(0) → set_configuration(1) with no sleep
Without the sleep, the call to `set_configuration(1)` itself triggers USB re-enumeration which causes snd-usb-audio to rebind before `claim_interface(1)` executes. Still lost the race.

### 6. Skip set_configuration entirely (device already configured)
When the device was already in config 1, skipping `set_configuration` avoided the re-enumeration trigger. Both interfaces claimed successfully. CDC sent. But **no MIDI data arrived** — the STM32's endpoint never became active because the firmware reinitialisation (ADDRESS→CONFIGURED transition) never happened. The device was in a half-initialised state.

### 7. set_auto_detach_kernel_driver(True)
Attempted atomic detach+claim to eliminate race. `AttributeError` — not available in the installed pyusb version on the Pi.

### 8. driver_override sysfs lockout
Attempted to write `"none"` to `/sys/bus/usb/devices/1-1.4:1.X/driver_override` to prevent kernel rebind after detach. Not tested to completion — session was wrapped up before confirming if the sysfs path was correct for this device.

### 9. CDC-only bridge (interface 2 only, leave interface 1 to snd-usb-audio)
Claimed only interface 2, sent CDC handshake, left snd-usb-audio owning interface 1. No MIDI data appeared at NerdSEQ. Either the CDC handshake alone is insufficient without the STM32 being in a freshly configured state, or snd-usb-audio's ALSA routing needs something additional to pass through to the NerdSEQ path. Not fully debugged.

---

## Current Service Architecture

### doremidi-powercycle.service
Runs before `doremidi-bridge.service` at boot. Cuts hub power to port 1-1.4, waits 3s, restores, waits 4s. Ensures STM32 gets a true power-on reset at boot time.

```ini
[Unit]
Description=Power cycle and prime DOREMiDi before bridge starts
Before=doremidi-bridge.service

[Service]
Type=oneshot
ExecStart=/bin/sh -c "/usr/sbin/uhubctl -l 1-1 -p 4 -a off && sleep 3 && /usr/sbin/uhubctl -l 1-1 -p 4 -a on && sleep 4"
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### doremidi-bridge.service
Runs as root. Attempts to claim interfaces, send CDC, read MIDI from EP 0x81, forward to virtual ALSA port "DOREMiDi MPC-20".

### midi-passthrough.service
Subscribes to all MIDI inputs including both the bridge's virtual port and the snd-usb-audio ALSA card. Routes to NerdSEQ and SSP outputs.

---

## Suspicions / Most Likely Root Cause

### Primary suspect: snd-usb-audio rebind race is unwinnable from userspace

The core problem is that `set_configuration(1)` — which is required to trigger STM32 firmware reinitialisation — also triggers USB re-enumeration, which causes the kernel to immediately rebind snd-usb-audio to interface 1. There is no atomic "reconfigure + claim" operation available in userspace libusb. The kernel always wins this race.

The `driver_override` sysfs approach (approach #8 above) is the most promising unexplored path — if you can lock interface 1 against rebinding before calling `set_configuration(1)`, you win the race permanently. This needs testing with the correct sysfs path for the device.

### Secondary suspect: STM32 firmware requires full power cycle, not just USB reconfiguration

The raw test worked because the device had been through `set_configuration(0)` (ADDRESS state) *and* snd-usb-audio had previously done a full probe of interfaces 0+1. It may be that the STM32 needs both: snd-usb-audio probe (to initialise its internal USB audio topology) AND a subsequent ADDRESS→CONFIGURED transition (to activate the MIDI endpoint). If that's true, the correct sequence is:

1. Wait for snd-usb-audio to fully probe (interfaces 0+1 bound)
2. Detach snd-usb-audio from interfaces 0+1
3. Send `set_configuration(0)` → STM32 enters ADDRESS state
4. Send `set_configuration(1)` → STM32 reinitialises *with audio topology already set up*
5. Claim interface 1 before snd-usb-audio can rebind

Step 5 remains the unsolved race. The `driver_override` approach blocks it.

### Tertiary suspect: CDC handshake needs interface 1 claimed, not just interface 2

The CDC SET_LINE_CODING goes to interface 2 (wIndex=2), which is the CDC control interface. But the MIDI endpoint is on interface 1. It's possible the STM32 only activates EP 0x81 when interface 1 is claimed by the host, not just when CDC is sent. This would explain why the CDC-only approach (attempt #9) produced no data even though the handshake appeared to succeed.

---

## Recommended Next Steps

1. **Test `driver_override` sysfs approach properly** — find correct path with `ls /sys/bus/usb/devices/ | grep 1-1.4`, write "none" to `driver_override` for interfaces 0, 1, 2, 3 before calling `set_configuration`. This prevents rebind without requiring atomic claim.

2. **Check if pyusb version supports `set_auto_detach_kernel_driver`** — `python3 -c "import usb; print(usb.__version__)"`. If not, `pip install --upgrade pyusb --break-system-packages` may resolve it.

3. **Try the full sequence with snd-usb-audio wait + driver_override + 0→1 cycle** — this is the most likely path to success based on everything learned.

4. **Contact Percussa** re: USB binding persistence (separate issue documented in `usb_enumeration_problems`).

5. **GPIO power bypass** (pins 4+6) — eliminate USB-C splitter voltage drop, independent of DOREMiDi issue.