#!/usr/bin/env python3
"""
DOREMiDi MPC-20 USB bridge service.

Performs CDC handshake to activate the MIDI endpoint, then reads USB MIDI
packets from EP 0x81 and forwards decoded MIDI to an ALSA virtual port
("DOREMiDi MPC-20") which midi-passthrough.service subscribes to.

Cold boot USB power cycling is handled by doremidi-powercycle.service
which runs uhubctl before this service starts.

--- KNOWN ISSUE (2026-03-07) ---
Interface 1 (MIDI Streaming, EP 0x81) is contested by snd-usb-audio.
The STM32 requires a full ADDRESS->CONFIGURED USB state transition
(set_configuration 0->1) to reinitialise its MIDI endpoint. This transition
triggers USB re-enumeration, causing snd-usb-audio to rebind to interface 1
before libusb can claim it. The race has not yet been won reliably.

See DOREMIDI_DEBUG_NOTES.md for full investigation history.

Most promising next step: sysfs driver_override to lock interfaces against
kernel rebind before the set_configuration cycle.
"""

import usb.core
import usb.util
import rtmidi
import struct
import time
import logging

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s')
log = logging.getLogger('doremidi_bridge')

VID = 0x1a86
PID = 0x752d
EP_IN = 0x81
POLL_TIMEOUT_MS = 100

CIN_LENGTH = {
    0x2: 2, 0x3: 3, 0x4: 3, 0x5: 1,
    0x6: 2, 0x7: 3, 0x8: 3, 0x9: 3,
    0xa: 3, 0xb: 3, 0xc: 2, 0xd: 2,
    0xe: 3, 0xf: 1,
}


def cdc_init(dev):
    """Send SET_LINE_CODING + SET_CONTROL_LINE_STATE to activate MIDI endpoint."""
    try:
        line_coding = struct.pack('<IBBB', 31250, 0, 0, 8)
        dev.ctrl_transfer(0x21, 0x20, 0, 2, line_coding)
        dev.ctrl_transfer(0x21, 0x22, 0x03, 2, None)
        log.info("CDC handshake sent")
    except usb.core.USBError as e:
        log.warning(f"CDC handshake failed: {e}")


def parse_usb_midi(data):
    """Parse USB MIDI packet into list of raw MIDI messages."""
    messages = []
    for i in range(0, len(data) - 3, 4):
        cin = data[i] & 0x0f
        if cin == 0:
            continue
        length = CIN_LENGTH.get(cin)
        if length is None:
            continue
        msg = list(data[i+1:i+1+length])
        if msg:
            messages.append(msg)
    return messages


def open_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        return None

    # Wait for snd-usb-audio to fully probe interfaces 0+1.
    # Its probe appears to initialise the STM32 USB audio topology
    # which is required before the MIDI endpoint becomes active.
    log.info("Waiting for snd-usb-audio to probe...")
    for _ in range(20):
        time.sleep(0.5)
        try:
            if dev.is_kernel_driver_active(0) or dev.is_kernel_driver_active(1):
                log.info("snd-usb-audio bound - waiting for full probe")
                time.sleep(1.0)
                break
        except Exception:
            pass
    else:
        log.warning("snd-usb-audio never bound - continuing anyway")

    # Detach kernel drivers from all interfaces
    for iface in [0, 1, 2, 3]:
        try:
            if dev.is_kernel_driver_active(iface):
                dev.detach_kernel_driver(iface)
                log.info(f"Detached interface {iface}")
            else:
                log.info(f"Interface {iface}: no kernel driver")
        except Exception as e:
            log.warning(f"Detach {iface}: {e}")

    # Force STM32 through ADDRESS->CONFIGURED state transition.
    # NOTE: triggers USB re-enumeration -> snd-usb-audio rebinds interface 1.
    # See known issue in module docstring.
    try:
        dev.set_configuration(0)
        log.info("Unconfigured (ADDRESS state)")
    except usb.core.USBError as e:
        log.warning(f"set_configuration(0): {e}")

    try:
        dev.set_configuration(1)
        log.info("Configured (firmware reinitialised)")
    except usb.core.USBError as e:
        log.warning(f"set_configuration(1): {e}")

    for iface in [1, 2]:
        try:
            usb.util.claim_interface(dev, iface)
            log.info(f"Interface {iface} claimed")
        except usb.core.USBError as e:
            log.warning(f"claim_interface {iface}: {e}")

    cdc_init(dev)
    time.sleep(0.3)
    cdc_init(dev)

    try:
        data = dev.read(EP_IN, 64, timeout=1000)
        log.info(f"Device verified - raw: {list(data)}")
    except usb.core.USBTimeoutError:
        log.info("Device initialised (waiting for input)")
    except usb.core.USBError as e:
        log.warning(f"Endpoint probe: {e}")
        try:
            usb.util.dispose_resources(dev)
        except Exception:
            pass
        return None

    return dev


def run():
    midi_out = rtmidi.MidiOut()
    midi_out.open_virtual_port("DOREMiDi MPC-20")
    log.info("Virtual MIDI port opened: DOREMiDi MPC-20")

    while True:
        dev = None
        log.info("Waiting for DOREMiDi device...")
        while dev is None:
            dev = open_device()
            if dev is None:
                try:
                    stale = usb.core.find(idVendor=VID, idProduct=PID)
                    if stale:
                        usb.util.dispose_resources(stale)
                except Exception:
                    pass
                time.sleep(2)

        log.info("DOREMiDi connected and initialised")
        try:
            while True:
                try:
                    data = dev.read(EP_IN, 64, timeout=POLL_TIMEOUT_MS)
                    log.info(f"[RAW] {list(data)}")
                    for msg in parse_usb_midi(data):
                        log.info(f"[TX] {[hex(b) for b in msg]}")
                        midi_out.send_message(msg)
                except usb.core.USBTimeoutError:
                    pass
                except usb.core.USBError as e:
                    log.warning(f"USB error: {e}")
                    break
        except Exception as e:
            log.error(f"Unexpected error: {e}")

        log.info("DOREMiDi disconnected, reconnecting...")
        time.sleep(2)


if __name__ == '__main__':
    run()