// XIAO MIDI host-to-host bridge — two Seeed XIAO SAMD21 boards, same firmware on both.
// Wiring: TX->RX, RX->TX, GND->GND.
// Based on marchingband/usb-midi-host2host, hardened:
//  - every packet is validated before it is forwarded (cable 0, CIN matches status, data < 0x80)
//  - invalid bytes are dropped one at a time, so the 4-byte framing resyncs by itself
//  - a partial packet older than BYTE_GAP_US is discarded (packets arrive back-to-back)
//  - internal pull-up on RX, so an unpowered partner board doesn't inject noise
//  - higher UART baud rate (must match on both boards)
//  - SysEx and system common messages pass through (validated)
//  - visible LED pulse per forwarded packet (the old toggle was invisible for even counts)
//  - self-recovery: a hardware watchdog resets the board if the loop ever stops for 2 s, and
//    the board resets itself if its USB host hasn't configured it (the "only the green power
//    LED" state after a host reboot or power dip). That self-reset backs off: after 10 s, then
//    30 s, then 2 min, then never again until the host configures it or the power goes. It
//    never fires while the host is mid-setup (address assigned): pulling out of a half-done
//    enumeration is what crashed the SSP's USB host (dwc2 "irq nobody cared").
//    On start-up the yellow LED blinks 3x after a watchdog reset and 2x after a self-reset.
//  - heartbeat to the Pi once a second, as a packet on cable 1 (dropped by any MIDI parser):
//    1F A5 (80|state) (80|attempts); state bit 0 = configured, 1 = addressed, 2 = gave up.
#include <Arduino.h>
#include <MIDIUSB.h>

#ifndef UART_BAUD
#define UART_BAUD 115200
#endif

static const uint32_t BYTE_GAP_US  = 1000;  // 4 bytes take ~350 us at 115200 baud
static const uint32_t LED_PULSE_MS = 15;
static const uint32_t RESET_AFTER_MS[] = { 10000, 30000, 120000 };  // per self-reset attempt
static const uint8_t  MAX_ATTEMPTS = 3;                               // then wait for the host
static const uint32_t HEARTBEAT_MS = 1000;

static uint8_t  buf[4];
static uint8_t  buf_n        = 0;
static uint32_t last_byte_us = 0;
static bool     led_idle     = LOW;
static bool     led_lit      = false;
static uint32_t led_off_at   = 0;

static void led_pulse() {
  if (!led_lit) { digitalWrite(LED_BUILTIN, !led_idle); led_lit = true; }
  led_off_at = millis() + LED_PULSE_MS;
}

// ── watchdog ────────────────────────────────────────────────────────────────
// GCLK2 (unused by the Arduino core) = OSCULP32K / 32 = 1024 Hz → WDT period 2048 = 2 s.
static void wdt_start() {
  GCLK->GENDIV.reg  = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);            // 2^(4+1) = 32
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY) {}
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;
  WDT->CTRL.reg = 0;
  while (WDT->STATUS.bit.SYNCBUSY) {}
  WDT->CONFIG.reg = WDT_CONFIG_PER_2K;
  WDT->CTRL.reg = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY) {}
}

static inline void wdt_feed() {
  if (!WDT->STATUS.bit.SYNCBUSY) WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

// Reset if the USB host hasn't configured us for a while (bus reset clears the configuration),
// with back-off. The attempt count survives NVIC_SystemReset in a RAM spot that neither this
// sketch (heap ends ~0x20000B00, stack grows down from 0x20008000) nor the UF2 bootloader
// (low RAM + top-of-RAM stack) touches; magic + check guard it, and any other reset cause
// (power, flashing, watchdog) starts again from 0.
struct Persist { uint32_t magic, attempts, check; };
static volatile Persist &persist = *reinterpret_cast<volatile Persist *>(0x20005F00u);
static const uint32_t PERSIST_MAGIC = 0x5A1A0B0Fu;
static uint8_t  attempts  = 0;       // self-resets since the host last configured us
static bool     gave_up   = false;
static uint32_t usb_ok_at = 0;

static void persist_store(uint8_t a) {
  persist.attempts = a; persist.check = ~(uint32_t)a; persist.magic = PERSIST_MAGIC;
}

static void persist_load() {
  const bool self_reset = PM->RCAUSE.reg & PM_RCAUSE_SYST;
  if (self_reset && persist.magic == PERSIST_MAGIC && persist.check == ~persist.attempts &&
      persist.attempts <= MAX_ATTEMPTS)
    attempts = (uint8_t)persist.attempts;
  else
    attempts = 0;
  gave_up = attempts >= MAX_ATTEMPTS;
  persist_store(attempts);
}

static inline bool usb_addressed() { return USB->DEVICE.DADD.bit.ADDEN; }

static void usb_health() {
  const uint32_t now = millis();
  if (USBDevice.configured()) {
    usb_ok_at = now;
    if (attempts || gave_up) { attempts = 0; gave_up = false; persist_store(0); }
    return;
  }
  if (usb_addressed()) { usb_ok_at = now; return; }   // host is setting us up: leave it alone
  if (gave_up) return;
  if (now - usb_ok_at > RESET_AFTER_MS[attempts]) {
    persist_store(attempts + 1);
    NVIC_SystemReset();
  }
}

static uint32_t hb_at = 0;
static void heartbeat() {
  const uint32_t now = millis();
  if (now - hb_at < HEARTBEAT_MS) return;
  hb_at = now;
  const uint8_t st = 0x80 | (USBDevice.configured() ? 1 : 0) | (usb_addressed() ? 2 : 0) |
                     (gave_up ? 4 : 0);
  const uint8_t p[4] = { 0x1F, 0xA5, st, (uint8_t)(0x80 | attempts) };
  Serial1.write(p, 4);
}

static void show_reset_cause() {
  const uint8_t rc = PM->RCAUSE.reg;
  const int blinks = (rc & PM_RCAUSE_WDT) ? 3 : (rc & PM_RCAUSE_SYST) ? 2 : 0;
  for (int i = 0; i < blinks; i++) {
    digitalWrite(LED_BUILTIN, !led_idle); delay(120);
    digitalWrite(LED_BUILTIN, led_idle);  delay(120);
  }
}

static void led_service() {
  if (led_lit && (int32_t)(millis() - led_off_at) >= 0) {
    digitalWrite(LED_BUILTIN, led_idle);
    led_lit = false;
  }
}

// USB-MIDI packet check: cable 0, CIN consistent with the bytes, data bytes < 0x80.
// Passes channel voice messages, SysEx (any length), song position/select,
// MTC quarter frame, tune request and clock/start/continue/stop/active sensing.
static inline bool d7(uint8_t b)      { return b < 0x80; }               // data byte
static inline bool sx_head(uint8_t b) { return b == 0xF0 || d7(b); }     // SysEx start or data

static bool valid_packet(const uint8_t *p) {
  if (p[0] >> 4) return false;                    // cable number must be 0
  const uint8_t cin = p[0] & 0x0F;
  switch (cin) {
    case 0x8: case 0x9: case 0xA: case 0xB: case 0xE:      // 3-byte voice messages
      return (p[1] >> 4) == cin && d7(p[2]) && d7(p[3]);
    case 0xC: case 0xD:                                    // program change, channel pressure
      return (p[1] >> 4) == cin && d7(p[2]);
    case 0x4:                                              // SysEx start / continue (3 bytes)
      return sx_head(p[1]) && d7(p[2]) && d7(p[3]);
    case 0x5:                                              // SysEx end (1 byte) or 1-byte common
      return p[1] == 0xF7 || p[1] == 0xF6;
    case 0x6:                                              // SysEx ends with 2 bytes
      return sx_head(p[1]) && p[2] == 0xF7;
    case 0x7:                                              // SysEx ends with 3 bytes
      return sx_head(p[1]) && d7(p[2]) && p[3] == 0xF7;
    case 0x2:                                              // 2-byte common: MTC QF, song select
      return (p[1] == 0xF1 || p[1] == 0xF3) && d7(p[2]);
    case 0x3:                                              // 3-byte common: song position
      return p[1] == 0xF2 && d7(p[2]) && d7(p[3]);
    case 0xF:                                              // single-byte realtime
      return p[1] == 0xF8 || p[1] == 0xFA || p[1] == 0xFB || p[1] == 0xFC || p[1] == 0xFE;
    default:
      return false;
  }
}

static void uart_to_usb() {
  bool sent = false;
  while (Serial1.available()) {
    const uint32_t now = micros();
    if (buf_n && (now - last_byte_us) > BYTE_GAP_US) buf_n = 0;   // stale partial packet
    last_byte_us = now;
    buf[buf_n++] = (uint8_t)Serial1.read();
    if (buf_n < 4) continue;
    if (valid_packet(buf)) {
      midiEventPacket_t pkt = { buf[0], buf[1], buf[2], buf[3] };
      MidiUSB.sendMIDI(pkt);
      sent = true;
      buf_n = 0;
      led_pulse();
    } else {                                      // resync: drop oldest byte, keep the rest
      buf[0] = buf[1]; buf[1] = buf[2]; buf[2] = buf[3];
      buf_n = 3;
    }
  }
  if (sent) MidiUSB.flush();
}

static void usb_to_uart() {
  for (;;) {
    midiEventPacket_t rx = MidiUSB.read();
    if (rx.header == 0) break;                    // nothing (more) to read
    const uint8_t p[4] = { rx.header, rx.byte1, rx.byte2, rx.byte3 };
    if (!valid_packet(p)) continue;
    Serial1.write(p, 4);
    led_pulse();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, led_idle);
  show_reset_cause();
  persist_load();
  Serial1.begin(UART_BAUD);
  // Pull-up on RX (pin stays muxed to the SERCOM; PULLEN + OUT=1 selects pull-up).
  const PinDescription &rxp = g_APinDescription[PIN_SERIAL1_RX];
  PORT->Group[rxp.ulPort].OUTSET.reg = (1ul << rxp.ulPin);
  PORT->Group[rxp.ulPort].PINCFG[rxp.ulPin].bit.PULLEN = 1;
  usb_ok_at = millis();
  wdt_start();
}

void loop() {
  wdt_feed();
  usb_health();
  heartbeat();
  usb_to_uart();
  uart_to_usb();
  led_service();
}
