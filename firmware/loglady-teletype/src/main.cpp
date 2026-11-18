// loglady — the Pi's exclusive link to Teletype.
//
// USB (to Teletype, which also powers this board):
//   interface 0   boot keyboard, 8-byte reports, 8-byte endpoint, 1 ms interval  (keyboard_forward.py)
//   interface 1+2 CDC ACM serial = the grid                                       (grid_macros.py)
//   Order matters: Teletype's CDC driver only takes the device when the CDC data interface is last.
//   An 8-byte report on an 8-byte endpoint is a full packet: no short packet / ZLP (Teletype's host
//   doesn't re-arm after those — the reason the Pi needed a patched usb_f_hid).
//
// UART (to the Pi, Serial1: D6 = TX, D7 = RX, LINK_BAUD):
//   frames = COBS(payload) 0x00,  payload = type, data..., crc8(type+data)
//   Pi -> loglady: 0x01 grid bytes to Teletype · 0x03 keyboard report (8) · 0x10 hello · 0x11 replug
//                  0x12 ping
//   loglady -> Pi: 0x02 grid bytes from Teletype · 0x20 ident "loglady teletype <ver>" · 0x21 status
//
// The ident answer is what makes loglady exclusive: the Pi only uses a board that answers it.
#include <Arduino.h>
#define private public            // the library resets the device class in clearConfiguration();
#include <Adafruit_TinyUSB.h>      // we need it back to "misc/IAD" like the Pi's gadget had
#undef private

#ifndef LINK_BAUD
#define LINK_BAUD 1000000
#endif

static const char IDENT[] = "loglady teletype 1";

// ── HID boot keyboard (same report descriptor as the Pi's gadget) ──────────────────────────────────
static const uint8_t desc_kbd[] = {
  0x05,0x01,0x09,0x06,0xa1,0x01,0x05,0x07,0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x08,
  0x81,0x02,0x95,0x01,0x75,0x08,0x81,0x03,0x95,0x05,0x75,0x01,0x05,0x08,0x19,0x01,0x29,0x05,0x91,0x02,
  0x95,0x01,0x75,0x03,0x91,0x03,0x95,0x06,0x75,0x08,0x15,0x00,0x25,0x65,0x05,0x07,0x19,0x00,0x29,0x65,
  0x81,0x00,0xc0 };

class KbdHID : public Adafruit_USBD_HID {
public:
  KbdHID() : Adafruit_USBD_HID(desc_kbd, sizeof(desc_kbd), HID_ITF_PROTOCOL_KEYBOARD, 1, false) {}
  uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf, uint16_t bufsize) override {
    // endpoint size 8 (library default is 64 → short packets), interval 1 ms
    uint8_t const d[] = { TUD_HID_DESCRIPTOR(itfnum, 0, HID_ITF_PROTOCOL_KEYBOARD, sizeof(desc_kbd),
                                             0x80, 8, 1) };
    if (bufsize < sizeof(d)) return 0;
    memcpy(buf, d, sizeof(d));
    return sizeof(d);
  }
};
static KbdHID kbd;

// ── keyboard report queue: one report per poll, never drop a release ───────────────────────────────
static const uint8_t KQ = 64;
static uint8_t kq[KQ][8];
static uint8_t kq_head = 0, kq_tail = 0;
static uint16_t kq_dropped = 0;
static bool kq_push(const uint8_t *r) {
  uint8_t next = (kq_head + 1) % KQ;
  if (next == kq_tail) { kq_dropped++; return false; }
  memcpy(kq[kq_head], r, 8); kq_head = next; return true;
}
static void kbd_service() {
  if (kq_head == kq_tail || !tud_mounted() || !tud_hid_ready()) return;
  if (tud_hid_report(0, kq[kq_tail], 8)) kq_tail = (kq_tail + 1) % KQ;
}

// ── crc8 + COBS ─────────────────────────────────────────────────────────────────────────────────
static uint8_t crc8(const uint8_t *p, size_t n) {
  uint8_t c = 0;
  while (n--) { c ^= *p++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
  return c;
}

static void send_frame(uint8_t type, const uint8_t *data, size_t n) {
  static uint8_t raw[260], enc[270];
  if (n > 250) n = 250;
  raw[0] = type; memcpy(raw + 1, data, n); raw[n + 1] = crc8(raw, n + 1);
  size_t len = n + 2, o = 1, code_at = 0; uint8_t code = 1;
  for (size_t i = 0; i < len; i++) {
    if (raw[i] == 0) { enc[code_at] = code; code_at = o++; code = 1; }
    else { enc[o++] = raw[i]; if (++code == 0xFF) { enc[code_at] = code; code_at = o++; code = 1; } }
  }
  enc[code_at] = code; enc[o++] = 0;
  Serial1.write(enc, o);
}

static size_t cobs_decode(const uint8_t *in, size_t n, uint8_t *out) {
  size_t i = 0, o = 0;
  while (i < n) {
    uint8_t code = in[i++];
    if (code == 0 || i + code - 1 > n) return 0;
    for (uint8_t k = 1; k < code; k++) out[o++] = in[i++];
    if (code != 0xFF && i < n) out[o++] = 0;
  }
  return o;
}

// ── state, status ───────────────────────────────────────────────────────────────────────────────
static uint16_t bad_frames = 0;
static uint32_t last_status = 0;
static uint8_t last_sig = 0xFF;

static void send_status(bool force) {
  uint8_t s[8];
  s[0] = tud_mounted();
  s[1] = tud_cdc_connected();
  s[2] = (uint8_t)((kq_head + KQ - kq_tail) % KQ);
  s[3] = tud_hid_ready();
  s[4] = bad_frames & 0xFF; s[5] = bad_frames >> 8;
  s[6] = kq_dropped & 0xFF; s[7] = kq_dropped >> 8;
  uint8_t sig = s[0] | (s[1] << 1);
  if (!force && sig == last_sig && millis() - last_status < 1000) return;
  last_sig = sig; last_status = millis();
  send_frame(0x21, s, sizeof(s));
}

// Write everything to the CDC FIFO (256 bytes): wait for room while the host reads, so LED-map
// bursts aren't cut off. Gives up after 50 ms without progress (host not reading) and drops the rest.
static uint16_t grid_dropped = 0;
static void cdc_write_all(const uint8_t *d, size_t n) {
  if (!tud_mounted()) return;
  uint32_t last_progress = millis();
  while (n) {
    uint32_t w = tud_cdc_write(d, n);
    if (w) { d += w; n -= w; last_progress = millis(); }
    else {
      tud_cdc_write_flush();
      tud_task();
      if (millis() - last_progress > 50) { grid_dropped += n; break; }
    }
  }
  tud_cdc_write_flush();
}

static void replug() {
  TinyUSBDevice.detach();
  uint32_t t = millis();
  while (millis() - t < 400) { if (!WDT->STATUS.bit.SYNCBUSY) WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY; }
  kq_head = kq_tail = 0;
  TinyUSBDevice.attach();
}

static void handle_frame(const uint8_t *p, size_t n) {
  if (n < 2 || crc8(p, n - 1) != p[n - 1]) { bad_frames++; return; }
  const uint8_t type = p[0]; const uint8_t *d = p + 1; const size_t dn = n - 2;
  switch (type) {
    case 0x01: cdc_write_all(d, dn); break;      // grid bytes -> Teletype
    case 0x03: if (dn == 8) kq_push(d); break;   // keyboard report
    case 0x10: send_frame(0x20, (const uint8_t *)IDENT, sizeof(IDENT) - 1); send_status(true); break;
    case 0x11: replug(); break;
    case 0x12: send_status(true); break;
    default: bad_frames++;
  }
}

static uint8_t rx_buf[300]; static size_t rx_n = 0; static bool rx_overflow = false;
static void uart_service() {
  static uint8_t dec[300];
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (b == 0) {
      if (!rx_overflow && rx_n) { size_t n = cobs_decode(rx_buf, rx_n, dec); if (n) handle_frame(dec, n); else bad_frames++; }
      rx_n = 0; rx_overflow = false;
    } else if (rx_n < sizeof(rx_buf)) rx_buf[rx_n++] = b;
    else rx_overflow = true;
  }
}

static void cdc_service() {
  uint8_t buf[64];
  while (tud_cdc_available()) {
    uint32_t n = tud_cdc_read(buf, sizeof(buf));
    if (!n) break;
    send_frame(0x02, buf, n);
  }
}

// ── watchdog (2 s), same setup as xiao-midi-bridge ─────────────────────────────────────────────────
static void wdt_start() {
  GCLK->GENDIV.reg  = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY) {}
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;
  WDT->CTRL.reg = 0; while (WDT->STATUS.bit.SYNCBUSY) {}
  WDT->CONFIG.reg = WDT_CONFIG_PER_2K;
  WDT->CTRL.reg = WDT_CTRL_ENABLE; while (WDT->STATUS.bit.SYNCBUSY) {}
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH);   // HIGH = off on XIAO
  TinyUSBDevice.setManufacturerDescriptor("ahab");
  TinyUSBDevice.setProductDescriptor("loglady");
  TinyUSBDevice.setSerialDescriptor("loglady-tt-1");

  // rebuild the configuration: keyboard first, CDC (grid) last
  SerialTinyUSB.end();                           // clears the configuration (removes the default CDC)
  kbd.begin();
  SerialTinyUSB.begin(115200);
  TinyUSBDevice._desc_device.bDeviceClass    = TUSB_CLASS_MISC;
  TinyUSBDevice._desc_device.bDeviceSubClass = MISC_SUBCLASS_COMMON;
  TinyUSBDevice._desc_device.bDeviceProtocol = MISC_PROTOCOL_IAD;
  TinyUSBDevice.detach(); delay(50); TinyUSBDevice.attach();       // host re-reads the descriptors

  Serial1.begin(LINK_BAUD);
  wdt_start();
  send_frame(0x20, (const uint8_t *)IDENT, sizeof(IDENT) - 1);       // announce after a reset
}

void loop() {
  if (!WDT->STATUS.bit.SYNCBUSY) WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif
  uart_service();
  cdc_service();
  kbd_service();
  send_status(false);
  digitalWrite(LED_BUILTIN, tud_mounted() ? LOW : HIGH);
}
