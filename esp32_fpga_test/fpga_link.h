// SPI link from the ESP32 to the MAX1000 video card.
//
// Wiring (ESP32 -> MAX1000 MKR header J1, plus a ground wire):
//   GPIO18 SCK  -> D0     GPIO23 MOSI -> D1     GPIO5 CS -> D2
//   GPIO19 MISO <- D3     GPIO4 READY <- D4     GND -> GND (J2 pin 11)
//
// Protocol (see video_card/cmd_parser.v): every transaction starts with
// [cmd, 0, 0, 0].
//   'W' + x, y, w, h (uint16 little endian) + w*h RGB332 bytes -> back buffer
//   'S' -> show the back buffer at the next vertical blank
//   any transaction shifts out the 64-bit status captured at the end of the
//   previous one: A5, flags, frame count, byte count, CRC-16 of its bytes
//
// CS is driven by hand so the 12-byte header and the pixel DMA can share one
// transaction, and the pixel DMA of one band can run while the next band is
// being drawn.
#pragma once
#include <Arduino.h>
#include "driver/spi_master.h"

namespace fpga {

constexpr int PIN_SCK = 18, PIN_MOSI = 23, PIN_MISO = 19, PIN_CS = 5, PIN_READY = 4;
constexpr int WIDTH = 360, HEIGHT = 240;
constexpr uint32_t STATUS_HZ = 8000000;   // MISO reads stay slow and safe

// status flag bits
enum : uint8_t {
  F_READY = 1, F_SELFTEST_DONE = 2, F_VERIFY_ERR = 4, F_FIFO_OVERFLOW = 8,
  F_FETCH_LATE = 16, F_FRONT = 32, F_CHECKING = 64, F_SDRAM_OK = 128
};

struct Status {
  bool valid = false;   // magic byte seen
  uint8_t flags = 0;
  uint16_t frame = 0, count = 0, crc = 0;
  bool errors() const { return flags & (F_VERIFY_ERR | F_FIFO_OVERFLOW | F_FETCH_LATE); }
};

inline uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc = 0xFFFF)
{
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int k = 0; k < 8; k++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

class Link {
public:
  bool begin(uint32_t hz)
  {
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_READY, INPUT_PULLDOWN);
    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_MOSI;
    bus.miso_io_num = PIN_MISO;
    bus.sclk_io_num = PIN_SCK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = WIDTH * 24 + 64;
    if (spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;
    // two "devices" on the same wires: a slow full-duplex one for reading the
    // status word, and a fast write-only one for pixels (see addDevice)
    if (!addDevice(&_slow, STATUS_HZ, false)) return false;
    return setSpeed(hz);
  }

  // Data clock. The FPGA takes up to ~40 MHz; jumper wires usually need less.
  // Returns false (and keeps the old speed) if the driver rejects it.
  bool setSpeed(uint32_t hz)
  {
    finish();
    spi_device_handle_t old = _fast;
    if (old) spi_bus_remove_device(old);
    _fast = nullptr;
    if (addDevice(&_fast, hz, true)) { _hz = hz; return true; }
    if (_hz && addDevice(&_fast, _hz, true)) return false;   // put the previous speed back
    return false;
  }
  uint32_t speed() const { return _hz; }

  // Start sending a rectangle of RGB332 pixels (row by row). Returns at once;
  // `px` must stay untouched until the next call or finish(). px must be in
  // DMA-capable RAM (heap_caps_malloc(..., MALLOC_CAP_DMA)).
  void sendRect(int x, int y, int w, int h, const uint8_t* px)
  {
    finish();
    _hdr[0] = 'W'; _hdr[1] = _hdr[2] = _hdr[3] = 0;
    put16(4, x); put16(6, y); put16(8, w); put16(10, h);
    digitalWrite(PIN_CS, LOW);
    xfer(_fast, _hdr, 12);
    if (w > 0 && h > 0) {
      _t = {};
      _t.length = (size_t)w * h * 8;
      _t.tx_buffer = px;
      spi_device_queue_trans(_fast, &_t, portMAX_DELAY);
      _pending = true;
    } else {
      digitalWrite(PIN_CS, HIGH);
    }
  }

  // Wait for the last sendRect's DMA and end its transaction.
  void finish()
  {
    if (!_pending) return;
    spi_transaction_t* done;
    spi_device_get_trans_result(_fast, &done, portMAX_DELAY);
    digitalWrite(PIN_CS, HIGH);
    _pending = false;
  }

  // Show the back buffer at the next vertical blank.
  void swap()
  {
    finish();
    uint8_t cmd[4] = { 'S', 0, 0, 0 };
    digitalWrite(PIN_CS, LOW);
    xfer(_fast, cmd, 4);
    digitalWrite(PIN_CS, HIGH);
  }

  // READY pin: high when the last swap has happened (safe to draw the next frame).
  bool ready() const { return digitalRead(PIN_READY) == HIGH; }
  bool waitReady(uint32_t timeoutMs)
  {
    uint32_t t0 = millis();
    while (!ready()) {
      if (millis() - t0 > timeoutMs) return false;
      delayMicroseconds(50);
    }
    return true;
  }

  // Status as of the end of the PREVIOUS transaction ('Q' itself is ignored).
  Status status()
  {
    finish();
    alignas(4) uint8_t tx[8] = { 'Q', 0, 0, 0, 0, 0, 0, 0 };
    alignas(4) uint8_t rx[8] = {};
    digitalWrite(PIN_CS, LOW);
    spi_transaction_t t = {};
    t.length = 64;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(_slow, &t);
    digitalWrite(PIN_CS, HIGH);
    delayMicroseconds(2);
    Status s;
    s.valid = rx[0] == 0xA5;
    s.flags = rx[1];
    s.frame = rx[2] << 8 | rx[3];
    s.count = rx[4] << 8 | rx[5];
    s.crc   = rx[6] << 8 | rx[7];
    return s;
  }

  // Link test: send n bytes in an ignored 'X' transaction, then check the
  // FPGA's byte count and CRC. buf must be DMA-capable, n >= 4.
  bool echoTest(uint8_t* buf, size_t n)
  {
    finish();
    buf[0] = 'X'; buf[1] = buf[2] = buf[3] = 0;
    digitalWrite(PIN_CS, LOW);
    xfer(_fast, buf, n);
    digitalWrite(PIN_CS, HIGH);
    delayMicroseconds(2);
    Status s = status();
    return s.valid && s.count == (uint16_t)n && s.crc == crc16(buf, n);
  }

private:
  // writeOnly: half-duplex, MOSI only. The ESP-IDF driver refuses full-duplex
  // devices above ~26.7 MHz (it can't read MISO reliably that fast), but a
  // device that never reads has no such limit, so pixels can go at 40 MHz.
  bool addDevice(spi_device_handle_t* dev, uint32_t hz, bool writeOnly)
  {
    spi_device_interface_config_t cfg = {};
    cfg.mode = 0;
    if (writeOnly) cfg.flags = SPI_DEVICE_HALFDUPLEX;
    cfg.clock_speed_hz = (int)hz;
    cfg.spics_io_num = -1;    // CS by hand
    cfg.queue_size = 2;
    return spi_bus_add_device(SPI3_HOST, &cfg, dev) == ESP_OK;
  }
  void xfer(spi_device_handle_t dev, const uint8_t* p, size_t n)
  {
    spi_transaction_t t = {};
    t.length = n * 8;
    t.tx_buffer = p;
    spi_device_polling_transmit(dev, &t);
  }
  void put16(int i, int v) { _hdr[i] = v & 0xFF; _hdr[i + 1] = (v >> 8) & 0xFF; }

  spi_device_handle_t _fast = nullptr, _slow = nullptr;
  spi_transaction_t _t = {};
  bool _pending = false;
  uint32_t _hz = 0;
  alignas(4) uint8_t _hdr[12] = {};
};

}  // namespace fpga
