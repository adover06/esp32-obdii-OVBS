// Strip renderer: flicker-free frames with a small buffer.
//
// A full off-screen frame (W x 240 bytes) doesn't fit next to the video
// framebuffer and Bluetooth on a WROOM ESP32. Instead, each frame is drawn
// in horizontal bands through one STRIP_H-row buffer:
//
//   for each band:  draw the whole frame, clipped to that band  ->  copy the band to the TV
//
// The TV only ever receives finished pixels, so nothing flickers. Drawing code
// stays in normal full-screen coordinates: `view` is a sprite that pretends to
// be W x 240, but its buffer pointer is shifted so the band's rows land in the
// real buffer, and its clip rectangle stops anything outside the band from
// being written. (Every LovyanGFX write goes through the clip rectangle; the
// desktop preview checks this under AddressSanitizer.)
//
// Drawing callbacks must be stateless: they run once per band.
//
// With the MAX1000 video card, bands go to the FPGA over SPI instead (renderTo).
#pragma once

class StripRenderer {
public:
  static constexpr int STRIP_H = 24;   // default band height: 240 / 24 = 10 bands per frame

  // target = the TV (or a desktop window), or nullptr when bands go to a sink
  // (the FPGA). buffers = 2 lets the next band be drawn while the previous one
  // is still being sent by DMA. Returns false if out of memory.
  // stripH: rows per band. Every band re-runs the whole drawing code (clipped),
  // so taller bands = fewer passes = much faster frames, at the cost of RAM
  // (width x stripH bytes per buffer).
  bool begin(lgfx::LovyanGFX* target, int width, int height, int buffers = 1, int stripH = STRIP_H)
  {
    _target = target;
    _w = width;
    _h = height;
    _sh = stripH < 1 ? 1 : stripH > height ? height : stripH;
    _n = buffers < 1 ? 1 : buffers > 2 ? 2 : buffers;
    size_t bytes = (size_t)width * _sh;
    // Guard bytes on each side of every strip catch drawing that escapes the
    // clip rectangle. Test builds (STRIP_GUARD) use a whole frame on each side
    // so every stray write is caught; the ESP32 uses 64 bytes as an alarm.
#ifdef STRIP_GUARD
    _guardBytes = (size_t)width * height;
#else
    _guardBytes = 64;
#endif
    for (int i = 0; i < _n; i++) {
      size_t total = _guardBytes * 2 + bytes;
#ifdef ESP_PLATFORM
      _alloc[i] = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_DMA);   // SPI DMA can only read internal RAM
#else
      _alloc[i] = (uint8_t*)malloc(total);
#endif
      if (!_alloc[i]) return false;
      memset(_alloc[i], GUARD, total);
      _bufs[i] = _alloc[i] + _guardBytes;
      memset(_bufs[i], 0, bytes);
    }
    _buf = _bufs[0];
    _band.setColorDepth(8);
    _band.setBuffer(_buf, width, _sh, 8);
    _view.setColorDepth(8);
    return true;
  }

  size_t bufferBytes() const { return (size_t)_w * _sh; }
  int stripHeight() const { return _sh; }
  uint8_t* buffer(int i = 0) const { return _bufs[i]; }

  static constexpr uint8_t GUARD = 0xA5;
  // Number of guard bytes that were overwritten (all buffers); reports the first one.
  size_t guardDamage(long* firstOffset) const
  {
    size_t bad = 0;
    *firstOffset = 0;
    size_t total = _guardBytes * 2 + bufferBytes();
    for (int b = 0; b < _n; b++)
      for (size_t i = 0; i < total; i++) {
        if (i >= _guardBytes && i < _guardBytes + bufferBytes()) continue;
        if (_alloc[b][i] != GUARD) { if (!bad) *firstOffset = (long)i - (long)_guardBytes; bad++; }
      }
    return bad;
  }

  // Put the guard bytes back (after reporting damage), so the next frame is checked fresh.
  void resetGuards()
  {
    for (int b = 0; b < _n; b++) {
      memset(_alloc[b], GUARD, _guardBytes);
      memset(_bufs[b] + bufferBytes(), GUARD, _guardBytes);
    }
  }

  // draw(gfx) draws one full frame; it's called once per band. Each finished
  // band is copied to the target (the TV).
  template <typename DrawFn>
  void render(DrawFn draw)
  {
    if (!_buf || !_target) return;   // begin() failed: draw nothing rather than write through a null buffer
    for (int y0 = 0; y0 < _h; y0 += _sh) {
      int rows = _h - y0 < _sh ? _h - y0 : _sh;
      drawBand(_buf, y0, rows, draw);
      _band.pushSprite(_target, 0, y0);
    }
  }

  // Same, but each finished band goes to sink(buffer, y0, rows) - e.g. a DMA
  // transfer to the FPGA. Buffers alternate, so the sink may still be sending
  // band N from one buffer while band N+1 is drawn into the other; the sink
  // must wait for its previous transfer before starting a new one.
  template <typename DrawFn, typename SinkFn>
  void renderTo(DrawFn draw, SinkFn sink)
  {
    if (!_bufs[_n - 1]) return;
    int k = 0;
    for (int y0 = 0; y0 < _h; y0 += _sh, k++) {
      int rows = _h - y0 < _sh ? _h - y0 : _sh;
      uint8_t* buf = _bufs[k % _n];
      drawBand(buf, y0, rows, draw);
      sink(buf, y0, rows);
    }
  }

private:
  template <typename DrawFn>
  void drawBand(uint8_t* buf, int y0, int rows, DrawFn& draw)
  {
    // Row y of the virtual frame lives at buf + (y - y0) * _w.
    // Only rows y0 .. y0+rows-1 are ever touched (clip rect below).
    _view.setBuffer(buf - (ptrdiff_t)y0 * _w, _w, _h, 8);
    _view.setClipRect(0, y0, _w, rows);
    draw(_view);
  }

  lgfx::LovyanGFX* _target = nullptr;
  int _w = 0, _h = 0, _n = 1, _sh = STRIP_H;
  uint8_t* _buf = nullptr;
  uint8_t* _bufs[2] = { nullptr, nullptr };
  uint8_t* _alloc[2] = { nullptr, nullptr };
  size_t _guardBytes = 0;
  LGFX_Sprite _band;   // the real STRIP_H-row buffer 0, pushed to the TV
  LGFX_Sprite _view;   // full-size window onto a band, clipped to that band
};
