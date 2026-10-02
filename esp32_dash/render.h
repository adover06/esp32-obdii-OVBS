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
#pragma once

class StripRenderer {
public:
  static constexpr int STRIP_H = 24;   // 240 / 24 = 10 bands per frame

  // target = the TV (or a desktop window). Returns false if out of memory.
  bool begin(lgfx::LovyanGFX* target, int width, int height)
  {
    _target = target;
    _w = width;
    _h = height;
    size_t bytes = (size_t)width * STRIP_H;
    // Guard bytes on each side of the strip catch drawing that escapes the
    // clip rectangle. Test builds (STRIP_GUARD) use a whole frame on each side
    // so every stray write is caught; the ESP32 uses 64 bytes as an alarm.
#ifdef STRIP_GUARD
    _guardBytes = (size_t)width * height;
#else
    _guardBytes = 64;
#endif
    _alloc = (uint8_t*)malloc(_guardBytes * 2 + bytes);
    if (!_alloc) return false;
    memset(_alloc, GUARD, _guardBytes * 2 + bytes);
    _buf = _alloc + _guardBytes;
    memset(_buf, 0, bytes);
    _band.setColorDepth(8);
    _band.setBuffer(_buf, width, STRIP_H, 8);
    _view.setColorDepth(8);
    return true;
  }

  size_t bufferBytes() const { return (size_t)_w * STRIP_H; }

  static constexpr uint8_t GUARD = 0xA5;
  // Number of guard bytes that were overwritten; reports the first one.
  size_t guardDamage(long* firstOffset) const
  {
    size_t bad = 0;
    *firstOffset = 0;
    size_t total = _guardBytes * 2 + bufferBytes();
    for (size_t i = 0; i < total; i++) {
      if (i >= _guardBytes && i < _guardBytes + bufferBytes()) continue;
      if (_alloc[i] != GUARD) { if (!bad) *firstOffset = (long)i - (long)_guardBytes; bad++; }
    }
    return bad;
  }

  // Put the guard bytes back (after reporting damage), so the next frame is checked fresh.
  void resetGuards()
  {
    memset(_alloc, GUARD, _guardBytes);
    memset(_buf + bufferBytes(), GUARD, _guardBytes);
  }

  // draw(gfx) draws one full frame; it's called once per band.
  template <typename DrawFn>
  void render(DrawFn draw)
  {
    if (!_buf) return;   // begin() failed: draw nothing rather than write through a null buffer
    for (int y0 = 0; y0 < _h; y0 += STRIP_H) {
      int rows = _h - y0 < STRIP_H ? _h - y0 : STRIP_H;
      // Row y of the virtual frame lives at _buf + (y - y0) * _w.
      // Only rows y0 .. y0+rows-1 are ever touched (clip rect below).
      _view.setBuffer(_buf - (ptrdiff_t)y0 * _w, _w, _h, 8);
      _view.setClipRect(0, y0, _w, rows);
      draw(_view);
      _band.pushSprite(_target, 0, y0);
    }
  }

private:
  lgfx::LovyanGFX* _target = nullptr;
  int _w = 0, _h = 0;
  uint8_t* _buf = nullptr;
  uint8_t* _alloc = nullptr;
  size_t _guardBytes = 0;
  LGFX_Sprite _band;   // the real STRIP_H-row buffer, pushed to the TV
  LGFX_Sprite _view;   // full-size window onto it, clipped to one band
};
