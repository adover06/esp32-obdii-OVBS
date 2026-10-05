// Band renderer: flicker-free frames with a small buffer.
//
// The video driver shows whatever is in its picture memory, continuously. If
// we drew straight into it, the TV would catch half-drawn frames (flicker).
// Instead each frame is drawn in horizontal bands through one band buffer:
//
//   for each band:  draw the whole frame, clipped to that band  ->  copy the band to the TV
//
// The TV only ever receives finished pixels. Drawing code stays in normal
// full-screen coordinates: `view` is a sprite that pretends to be the whole
// picture, but its memory pointer is shifted so the band's rows land in the
// real buffer, and its clip rectangle stops anything outside the band from
// being written. (In Python terms: a "view" onto one slice of a big array.)
//
// Drawing callbacks must be stateless: they run once per band.
#pragma once

class StripRenderer {
public:
  // target = the TV. bandRows = rows per band: taller bands = fewer passes
  // = faster frames, but more RAM (width x bandRows bytes).
  bool begin(lgfx::LovyanGFX* target, int width, int height, int bandRows)
  {
    _target = target;
    _w = width;
    _h = height;
    _sh = bandRows < 1 ? 1 : bandRows > height ? height : bandRows;
    // Guard bytes on each side of the buffer catch drawing that escapes the
    // clip rectangle (64 bytes: an alarm, reported by guardDamage()).
    size_t total = GUARD_BYTES * 2 + bufferBytes();
    _alloc = (uint8_t*)malloc(total);
    if (!_alloc) return false;
    memset(_alloc, GUARD, total);
    _buf = _alloc + GUARD_BYTES;
    memset(_buf, 0, bufferBytes());
    _band.setColorDepth(8);
    _band.setBuffer(_buf, width, _sh, 8);
    _view.setColorDepth(8);
    return true;
  }

  size_t bufferBytes() const { return (size_t)_w * _sh; }

  static constexpr uint8_t GUARD = 0xA5;
  static constexpr size_t GUARD_BYTES = 64;

  // Number of guard bytes that were overwritten; reports the first one.
  size_t guardDamage(long* firstOffset) const
  {
    size_t bad = 0;
    *firstOffset = 0;
    size_t total = GUARD_BYTES * 2 + bufferBytes();
    for (size_t i = 0; i < total; i++) {
      if (i >= GUARD_BYTES && i < GUARD_BYTES + bufferBytes()) continue;
      if (_alloc[i] != GUARD) { if (!bad) *firstOffset = (long)i - (long)GUARD_BYTES; bad++; }
    }
    return bad;
  }

  // Put the guard bytes back (after reporting damage), so the next frame is checked fresh.
  void resetGuards()
  {
    memset(_alloc, GUARD, GUARD_BYTES);
    memset(_buf + bufferBytes(), GUARD, GUARD_BYTES);
  }

  // draw(gfx) draws one full frame; it's called once per band.
  template <typename DrawFn>
  void render(DrawFn draw)
  {
    if (!_buf) return;   // begin() failed: draw nothing rather than write through a null pointer
    for (int y0 = 0; y0 < _h; y0 += _sh) {
      int rows = _h - y0 < _sh ? _h - y0 : _sh;
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
  int _w = 0, _h = 0, _sh = 24;
  uint8_t* _buf = nullptr;
  uint8_t* _alloc = nullptr;
  LGFX_Sprite _band;   // the real band buffer, pushed to the TV
  LGFX_Sprite _view;   // full-size window onto it, clipped to one band
};
