// The animated test picture (360x240, RGB332 colours).
//
// Kept in its own file with no ESP32-only code, so the Mac can run the exact
// same drawing code under the memory checker (dash_preview_app/validate_fpga.sh).
//
// Colours are RGB332 bytes: rrrgggbb (3 bits red, 3 green, 2 blue).
// IMPORTANT: LovyanGFX decides the colour format from the C++ TYPE:
// a uint8_t is RGB332, but a plain number like 0x1F is an int, which it reads
// as 24-bit RGB888 (0x00001F = dark blue!). So every colour here is a named
// uint8_t constant. (A Python analogy: as if colour(31) and colour(np.uint8(31))
// gave different colours.)
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

namespace scene {

constexpr int W = 360, H = 240;

constexpr uint8_t BLACK = 0x00, NAVY = 0x05, GREY = 0x29, LIGHT = 0xBB, WHITE = 0xFF;
constexpr uint8_t CYAN = 0x1F, GREEN = 0x3D, YELLOW = 0xFC, ORANGE = 0xF8, RED = 0xE0;

// Everything the picture shows. Like a Python dataclass: plain fields only.
struct State {
  uint32_t frame = 0;   // frame counter, drives the animation
  float fps = 0;        // measured frames per second
  float mbps = 0;       // measured link throughput, MB/s
  float mhz = 0;        // SPI clock in use
  uint8_t flags = 0;    // FPGA status flags (fpga_link.h F_*)
  bool errors = false;  // FPGA reported an error flag
};

// Draws one whole frame. It is called once per 24-row band (10 times per
// frame) with drawing clipped to that band, so it must only DRAW: no
// counters, no state changes, nothing that differs between calls.
//
// Only use LovyanGFX calls that respect the clip rectangle (fillRect, drawRect,
// fillCircle, drawLine, fillArc, drawString...). drawWideLine/drawWedgeLine
// ignore it and would write outside the band buffer (the old boot-loop bug).
inline void draw(lgfx::LovyanGFX& g, const State& s)
{
  const float DEG = 3.14159265f / 180.0f;

  g.fillRect(0, 0, W, H, BLACK);                      // black background

  // title bar
  g.fillRect(0, 0, W, 22, NAVY);
  g.setFont(&fonts::Font2);
  g.setTextSize(1);
  g.setTextColor(CYAN);
  g.drawString("MAX1000 VIDEO CARD  360x240", 8, 3);

  // 8 colour bars sliding left 2 pixels per frame
  static const uint8_t bars[8] = { 0xFF, 0xFC, 0x1F, 0x1C, 0xE3, 0xE0, 0x03, 0x92 };
  int off = (int)(s.frame * 2 % 360);
  for (int i = 0; i < 9; i++) {
    int x = i * 45 - off % 45;
    g.fillRect(x, 26, 45, 40, bars[(i + off / 45) % 8]);
  }

  // every one of the 256 colours, 32 per row
  for (int c = 0; c < 256; c++) g.fillRect(8 + (c % 32) * 11, 72 + (c / 32) * 7, 10, 6, (uint8_t)c);

  // gauge: grey track, coloured fill that sweeps up and down, white needle
  const int cx = 290, cy = 175, r = 48;
  g.fillArc(cx, cy, r, r - 8, 135, 405, GREY);
  float t = (sinf(s.frame * 0.05f) + 1) * 0.5f;      // 0..1
  uint8_t fill = t > 0.8f ? RED : t > 0.6f ? ORANGE : GREEN;
  if (t > 0.004f) g.fillArc(cx, cy, r, r - 8, 135, 135 + 270 * t, fill);
  float a = (135 + 270 * t) * DEG;
  g.drawLine(cx, cy, cx + (int)(cosf(a) * (r - 12)), cy + (int)(sinf(a) * (r - 12)), WHITE);
  g.fillCircle(cx, cy, 4, WHITE);

  // live numbers
  char line[48];
  g.setTextColor(LIGHT);
  snprintf(line, sizeof(line), "frame %lu", (unsigned long)s.frame);
  g.drawString(line, 8, 136);
  snprintf(line, sizeof(line), "%.1f fps   %.2f MB/s", s.fps, s.mbps);
  g.drawString(line, 8, 154);
  snprintf(line, sizeof(line), "SPI %.1f MHz", s.mhz);
  g.drawString(line, 8, 172);
  g.setTextColor(s.errors ? RED : GREEN);
  snprintf(line, sizeof(line), "FPGA %s  flags %02X", s.errors ? "ERROR" : "OK", s.flags);
  g.drawString(line, 8, 190);

  // ball bouncing left and right along the bottom
  int bx = 20 + abs((int)(s.frame * 3 % 400) - 200);
  g.fillCircle(bx, 222, 8, YELLOW);

  // 1-pixel white border: shows how much of the picture the TV crops
  g.drawRect(0, 0, W, H, WHITE);
}

}  // namespace scene
