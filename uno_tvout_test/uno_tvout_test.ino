// Arduino Uno composite video (black & white) test pattern
// Step 1 of the car speed display project.
//
// Wiring (TVout on Uno):
//   Pin 9 --[ 1k  ]--+
//                    +--> RCA center pin (video)
//   Pin 7 --[ 470 ]--+
//   GND ----------------> RCA outer shell
//
// Shows a border to check overscan, some text, and a big counting
// number to confirm the frame is updating live.

#include <TVout.h>
#include <fontALL.h>

// Most US head units want NTSC. Switch to PAL if the picture rolls.
#define VIDEO_MODE NTSC

TVout tv;

// 7-segment digit built from rectangles so it can be any size.
// Segment order: a (top), b, c, d (bottom), e, f, g (middle)
const uint8_t SEGMENTS[10] = {
  0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110,
  0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111
};

void drawDigit(uint8_t x, uint8_t y, uint8_t digit, uint8_t w, uint8_t h, uint8_t t)
{
  // Clear the digit area first
  tv.draw_rect(x, y, w, h, BLACK, BLACK);
  uint8_t s = SEGMENTS[digit];
  uint8_t mid = y + (h - t) / 2;
  if (s & 0x01) tv.draw_rect(x, y, w - 1, t - 1, WHITE, WHITE);                 // a
  if (s & 0x02) tv.draw_rect(x + w - t, y, t - 1, h / 2, WHITE, WHITE);         // b
  if (s & 0x04) tv.draw_rect(x + w - t, mid, t - 1, h / 2, WHITE, WHITE);       // c
  if (s & 0x08) tv.draw_rect(x, y + h - t, w - 1, t - 1, WHITE, WHITE);         // d
  if (s & 0x10) tv.draw_rect(x, mid, t - 1, h / 2, WHITE, WHITE);               // e
  if (s & 0x20) tv.draw_rect(x, y, t - 1, h / 2, WHITE, WHITE);                 // f
  if (s & 0x40) tv.draw_rect(x, mid, w - 1, t - 1, WHITE, WHITE);               // g
}

void drawNumber(uint8_t x, uint8_t y, uint16_t value)
{
  const uint8_t w = 24, h = 44, t = 5, gap = 6;
  uint8_t digits[3] = { (uint8_t)(value / 100 % 10), (uint8_t)(value / 10 % 10), (uint8_t)(value % 10) };
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t dx = x + i * (w + gap);
    // Blank leading zeros
    if ((i == 0 && value < 100) || (i == 1 && value < 10)) {
      tv.draw_rect(dx, y, w, h, BLACK, BLACK);
    } else {
      drawDigit(dx, y, digits[i], w, h, t);
    }
  }
}

void setup()
{
  tv.begin(VIDEO_MODE, 120, 96);
  tv.clear_screen();

  // Border lines: if you can't see all four edges, the head unit is
  // cropping (overscan) and we'll need to pad the layout later.
  tv.draw_rect(0, 0, tv.hres() - 1, tv.vres() - 1, WHITE);
  tv.draw_rect(4, 4, tv.hres() - 9, tv.vres() - 9, WHITE);

  tv.select_font(font6x8);
  tv.print(10, 10, "UNO CVBS OK");
  tv.select_font(font4x6);
  tv.print(80, 76, "MPH");
}

void loop()
{
  static uint16_t counter = 0;

  // Fake "speed" that counts 0-199 so we can see live updates
  drawNumber(10, 26, counter % 200);
  counter++;
  tv.delay_frame(6);  // ~10 updates per second
}
