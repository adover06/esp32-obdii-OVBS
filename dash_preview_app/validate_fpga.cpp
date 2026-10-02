// Mac check for esp32_fpga_test/scene.h, drawn exactly the way the ESP32 does
// it (10 bands of 24 rows through a "view" sprite), but with a whole frame of
// guard bytes on each side of the band buffer. Any drawing that escapes the
// band (memory corruption on the ESP32) changes a guard byte and fails.
// Each banded frame must also match a normal full-frame drawing.
#include <LovyanGFX.hpp>
#include <string.h>
#include "scene.h"

int main()
{
  const int W = scene::W, H = scene::H, BAND = 24;
  const size_t GUARD = (size_t)W * H;
  static uint8_t mem[GUARD + W * BAND + GUARD];
  uint8_t* band = mem + GUARD;
  LGFX_Sprite view;  view.setColorDepth(8);
  LGFX_Sprite full;  full.setColorDepth(8);  full.createSprite(W, H);
  static uint8_t assembled[W * H];

  // ordinary frames, plus extreme values for every field shown on screen
  long frames = 0, escapes = 0, diffs = 0;
  for (int pass = 0; pass < 2; pass++) {
    for (uint32_t f = 0; f < 3000; f += 7) {
      scene::State s;
      s.frame = pass ? 0xFFFFFFFFu - f : f;
      s.fps = pass ? 99999.9f : 29.7f;
      s.mbps = pass ? -1e9f : 2.5f;
      s.mhz = pass ? 1e12f : 20.0f;
      s.flags = (uint8_t)(f * 37);
      s.errors = (f / 7) & 1;
      memset(mem, 0xA5, sizeof(mem));
      for (int y0 = 0; y0 < H; y0 += BAND) {
        view.setBuffer(band - (ptrdiff_t)y0 * W, W, H, 8);
        view.setClipRect(0, y0, W, BAND);
        scene::draw(view, s);
        memcpy(assembled + y0 * W, band, W * BAND);
      }
      for (size_t i = 0; i < GUARD; i++)
        if (mem[i] != 0xA5 || mem[GUARD + W * BAND + i] != 0xA5) { escapes++; break; }
      scene::draw(full, s);
      const uint8_t* ref = (const uint8_t*)full.getBuffer();
      int d = 0;
      for (int i = 0; i < W * H; i++) d += assembled[i] != ref[i];
      if (d > 4) { if (diffs++ < 5) printf("frame %u: %d pixels differ\n", s.frame, d); }
      frames++;
      if (pass == 0 && f == 700) {          // save one frame to look at
        FILE* fp = fopen("validate_out/fpga_scene.ppm", "wb");
        fprintf(fp, "P6 %d %d 255\n", W, H);
        for (int i = 0; i < W * H; i++) {
          uint8_t c = assembled[i];
          fputc((c >> 5) * 255 / 7, fp); fputc(((c >> 2) & 7) * 255 / 7, fp); fputc((c & 3) * 255 / 3, fp);
        }
        fclose(fp);
      }
    }
  }
  printf("%ld frames: %ld memory escapes, %ld output mismatches -> %s\n", frames, escapes, diffs,
         escapes || diffs ? "FAIL" : "PASS");
  return escapes || diffs ? 1 : 0;
}
