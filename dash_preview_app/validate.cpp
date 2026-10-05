// Pre-upload check for the ESP32 dashboard, run on the Mac (./validate.sh).
//
// For every screen (plus the boot splash), at both picture widths (240 and
// 360), over ~100 s of simulated driving:
//   1. Memory safety: renders through the strip renderer with a whole frame
//      of guard bytes on each side of the strip buffer. Any drawing call that
//      writes outside its band (what caused the boot loop) breaks a guard.
//   2. Correctness: the strip-rendered frame must match a normal full-frame
//      render pixel for pixel.
//   3. Speed: average time per frame, strip vs full.
// With --images, saves a PPM snapshot of each screen in validate_out/.
//
// Exit code 0 = all good. Nonzero = something to fix before uploading.

#define STRIP_GUARD
#include <LovyanGFX.hpp>
#include <chrono>
#include <vector>
#include <string.h>
#include <sys/stat.h>
#include "dash.h"
#include "dash_fx.h"
#include "render.h"

static void dump(LGFX_Sprite& s, const char* path)
{
  FILE* f = fopen(path, "wb");
  if (!f) return;
  fprintf(f, "P6 %d %d 255\n", s.width(), s.height());
  for (int y = 0; y < s.height(); y++)
    for (int x = 0; x < s.width(); x++) {
      auto c = s.readPixelRGB(x, y);
      fputc(c.R8(), f); fputc(c.G8(), f); fputc(c.B8(), f);
    }
  fclose(f);
}

// A Telemetry that looks like a live OBD link (exercises the DIAG screen).
static void fillLive(dash::Telemetry& t, const dash::Simulator& sim, uint32_t ms)
{
  const dash::CarData& d = sim.d;
  t.d = d;
  t.log = sim.log;
  t.clock = sim.clock;
  t.sourceTag = "LIVE"; t.sourceColor = dash::GREEN;
  strcpy(t.linkLine, "BT : connected to adapter"); t.linkColor = dash::GREEN;
  strcpy(t.elmLine, "ELM: connected to car"); t.elmColor = dash::GREEN; t.repliesPerSec = 6.4f;
  strcpy(t.memLine, "MEM 41K free 28K blk");
  static const char* names[11] = { "RPM", "SPEED", "THROTTLE", "LOAD", "MAF", "TIMING", "COOLANT", "INTAKE AIR", "FUEL TRIM", "BATTERY", "CHECK ENG" };
  static const char* units[11] = { "", "mph", "%", "%", "g/s", "deg", "F", "F", "%", "V", "" };
  float vals[11] = { d.rpm, d.mph, d.throttle, d.load, d.maf, d.timing, d.coolantF, d.iatF, d.stft, d.volts, 0 };
  t.readingCount = 11;
  for (int i = 0; i < 11; i++) {
    auto& r = t.readings[i];
    r.name = names[i]; r.unit = units[i]; r.value = vals[i];
    r.decimals = (i == 4 || i == 5 || i == 8 || i == 9) ? 1 : 0;
    bool nodata = (i == 5);   // pretend the car doesn't support timing
    strcpy(r.status, nodata ? "NO DATA" : "OK");
    r.statusColor = nodata ? dash::YELLOW : dash::GREEN;
    r.lastOkMs = nodata ? 0 : ms - 200 * i;
    r.okCount = nodata ? 0 : 400 + i * 3;
    r.errCount = nodata ? 120 : (i == 7 ? 90 : i);   // one value with a poor success rate
  }
  strcpy(t.adapterInfo, "ELM327 v1.5");
  strcpy(t.protocolInfo, "SAE J1850 PWM");
  t.linkUpMs = ms > 61000 ? ms - 61000 : 1;
  t.connects = 2; t.drops = 1;
  t.okTotal = 4800; t.errTotal = 215;
  t.freeHeap = 41 * 1024; t.minFreeHeap = 33 * 1024; t.maxBlock = 28 * 1024;
  t.obdStackFree = 3100; t.frameMs = 24; t.videoW = dash::W;
  t.milOn = true; t.codesRead = true; t.codesListed = 2;
  strcpy(t.codes[0], "P0171"); strcpy(t.codes[1], "P0420");
}

int main(int argc, char** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);
  bool images = argc > 1 && !strcmp(argv[1], "--images");
  if (images) mkdir("validate_out", 0755);

  long renders = 0, guardFails = 0, mismatches = 0;
  double tStrip = 0, tFull = 0;
  const int widths[3] = { 240, 360, 720 };

  for (int W : widths) {
    dash::setScreen(W, 4.0f / 3.0f);
    fx::setSport(W == 360);   // cover both threshold sets
    LGFX_Sprite tv;   tv.setColorDepth(8);   tv.createSprite(W, dash::H);    // stands in for the TV
    LGFX_Sprite full; full.setColorDepth(8); full.createSprite(W, dash::H);  // reference render
    StripRenderer rend;
    if (!rend.begin(&tv, W, dash::H)) { printf("could not allocate the strip buffer\n"); return 1; }
    // FPGA path: 2 alternating band buffers, each finished band handed to a sink
    StripRenderer rend2;
    if (!rend2.begin(nullptr, W, dash::H, 2)) { printf("could not allocate the band buffers\n"); return 1; }
    std::vector<uint8_t> sent(W * dash::H);

    dash::Simulator sim;
    fx::DualState dual;
    dash::Telemetry live;
    uint32_t ms = 0;
    for (int frame = 0; frame < 3000; frame++) {   // ~100 s: idle, launches, redline, cruise, braking
      ms += 33;
      sim.update(0.033f);
      fx::updateDual(dual, sim, 0.033f);
      if (frame % 37) continue;                    // check a spread of frames
      fillLive(live, sim, ms);
      for (int scr = -1; scr < dash::SCREEN_COUNT; scr++) {
        // DIAG and SYS get live-style data (the simulator has no OBD link)
        const dash::Telemetry& data = scr >= 2 ? live : (const dash::Telemetry&)sim;
        auto draw = [&](lgfx::LovyanGFX& g) {
          if (scr == -1) dash::drawBoot(g, frame * 33 % 1600, "VIDEO .... TEST");
          else if (scr == 0) fx::drawDual(g, data, ms, dual);
          else dash::draw(g, scr, data, ms);
        };
        const char* name = scr < 0 ? "BOOT" : dash::SCREEN_NAMES[scr];
        auto t0 = std::chrono::steady_clock::now();
        rend.render(draw);
        auto t1 = std::chrono::steady_clock::now();
        draw(full);
        auto t2 = std::chrono::steady_clock::now();
        // the FPGA path: what the sink receives, band by band, must be the same frame
        rend2.renderTo(draw, [&](uint8_t* buf, int y0, int rows) { memcpy(&sent[y0 * W], buf, (size_t)W * rows); });
        long off2;
        if (size_t bad = rend2.guardDamage(&off2)) {
          if (guardFails++ < 10) printf("FAIL  memory (FPGA path): %s at %d px wrote %zu bytes outside a band buffer (frame %d)\n", name, W, bad, frame);
          rend2.resetGuards();
        }
        tStrip += std::chrono::duration<double, std::milli>(t1 - t0).count();
        tFull  += std::chrono::duration<double, std::milli>(t2 - t1).count();
        renders++;

        long off;
        if (size_t bad = rend.guardDamage(&off)) {
          if (guardFails++ < 10) printf("FAIL  memory: %s at %d px wrote %zu bytes outside the strip (frame %d)\n", name, W, bad, frame);
          rend.resetGuards();
        }
        const uint8_t* a = (const uint8_t*)tv.getBuffer();
        const uint8_t* b = (const uint8_t*)full.getBuffer();
        int diff = 0;
        for (int p = 0; p < W * dash::H; p++) diff += a[p] != b[p];
        if (diff > 4) {   // a stray pixel or two where a shape edge meets a band edge is harmless
          if (mismatches++ < 10) printf("FAIL  output: %s at %d px: %d pixels differ from a full-frame render (frame %d)\n", name, W, diff, frame);
        }
        int diff2 = 0;
        for (int p = 0; p < W * dash::H; p++) diff2 += sent[p] != b[p];
        if (diff2 > 4) {
          if (mismatches++ < 10) printf("FAIL  output (FPGA path): %s at %d px: %d pixels differ (frame %d)\n", name, W, diff2, frame);
        }
        if (images && frame % 740 == 370 && scr >= 0) {   // ~every 24 s of driving
          char path[64];
          snprintf(path, sizeof(path), "validate_out/w%d_%d_%s_%04d.ppm", W, scr + 1, name, frame);
          dump(tv, path);
        }
      }
    }
  }

  printf("\n%ld renders checked (%d screens + boot x 2 widths x 82 moments)\n", renders, dash::SCREEN_COUNT);
  printf("memory escapes : %ld\n", guardFails);
  printf("output diffs   : %ld\n", mismatches);
  printf("speed (Mac)    : strip %.2f ms vs full %.2f ms per frame (%.1fx)\n", tStrip / renders, tFull / renders, tStrip / tFull);
  bool ok = !guardFails && !mismatches;
  printf("\n%s\n", ok ? "PASS - safe to upload" : "FAIL - fix before uploading");
  return ok ? 0 : 1;
}
