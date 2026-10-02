// ESP32 -> MAX1000 video card test.
//
// What it does, in order:
//   1. Waits for the FPGA to answer over SPI and prints its status flags
//      (the FPGA's power-up SDRAM self test must have finished).
//   2. Link test: at each SPI speed, sends 20 blocks of 4 KB of test data and
//      asks the FPGA how many bytes it got and their CRC (a checksum). If both
//      match what we sent, that speed is clean. Uses the fastest clean speed
//      whose next-slower speed is also clean (a little safety margin).
//   3. Forever: draws an animated 360x240 picture (scene.h) and streams it.
//
// How a frame is sent (see fpga_link.h for the wire protocol):
//   - The picture is drawn in 10 horizontal bands of 24 rows, because a whole
//     frame (86 KB) is more RAM than we want to spend. Two band buffers take
//     turns: while the DMA hardware sends band N from one buffer, the CPU
//     draws band N+1 into the other.
//   - After the 10th band we send a "swap" command. The FPGA shows the new
//     frame at the TV's next vertical blank, so a frame is never half shown.
//   - Before drawing the next frame we wait for the FPGA's READY pin, which
//     goes high once that swap has happened.
//
// Wiring: see fpga_link.h. Board: ESP32-WROOM-DA. No Bluetooth here.
// Serial monitor (115200 baud): t = rerun link test, 1-7 = force a speed,
// p = pause/resume.

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "fpga_link.h"
#include "scene.h"

// SPI speeds to try, slowest first. The ESP32 can only make 80 MHz / n.
static const uint32_t SPEEDS[] = { 8000000, 10000000, 13333333, 16000000, 20000000, 26666666, 40000000 };
static const int NSPEEDS = sizeof(SPEEDS) / sizeof(SPEEDS[0]);
constexpr int W = fpga::WIDTH, H = fpga::HEIGHT, BAND = 24;

fpga::Link fpgaLink;       // (not called "link": that name is taken by a C library function)
uint8_t* bandBuf[2];       // the two band buffers, 360 x 24 bytes each, DMA-capable RAM
LGFX_Sprite view;          // a "window" that makes one band look like a full frame (see sendFrame)
scene::State state;        // what the picture shows (frame number, fps, ...)
bool paused = false;

void printStatus(const fpga::Status& s)
{
  Serial.printf("FPGA: %s  flags %02X [%s%s%s%s%s%s%s]  frame %u\n", s.valid ? "OK" : "no answer", s.flags,
                s.flags & fpga::F_SDRAM_OK ? "sdram " : "",
                s.flags & fpga::F_SELFTEST_DONE ? "selftest-written " : "",
                s.flags & fpga::F_CHECKING ? "checking " : "",
                s.flags & fpga::F_READY ? "ready " : "",
                s.flags & fpga::F_VERIFY_ERR ? "SDRAM-DATA-ERROR " : "",
                s.flags & fpga::F_FIFO_OVERFLOW ? "FIFO-OVERFLOW " : "",
                s.flags & fpga::F_FETCH_LATE ? "FETCH-LATE " : "", s.frame);
}

// Step 2: find the fastest SPI speed that gets every byte through intact.
void linkTest()
{
  Serial.println("\nLink test (20 x 4 KB per speed, byte count + CRC checked by the FPGA):");
  uint8_t* buf = bandBuf[0];     // borrow a band buffer as test data
  bool pass[NSPEEDS];
  uint32_t rng = 12345;          // simple pseudo-random generator (same data every run)
  for (int i = 0; i < NSPEEDS; i++) {
    pass[i] = false;
    if (!fpgaLink.setSpeed(SPEEDS[i])) {
      Serial.printf("  %5.1f MHz: driver refused this speed\n", SPEEDS[i] / 1e6);
      continue;
    }
    int bad = 0;
    for (int n = 0; n < 20; n++) {
      for (int k = 0; k < 4096; k++) { rng = rng * 1103515245 + 12345; buf[k] = rng >> 16; }
      if (n == 0) {              // also long runs of 1s and 0s, which stress the wiring differently
        memset(buf + 4, 0xFF, 64);
        memset(buf + 68, 0x00, 64);
      }
      if (!fpgaLink.echoTest(buf, 4096)) bad++;
    }
    pass[i] = (bad == 0);
    if (bad) Serial.printf("  %5.1f MHz: %d/20 FAILED\n", SPEEDS[i] / 1e6, bad);
    else     Serial.printf("  %5.1f MHz: ok\n", SPEEDS[i] / 1e6);
  }
  // fastest speed that passed AND whose slower neighbour passed
  int best = -1;
  for (int i = NSPEEDS - 1; i >= 0; i--)
    if (pass[i] && (i == 0 || pass[i - 1])) { best = i; break; }
  if (best < 0) {
    Serial.println("No speed passed. Check wiring (SCK/MOSI/MISO/CS/GND), keep wires short.");
    best = 0;
  }
  fpgaLink.setSpeed(SPEEDS[best]);
  Serial.printf("Using %.1f MHz\n", SPEEDS[best] / 1e6);
}

// Step 3: draw and send one frame, band by band.
void sendFrame()
{
  for (int b = 0; b < H / BAND; b++) {
    int y0 = b * BAND;                 // first row of this band
    uint8_t* buf = bandBuf[b & 1];     // alternate between the two buffers

    // `view` pretends to be a full 360x240 picture whose memory starts
    // y0 rows BEFORE our band buffer, so row y lands at buf + (y - y0) * W.
    // The clip rectangle limits drawing to rows y0..y0+23, the only rows
    // that are real memory. That way scene::draw uses normal screen
    // coordinates and doesn't know about bands at all.
    view.setBuffer(buf - (ptrdiff_t)y0 * W, W, H, 8);
    view.setClipRect(0, y0, W, BAND);
    scene::draw(view, state);

    // Starts the DMA and returns at once. It first waits for the previous
    // band's DMA, which is why we alternate buffers: never draw into the
    // buffer that is still being sent.
    fpgaLink.sendRect(0, y0, W, BAND, buf);
  }
  fpgaLink.finish();   // wait for the last band
  fpgaLink.swap();     // show it at the next vertical blank
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP32 -> MAX1000 video card test ===");

  // DMA can only read internal RAM, hence heap_caps_malloc with MALLOC_CAP_DMA
  for (int i = 0; i < 2; i++) bandBuf[i] = (uint8_t*)heap_caps_malloc(W * BAND, MALLOC_CAP_DMA);
  if (!bandBuf[0] || !bandBuf[1] || !fpgaLink.begin(SPEEDS[0])) {
    Serial.println("ERROR: SPI/DMA init failed");
    while (true) delay(1000);
  }
  view.setColorDepth(8);

  // Step 1. The status word describes the END of the previous transaction,
  // so read it twice: the first read just makes sure the second is fresh.
  Serial.print("Waiting for the FPGA");
  fpga::Status s;
  for (int i = 0; i < 100; i++) {
    fpgaLink.status();
    s = fpgaLink.status();
    if (s.valid && (s.flags & fpga::F_SELFTEST_DONE)) break;
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  printStatus(s);
  if (!s.valid) Serial.println("No answer: check MISO (GPIO19 <- D3), CS, SCK, GND, and that the FPGA is programmed.");

  linkTest();
}

void loop()
{
  // serial keyboard commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 't') linkTest();
    else if (c == 'p') paused = !paused;
    else if (c >= '1' && c < '1' + NSPEEDS) {
      bool ok = fpgaLink.setSpeed(SPEEDS[c - '1']);
      Serial.printf("Speed %.1f MHz%s\n", fpgaLink.speed() / 1e6, ok ? "" : " (requested speed refused)");
    }
  }
  if (paused) { delay(50); return; }

  // wait until the previous frame is on screen (READY pin)
  static uint32_t statAt = millis(), frames = 0, busyUs = 0;
  if (!fpgaLink.waitReady(200)) {
    Serial.println("READY stuck low (check GPIO4 <- D4); reading status instead");
    printStatus(fpgaLink.status());
  }

  uint32_t t0 = micros();
  sendFrame();
  busyUs += micros() - t0;
  state.frame++;
  frames++;

  // every 2 s: update the numbers shown on screen and print them
  if (millis() - statAt >= 2000) {
    float secs = (millis() - statAt) / 1000.0f;
    fpgaLink.status();                          // (first read refreshes, see setup)
    fpga::Status st = fpgaLink.status();
    state.fps = frames / secs;
    state.mbps = frames * (W * H + 12 * 10) / secs / 1e6f;   // pixels + 10 band headers
    state.mhz = fpgaLink.speed() / 1e6f;
    state.flags = st.flags;
    state.errors = st.errors();
    Serial.printf("%.1f fps, %.1f ms per frame (draw+send), %.2f MB/s | ",
                  state.fps, busyUs / 1000.0f / frames, state.mbps);
    printStatus(st);
    frames = busyUs = 0;
    statAt = millis();
  }
}
