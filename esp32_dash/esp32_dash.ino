// FOCUS//SE dashboard - live OBD data over Bluetooth, shown on the car's
// composite (RCA) video input.
//
// Two ways to make the video (OUTPUT_FPGA below):
//   true  (normal): the MAX1000 FPGA video card. The ESP32 draws 720x240
//         frames in 24-row bands and streams them over SPI; the FPGA stores
//         them (double-buffered) and generates the NTSC signal. No video
//         framebuffer on the ESP32, no tearing, no flicker.
//   false (fallback): the ESP32 makes the video itself on GPIO25, 240x240,
//         like before the FPGA. Use it if the FPGA board is unplugged/broken.
//
// Board:  ESP32-WROOM-DA ("ESP32-WROOM-DA Module").
// FPGA:   SCK GPIO18 -> D0, MOSI GPIO23 -> D1, CS GPIO5 -> D2,
//         MISO GPIO19 <- D3, READY GPIO4 <- D4, GND <-> GND   (see fpga_link.h)
// Shift lights (FPGA mode): LM3914 bar on GPIO26, RGB on GPIO32/33/13 (shift_light.h)
// Button: GPIO27 -> pushbutton -> GND (or the BOOT button).
//         Short press = next screen, hold 1 s = toggle auto-cycle.
// Serial: 1-4 = jump to a screen, n = next (115200 baud)
// Screens: 1 DUAL (dual dials), 2 ARC (round tach), 3 DIAG (every value), 4 SYS (link + ESP32)
// Car:    key at ON; adapter "OBDII" not connected to a phone or Mac.
//
// Files: dash.h (screens 2-4 + simulator), dash_fx.h (dual dials),
//        render.h (strip renderer), obd.h (Bluetooth/OBD link),
//        fpga_link.h (SPI link to the FPGA), shift_light.h (LED bar + RGB).

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "dash.h"
#include "dash_fx.h"
#include "render.h"
#include "obd.h"
#include "fpga_link.h"
#include "shift_light.h"

// ---- settings to tweak -------------------------------------------------
#define OUTPUT_FPGA     true   // false = old direct video on GPIO25 (fallback)
#define SHIFT_LIGHTS    true   // LM3914 bar + RGB (FPGA mode only: GPIO26 is the old video's DAC twin)
#define VIDEO_SIGNAL    NTSC   // direct-video mode only: NTSC, NTSC_J, PAL, PAL_M, PAL_N
#define VIDEO_PIN       25
#define OUTPUT_LEVEL    128    // direct-video mode only: raise (e.g. 180-220) if the picture is dim
#define DISPLAY_ASPECT  (4.0f / 3.0f)  // 5/3 or 16/9 if the head unit stretches the picture wide
#define BUTTON_PIN      27
#define BOOT_BUTTON_PIN 0
#define AUTO_CYCLE_MS   6000
#define FRAME_MS        33     // ~30 fps target
#define USE_FAKE_DATA   false  // true = simulator, for testing without the car
// ------------------------------------------------------------------------

// Direct composite video output (fallback mode). The picture size is chosen
// at boot, so the panel is configured in start() rather than the constructor.
class TV : public lgfx::LGFX_Device
{
  lgfx::Panel_CVBS _panel;

public:
  TV() { setPanel(&_panel); }

  // psram: 0 = picture in internal RAM, 2 = picture in PSRAM
  bool start(int w, int h, uint8_t psram)
  {
    auto cfg = _panel.config();
    cfg.memory_width  = w;
    cfg.memory_height = h;
    cfg.panel_width   = w;
    cfg.panel_height  = h;
    cfg.offset_x = 0;
    cfg.offset_y = 0;
    _panel.config(cfg);

    auto det = _panel.config_detail();
    det.signal_type  = det.signal_type_t::VIDEO_SIGNAL;
    det.pin_dac      = VIDEO_PIN;
    det.use_psram    = psram;
    // with the picture in PSRAM the driver runs a high-priority copy task;
    // keep it on core 1 so it can't starve Bluetooth on core 0
    if (psram) det.task_pinned_core = 1;
    det.output_level = OUTPUT_LEVEL;
    det.chroma_level = 128;
    _panel.config_detail(det);

    setColorDepth(8);   // RGB332
    return init();
  }
};

TV tv;                     // only started in direct-video mode
StripRenderer renderer;
fpga::Link fpgaLink;
bool fpgaAnswered = false;
uint32_t fpgaHz = 0;
fpga::Status fpgaStatus;
uint32_t readyTimeouts = 0;

#if USE_FAKE_DATA
dash::Simulator sim;        // fake data
dash::Telemetry& data = sim;
#else
dash::Telemetry live;       // real data, filled by obd::copyInto()
dash::Telemetry& data = live;
#endif
fx::DualState dual;

int screen = 0;
bool autoCycle = false;
uint32_t lastCycle = 0;

// Draw one frame with `draw` and show it, on whichever output is in use.
template <typename DrawFn>
void showFrame(DrawFn draw)
{
#if OUTPUT_FPGA
  // the previous frame must be on screen before we overwrite the back buffer;
  // READY goes high at the vertical blank that shows it (at most ~17 ms)
  if (!fpgaLink.waitReady(40)) readyTimeouts++;
  renderer.renderTo(draw, [](uint8_t* buf, int y0, int rows) {
    fpgaLink.sendRect(0, y0, dash::W, rows, buf);   // DMA; returns while it sends
  });
  fpgaLink.finish();
  fpgaLink.swap();
#else
  renderer.render(draw);
#endif
}

// Ignition-style gauge sweep: both dials and the shift lights ramp up to
// full scale, hold, and drop back to zero, like a real instrument cluster
// at key-on. ~2 s. Uses smoothstep easing so it starts and stops softly.
void startupSweep()
{
  const uint32_t UP = 900, HOLD = 250, DOWN = 900;
  uint32_t t0 = millis();
  for (;;) {
    uint32_t t = millis() - t0;
    if (t > UP + HOLD + DOWN) break;
    float x = t < UP ? t / (float)UP : t < UP + HOLD ? 1.0f : 1.0f - (t - UP - HOLD) / (float)DOWN;
    float e = x * x * (3 - 2 * x);                    // smoothstep: 0..1, soft at both ends
    data.d.rpm = e * dash::RPM_MAX;
    data.d.mph = e * 120;
    dual.rpmShown = data.d.rpm;                       // needles follow exactly (no glide)
    dual.mphShown = data.d.mph;
#if OUTPUT_FPGA && SHIFT_LIGHTS
    shift::update(data.d.rpm, millis());
#endif
    uint32_t now = millis();
    showFrame([&](lgfx::LovyanGFX& g) { fx::drawDual(g, data, now, dual); });
#if !OUTPUT_FPGA
    delay(FRAME_MS);
#endif
  }
  data.d.rpm = 0;
  data.d.mph = 0;
  dual.rpmShown = dual.mphShown = 0;
#if OUTPUT_FPGA && SHIFT_LIGHTS
  shift::update(0, millis());
#endif
}

void nextScreen()
{
  screen = (screen + 1) % dash::SCREEN_COUNT;
  lastCycle = millis();
  Serial.printf("Screen %d: %s\n", screen + 1, dash::SCREEN_NAMES[screen]);
}

// Debounced button: short press = next screen, long press = auto-cycle
void handleButton(uint32_t now)
{
  static bool lastRaw = false, stable = false, longFired = false;
  static uint32_t changedAt = 0, pressedAt = 0;

  bool raw = digitalRead(BUTTON_PIN) == LOW || digitalRead(BOOT_BUTTON_PIN) == LOW;
  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = now;
  }
  if (now - changedAt > 25 && raw != stable) {
    stable = raw;
    if (stable) {
      pressedAt = now;
      longFired = false;
    } else if (!longFired) {
      nextScreen();
    }
  }
  if (stable && !longFired && now - pressedAt > 1000) {
    longFired = true;
    autoCycle = !autoCycle;
    lastCycle = now;
    Serial.printf("Auto-cycle %s\n", autoCycle ? "ON" : "OFF");
  }
}

void handleSerial()
{
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '0' + dash::SCREEN_COUNT) {
      screen = c - '1';
      Serial.printf("Screen %d: %s\n", screen + 1, dash::SCREEN_NAMES[screen]);
    } else if (c == 'n') {
      nextScreen();
    }
  }
}

void logMem(const char* when)
{
  Serial.printf("[MEM] %-16s free %6u  largest block %6u\n", when, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
}

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  delay(200);
  Serial.println("\n=== FOCUS//SE DASH ===");
  logMem("boot");

#if OUTPUT_FPGA
  // 720x240 through the FPGA (one pixel per 14.3 MHz video sample, the full
  // width NTSC carries). Two 17 KB band buffers (one draws while the other is
  // sent by DMA) replace the old 56 KB picture.
  int w = fpga::WIDTH;
  dash::setScreen(w, DISPLAY_ASPECT);
  if (!renderer.begin(nullptr, w, dash::H, 2)) Serial.println("[MEM] ERROR: no RAM for the band buffers");
  logMem("after band bufs");
  if (!fpgaLink.begin(8000000)) Serial.println("[FPGA] ERROR: SPI init failed");
  // is the FPGA there? (status describes the previous transaction: read twice)
  for (int i = 0; i < 20 && !fpgaAnswered; i++) {
    fpgaLink.status();
    fpgaStatus = fpgaLink.status();
    fpgaAnswered = fpgaStatus.valid && (fpgaStatus.flags & fpga::F_SELFTEST_DONE);
    if (!fpgaAnswered) delay(50);
  }
  if (fpgaAnswered) {
    fpgaHz = fpgaLink.autoSpeed(renderer.buffer(0), 4096);
    Serial.printf("[FPGA] OK, flags %02X, SPI %.1f MHz\n", fpgaStatus.flags, fpgaLink.speed() / 1e6);
  } else {
    Serial.println("[FPGA] no answer: check the 6 wires and that the FPGA has power. Sending anyway at 8 MHz.");
  }
#if SHIFT_LIGHTS
  shift::begin();
#endif
#else
  // Direct video: 360x240 needs 86 KB, which only fits next to Bluetooth when
  // the picture can live in PSRAM. Otherwise 240x240 (56 KB, even pixel mapping).
  bool psram = psramFound();
  int w = psram ? 360 : 240;
  dash::setScreen(w, DISPLAY_ASPECT);
  // small allocation first, then the video picture
  if (!renderer.begin(&tv, w, dash::H)) Serial.println("[MEM] ERROR: no RAM for the strip buffer");
  logMem("after strip buf");
  if (!tv.start(w, dash::H, psram ? 2 : 0)) Serial.println("[VID] ERROR: video failed to start");
  Serial.printf("[VID] %dx%d %s, strip buffer %u bytes\n", w, dash::H, psram ? "(picture in PSRAM)" : "(no PSRAM)",
                (unsigned)renderer.bufferBytes());
  logMem("after video");
#endif

  // boot splash
  char mode[40];
  snprintf(mode, sizeof(mode), "VIDEO .... %dx%d %s", w, dash::H, OUTPUT_FPGA ? "FPGA" : "");
  uint32_t start = millis();
  while (millis() - start < 1500) {
    uint32_t t = millis() - start;
    showFrame([&](lgfx::LovyanGFX& g) { dash::drawBoot(g, t, mode); });
    delay(OUTPUT_FPGA ? 0 : FRAME_MS);
  }
  startupSweep();

#if !USE_FAKE_DATA
  if (!obd::beginBluetooth()) Serial.println("[BT ] ERROR: Bluetooth could not start");
  logMem("after Bluetooth");
  obd::start();   // connect + read the car on a background task
  logMem("after OBD task");
#endif

  Serial.printf("Dashboard ready (%s data, %s). Button on GPIO27 (or BOOT). Serial: 1-4 = screen, n = next\n",
                USE_FAKE_DATA ? "FAKE" : "LIVE", OUTPUT_FPGA ? "FPGA video" : "direct video");
}

void loop()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  float dt = (now - last) / 1000.0f;
  last = now;

  // 1. update state (once per frame)
#if USE_FAKE_DATA
  sim.update(dt);
#else
  obd::copyInto(live, dt);
#endif
  fx::updateDual(dual, data, dt);
  handleButton(now);
  handleSerial();
  if (autoCycle && now - lastCycle > AUTO_CYCLE_MS) nextScreen();
#if OUTPUT_FPGA && SHIFT_LIGHTS
  shift::update(data.d.rpm, now);
#endif

  // 2. draw the frame band by band (drawing only reads state)
  int s = screen;
  showFrame([&](lgfx::LovyanGFX& g) {
    if (s == 0) fx::drawDual(g, data, now, dual);
    else dash::draw(g, s, data, now);
  });

  // the band buffers have guard bytes around them: if drawing ever escapes the
  // band (a LovyanGFX call that ignores the clip rect), say which screen did it
  long badAt;
  if (size_t bad = renderer.guardDamage(&badAt)) {
    Serial.printf("[RENDER] ERROR: screen %d (%s) wrote %u bytes outside the strip buffer (offset %ld)\n",
                  s + 1, dash::SCREEN_NAMES[s], (unsigned)bad, badAt);
    renderer.resetGuards();
  }

  // 3. stats: SYS screen once a second, Serial log every 5 s
  uint32_t spent = millis() - now;
  static uint32_t statFrames = 0, statSum = 0, statMax = 0, statAt = millis();
  static uint32_t sysFrames = 0, sysSum = 0, sysAt = millis();
  statFrames++;
  statSum += spent;
  if (spent > statMax) statMax = spent;
  sysFrames++;
  sysSum += spent;
  if (millis() - sysAt >= 1000) {
    data.frameMs = (float)sysSum / sysFrames;
    data.freeHeap = ESP.getFreeHeap();
    data.minFreeHeap = ESP.getMinFreeHeap();   // lowest it has ever been since boot
    data.maxBlock = ESP.getMaxAllocHeap();
    data.videoW = dash::W;
#if !USE_FAKE_DATA
    data.obdStackFree = obd::stackFreeBytes();
#endif
    sysFrames = sysSum = 0;
    sysAt = millis();
  }
  if (millis() - statAt >= 5000) {
    Serial.printf("[FPS] screen %d: avg %u ms, max %u ms per frame | free %u, lowest %u, block %u",
                  screen + 1, (unsigned)(statSum / statFrames), (unsigned)statMax, ESP.getFreeHeap(),
                  ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
#if !USE_FAKE_DATA
    Serial.printf(" | OBD stack free %u", (unsigned)obd::stackFreeBytes());
#endif
#if OUTPUT_FPGA
    fpgaLink.status();
    fpgaStatus = fpgaLink.status();
    if (!fpgaStatus.valid) Serial.print(" | FPGA no answer");
    else Serial.printf(" | FPGA flags %02X%s, SPI %.1f MHz, READY waits timed out %u",
                       fpgaStatus.flags, fpgaStatus.errors() ? " ERROR" : "", fpgaLink.speed() / 1e6,
                       (unsigned)readyTimeouts);
#endif
    Serial.println();
    statFrames = statSum = statMax = 0;
    statAt = millis();
  }

#if !OUTPUT_FPGA
  if (spent < FRAME_MS) delay(FRAME_MS - spent);   // FPGA mode is paced by READY (the TV's 60 Hz)
#endif
}
