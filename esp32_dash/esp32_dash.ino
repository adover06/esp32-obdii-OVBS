// FOCUS//SE dashboard - composite video, live OBD data over Bluetooth
//
// Board:  original ESP32 (WROOM-32 / WROOM-DA). Select "ESP32-WROOM-DA Module".
//         With a PSRAM board (ESP32-WROVER, Tools > PSRAM: Enabled) it switches
//         to 360x240 automatically.
// Video:  GPIO25 -> RCA center pin, GND -> RCA shell
// Button: GPIO27 -> pushbutton -> GND (no resistor needed, internal pull-up)
//         The on-board BOOT button (GPIO0) works too.
//         Short press = next screen, hold 1 s = toggle auto-cycle.
// Serial: send 1-7 to jump to a screen, n = next (115200 baud)
// Car:    key at ON; adapter "OBDII" not connected to a phone or Mac.
//
// Files: dash.h (screens 2-7 + simulator), dash_fx.h (dual dials),
//        render.h (flicker-free strip renderer), obd.h (Bluetooth/OBD link).
//
// Memory plan (no PSRAM): video picture 240x240 = 56 KB, strip buffer 6 KB,
// Bluetooth ~86 KB, leaving ~40 KB free so Bluetooth can connect reliably.

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "dash.h"
#include "dash_fx.h"
#include "render.h"
#include "obd.h"

// ---- settings to tweak -------------------------------------------------
#define VIDEO_SIGNAL    NTSC   // NTSC, NTSC_J, PAL, PAL_M, PAL_N
#define VIDEO_PIN       25
#define OUTPUT_LEVEL    128    // raise (e.g. 180-220) if the picture is dim
#define DISPLAY_ASPECT  (4.0f / 3.0f)  // 5/3 or 16/9 if the head unit stretches the picture wide
#define BUTTON_PIN      27
#define BOOT_BUTTON_PIN 0
#define AUTO_CYCLE_MS   6000
#define FRAME_MS        33     // ~30 fps target
#define USE_FAKE_DATA   false  // true = simulator, for testing without the car
// ------------------------------------------------------------------------

// Composite video output. The picture size is chosen at boot, so the panel is
// configured in start() rather than the constructor.
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

TV tv;
StripRenderer renderer;

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

  // Picture size: 360x240 needs 86 KB, which only fits next to Bluetooth when
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

  // boot splash
  char mode[40];
  snprintf(mode, sizeof(mode), "VIDEO .... %dx%d", w, dash::H);
  uint32_t start = millis();
  while (millis() - start < 1500) {
    uint32_t t = millis() - start;
    renderer.render([&](lgfx::LovyanGFX& g) { dash::drawBoot(g, t, mode); });
    delay(FRAME_MS);
  }

#if !USE_FAKE_DATA
  if (!obd::beginBluetooth()) Serial.println("[BT ] ERROR: Bluetooth could not start");
  logMem("after Bluetooth");
  obd::start();   // connect + read the car on a background task
  logMem("after OBD task");
#endif

  Serial.printf("Dashboard ready (%s data). Button on GPIO27 (or BOOT). Serial: 1-7 = screen, n = next\n",
                USE_FAKE_DATA ? "FAKE" : "LIVE");
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

  // 2. draw the frame band by band (drawing only reads state)
  int s = screen;
  renderer.render([&](lgfx::LovyanGFX& g) {
    if (s == 0) fx::drawDual(g, data, now, dual);
    else dash::draw(g, s, data, now);
  });

  // the strip buffer has guard bytes around it: if drawing ever escapes the
  // band (a LovyanGFX call that ignores the clip rect), say which screen did it
  long badAt;
  if (size_t bad = renderer.guardDamage(&badAt)) {
    Serial.printf("[RENDER] ERROR: screen %d (%s) wrote %u bytes outside the strip buffer (offset %ld)\n",
                  s + 1, dash::SCREEN_NAMES[s], (unsigned)bad, badAt);
    renderer.resetGuards();
  }

  // 3. stats every 5 s: frame time, memory, OBD task stack headroom
  uint32_t spent = millis() - now;
  static uint32_t statFrames = 0, statSum = 0, statMax = 0, statAt = millis();
  statFrames++;
  statSum += spent;
  if (spent > statMax) statMax = spent;
  if (millis() - statAt >= 5000) {
    Serial.printf("[FPS] screen %d: avg %u ms, max %u ms per frame | free %u, block %u",
                  screen + 1, (unsigned)(statSum / statFrames), (unsigned)statMax, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
#if !USE_FAKE_DATA
    Serial.printf(" | OBD stack free %u", (unsigned)obd::stackFreeBytes());
#endif
    Serial.println();
    statFrames = statSum = statMax = 0;
    statAt = millis();
  }

  if (spent < FRAME_MS) delay(FRAME_MS - spent);
}
