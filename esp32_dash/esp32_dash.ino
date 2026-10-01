// FOCUS//SE dashboard - composite video, live OBD data over Bluetooth
//
// Board:  original ESP32 (WROOM-32 / WROOM-DA). Select "ESP32-WROOM-DA Module".
// Video:  GPIO25 -> RCA center pin, GND -> RCA shell
// Button: GPIO27 -> pushbutton -> GND (no resistor needed, internal pull-up)
//         The on-board BOOT button (GPIO0) works too.
//         Short press = next screen, hold 1 s = toggle auto-cycle.
// Serial: send 1-7 to jump to a screen, n = next (115200 baud)
// Car:    key at ON; adapter "OBDII" not connected to a phone or Mac.
//
// Screens 2-7 and the fake-data simulator live in dash.h, the dual-dial
// screen (1) in dash_fx.h, and the Bluetooth/OBD link in obd.h.

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "dash.h"
#include "dash_fx.h"
#include "obd.h"

// ---- settings to tweak -------------------------------------------------
#define VIDEO_SIGNAL   NTSC   // NTSC, NTSC_J, PAL, PAL_M, PAL_N
#define VIDEO_PIN      25
#define OUTPUT_LEVEL   128    // raise (e.g. 180-220) if the picture is dim
#define BUTTON_PIN     27
#define BOOT_BUTTON_PIN       0
#define AUTO_CYCLE_MS  6000
#define FRAME_MS       33     // ~30 fps
#define USE_FAKE_DATA  false  // true = simulator, for testing without the car

// MEMORY TEST: smaller video picture + no 86 KB drawing buffer, to leave
// Bluetooth room to connect. Screens are still laid out for 360x240, so the
// right/bottom edges get cut off and there's some flicker until they're redone.
#define VIDEO_W        320    // was 360  (picture RAM = VIDEO_W * VIDEO_H bytes)
#define VIDEO_H        200    // was 240
#define USE_CANVAS     false  // true = flicker-free 86 KB buffer (needs more RAM)
// ------------------------------------------------------------------------

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_CVBS _panel;

public:
  LGFX(void)
  {
    {
      auto cfg = _panel.config();
      cfg.memory_width  = VIDEO_W;
      cfg.memory_height = VIDEO_H;
      cfg.panel_width   = VIDEO_W;
      cfg.panel_height  = VIDEO_H;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel.config(cfg);
    }
    {
      auto cfg = _panel.config_detail();
      cfg.signal_type  = cfg.signal_type_t::VIDEO_SIGNAL;
      cfg.pin_dac      = VIDEO_PIN;
      cfg.use_psram    = 0;
      cfg.output_level = OUTPUT_LEVEL;
      cfg.chroma_level = 128;
      _panel.config_detail(cfg);
    }
    setPanel(&_panel);
  }
};

LGFX tv;
LGFX_Sprite canvas(&tv);   // off-screen frame, pushed whole to avoid flicker
#if USE_FAKE_DATA
dash::Simulator sim;        // fake data
dash::Telemetry& data = sim;
#else
dash::Telemetry live;       // real data, filled by obd::copyInto()
dash::Telemetry& data = live;
#endif
fx::DualState dual;

// If Bluetooth can't fit next to the 86 KB canvas, the canvas is freed and
// screens draw straight onto the TV instead (may flicker a bit more).
bool drawDirect = false;

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

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  Serial.printf("Heap before video: free %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // The canvas needs one 86 KB block, so grab it before the video driver
  // (which allocates in small chunks) breaks up the heap.
  bool canvasOk = false;
  if (USE_CANVAS) {
    canvas.setColorDepth(8);   // RGB332, same format as the video output
    canvasOk = canvas.createSprite(dash::W, dash::H);
  }
  drawDirect = !canvasOk;    // no canvas = draw straight onto the TV

  tv.setColorDepth(8);
  tv.init();
  Serial.printf("[VID] %dx%d picture, %s\n", VIDEO_W, VIDEO_H, drawDirect ? "drawing direct (no canvas)" : "with canvas");
  Serial.printf("[MEM] after canvas + video: free %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // boot splash
  uint32_t start = millis();
  while (millis() - start < 1800) {
    if (drawDirect) dash::drawBoot(tv, millis() - start);
    else { dash::drawBoot(canvas, millis() - start); canvas.pushSprite(0, 0); }
    delay(FRAME_MS);
  }

#if !USE_FAKE_DATA
  if (!obd::beginBluetooth() && !drawDirect) {
    Serial.println("[MEM] not enough memory: freeing the canvas and retrying Bluetooth");
    canvas.deleteSprite();
    drawDirect = true;
    obd::beginBluetooth();
  }
  obd::start();   // connect + read the car on a background task
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

#if USE_FAKE_DATA
  sim.update(dt);
#else
  obd::copyInto(live, dt);
#endif
  handleButton(now);
  handleSerial();
  if (autoCycle && now - lastCycle > AUTO_CYCLE_MS) nextScreen();

  lgfx::LovyanGFX& target = drawDirect ? (lgfx::LovyanGFX&)tv : canvas;
  if (screen == 0) fx::drawDual(target, data, now, dual, dt);
  else dash::draw(target, screen, data, now);
  if (!drawDirect) canvas.pushSprite(0, 0);

  uint32_t spent = millis() - now;
  if (spent < FRAME_MS) delay(FRAME_MS - spent);
}
