// FOCUS//SE dashboard - live engine data from the car's OBD-II port, shown on
// the head unit's composite (RCA) video input. One ESP32 does everything:
//
//   car --OBD-II--> Veepeak ELM327 adapter --Bluetooth--> ESP32 --GPIO25--> TV
//                                                           |
//                                          shift-light LED sticks (GPIO14)
//
// Board: ESP32-WROOM-DA ("ESP32-WROOM-DA Module" in Tools > Board).
//
// Wiring (all switches: pin -> switch -> GND, no resistors needed):
//   GPIO25  video -> RCA centre (RCA shell -> GND)
//   GPIO14  -> 270 ohm -> LED stick DIN (sticks chained DOUT -> DIN), stick 5V -> VIN, GND -> GND
//   GPIO27  screen switch (latching): every flip, either way = next screen
//   GPIO16  RPM-lights switch: on = LED sticks work, off = dark
//   GPIO17  sport switch: on = sport RPM thresholds + SPORT badge
//   BOOT button: short press = next screen, hold 1 s = auto-cycle the screens
// Serial monitor (115200): 1-4 = jump to a screen, n = next.
// Screens: 1 DUAL (two dials), 2 ARC (round tach), 3 DIAG (every value), 4 SYS (link + ESP32).
// In the car: key at ON; the adapter must not be connected to a phone or a Mac.
//
// Files:
//   dash.h        screen size, colours, RPM thresholds, data types, screens 2-4, simulator
//   dash_fx.h     the DUAL screen (two dials) and sport mode (fx::setSport)
//   render.h      draws each frame in bands into a small buffer (flicker-free)
//   obd.h         background task: Bluetooth -> ELM327 -> car, values for the screens
//   tiny_spp.h    our own minimal Bluetooth serial client (saves ~80 KB RAM)
//   shift_light.h the LED sticks: RPM bar + shift light

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "dash.h"
#include "dash_fx.h"
#include "render.h"
#include "obd.h"
#include "shift_light.h"

// ---- settings to tweak -------------------------------------------------
#define VIDEO_SIGNAL    NTSC   // NTSC, NTSC_J, PAL, PAL_M, PAL_N
#define VIDEO_PIN       25     // must be 25 or 26 (the ESP32's DAC pins)
#define OUTPUT_LEVEL    128    // raise (e.g. 180-220) if the picture is dim
#define DISPLAY_ASPECT  (4.0f / 3.0f)  // 5/3 or 16/9 if the head unit stretches the picture wide
#define VIDEO_W         360    // picture: 360 x 240 pixels (86 KB of RAM)
#define BAND_ROWS       120    // rows per drawing band: 2 bands per frame
#define SCREEN_SW_PIN   27
#define LIGHTS_SW_PIN   16
#define SPORT_SW_PIN    17
#define BOOT_BUTTON_PIN 0
#define AUTO_CYCLE_MS   6000   // auto-cycle: time on each screen
#define FRAME_MS        33     // ~30 fps target
#define USE_FAKE_DATA   false  // true = simulated driving, for testing without the car
#define SPORT_LED_PIN   32
#define RPM_LED_PIN     33
// ------------------------------------------------------------------------

// The composite video output (LovyanGFX's CVBS driver on the ESP32's DAC).
class TV : public lgfx::LGFX_Device
{
  lgfx::Panel_CVBS _panel;

public:
  TV() { setPanel(&_panel); }

  bool start(int w, int h)
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
    det.use_psram    = 0;          // picture in internal RAM (this board has no PSRAM)
    det.output_level = OUTPUT_LEVEL;
    det.chroma_level = 128;
    _panel.config_detail(det);

    setColorDepth(8);   // 1 byte per pixel (RGB332)
    return init();
  }
};

TV tv;
StripRenderer renderer;

#if USE_FAKE_DATA
dash::Simulator sim;        // fake driving
dash::Telemetry& data = sim;
#else
dash::Telemetry live;       // real data, filled by obd::copyInto()
dash::Telemetry& data = live;
#endif
fx::DualState dual;         // the DUAL screen's needle positions (they glide)

int screen = 0;             // 0..3
bool autoCycle = false;
uint32_t lastCycle = 0;

// Draw one frame with `draw` and show it.
template <typename DrawFn>
void showFrame(DrawFn draw)
{
  renderer.render(draw);
}

void nextScreen()
{
  screen = (screen + 1) % dash::SCREEN_COUNT;
  lastCycle = millis();
  Serial.printf("Screen %d: %s\n", screen + 1, dash::SCREEN_NAMES[screen]);
}

// Ignition-style gauge sweep: both dials and the shift lights ramp up to
// full scale, hold, and drop back to zero, like a real instrument cluster
// at key-on. ~2 s, with smoothstep easing so it starts and stops softly.
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
    shift::update(data.d.rpm, millis());
    uint32_t now = millis();
    showFrame([&](lgfx::LovyanGFX& g) { fx::drawDual(g, data, now, dual); });
    delay(FRAME_MS);
  }
  data.d.rpm = 0;
  data.d.mph = 0;
  dual.rpmShown = dual.mphShown = 0;
  shift::update(0, millis());
}// ---- switches -------------------------------------------------------------------
// A switch input, debounced: `on` only changes after the pin has held its new
// level for 30 ms (contacts bounce for a few ms when flipped). Switches go
// from the pin to GND, so ON reads LOW (internal pull-up resistor).

struct statusLED {
  int pin;
  bool on = false;
  void begin() { 
    pinMode(pin, OUTPUT);
  }
  void update(bool value) { 
    on = value; 
    digitalWrite(pin, on ? HIGH : LOW); 
  }
  void turnOn() { 
    update(true); 
  }
  void turnOff() { 
    update(false); 
  }
  void toggle(){
    update(!on);
  }
};

statusLED sportlight{SPORT_LED_PIN};
statusLED RPMLight{RPM_LED_PIN};



struct Switch {
  int pin;
  bool on = false, raw = false;
  uint32_t changedAt = 0;
  void begin() { pinMode(pin, INPUT_PULLUP); on = raw = digitalRead(pin) == LOW; }
  bool update(uint32_t now)   // true when `on` just changed
  {
    bool r = digitalRead(pin) == LOW;
    if (r != raw) { raw = r; changedAt = now; }
    if (raw != on && now - changedAt > 30) { on = raw; return true; }
    return false;
  }
};
Switch screenSw{ SCREEN_SW_PIN }, lightsSw{ LIGHTS_SW_PIN }, sportSw{ SPORT_SW_PIN };

void handleSwitches(uint32_t now)
{
  // latching screen switch: compared with its last state, so either direction = next screen
  if (screenSw.update(now)) nextScreen();
  if (lightsSw.update(now)) {
    RPMLight.toggle();
    shift::enabled = lightsSw.on;
    Serial.printf("RPM lights %s\n", lightsSw.on ? "ON" : "OFF");
  }
  if (sportSw.update(now)) {
    sportlight.toggle();
    fx::setSport(sportSw.on);
    Serial.printf("Sport mode %s (shift at %d rpm)\n", sportSw.on ? "ON" : "OFF", (int)dash::SHIFT_RPM);
  }
}

// BOOT button (momentary): short press = next screen, hold 1 s = auto-cycle
void handleBootButton(uint32_t now)
{
  static bool lastRaw = false, stable = false, longFired = false;
  static uint32_t changedAt = 0, pressedAt = 0;

  bool raw = digitalRead(BOOT_BUTTON_PIN) == LOW;
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

// ---- setup: runs once at power-up ---------------------------------------------------
void setup()
{
  Serial.begin(115200);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  screenSw.begin();               // remember where each switch starts (no screen change at boot)
  lightsSw.begin();
  sportSw.begin();
  fx::setSport(sportSw.on);
  delay(200);
  Serial.println("\n=== FOCUS//SE DASH ===");
  logMem("boot");
  sportlight.begin();
  RPMLight.begin();
  sportlight.update(sportSw.on);
  RPMLight.update(lightsSw.on);
  
  // the band buffer first (43 KB), then the video picture (86 KB)
  dash::setScreen(VIDEO_W, DISPLAY_ASPECT);
  if (!renderer.begin(&tv, VIDEO_W, dash::H, BAND_ROWS)) Serial.println("[MEM] ERROR: no RAM for the band buffer");
  if (!tv.start(VIDEO_W, dash::H)) Serial.println("[VID] ERROR: video failed to start");
  Serial.printf("[VID] %dx%d on GPIO%d, band buffer %u bytes\n", VIDEO_W, dash::H, VIDEO_PIN,
                (unsigned)renderer.bufferBytes());
  logMem("after video");

  shift::begin();
  shift::enabled = lightsSw.on;

  // boot splash, then the gauge sweep
  char mode[40];
  snprintf(mode, sizeof(mode), "VIDEO .... %dx%d", VIDEO_W, dash::H);
  uint32_t start = millis();
  while (millis() - start < 1500) {
    uint32_t t = millis() - start;
    showFrame([&](lgfx::LovyanGFX& g) { dash::drawBoot(g, t, mode); });
    delay(FRAME_MS);
  }
  startupSweep();

#if !USE_FAKE_DATA
  if (!obd::beginBluetooth()) Serial.println("[BT ] ERROR: Bluetooth could not start");
  logMem("after Bluetooth");
  obd::start();   // connect + read the car on a background task
  logMem("after OBD task");
#endif

  Serial.printf("Dashboard ready (%s data). Serial: 1-4 = screen, n = next\n", USE_FAKE_DATA ? "FAKE" : "LIVE");
}

// ---- loop: runs forever, one frame each time -------------------------------------------
void loop()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  float dt = (now - last) / 1000.0f;   // seconds since the last frame
  last = now;

  // 1. update the data and inputs (once per frame)
#if USE_FAKE_DATA
  sim.update(dt);
#else
  obd::copyInto(live, dt);
#endif
  fx::updateDual(dual, data, dt);
  handleBootButton(now);
  handleSwitches(now);
  handleSerial();
  if (autoCycle && now - lastCycle > AUTO_CYCLE_MS) nextScreen();
  shift::update(data.d.rpm, now);

  // 2. draw the frame band by band (drawing only reads the data)
  int s = screen;
  showFrame([&](lgfx::LovyanGFX& g) {
    if (s == 0) fx::drawDual(g, data, now, dual);
    else dash::draw(g, s, data, now);
  });

  // the band buffer has guard bytes around it: if drawing ever escapes the
  // band (a LovyanGFX call that ignores the clip rect), say which screen did it
  long badAt;
  if (size_t bad = renderer.guardDamage(&badAt)) {
    Serial.printf("[RENDER] ERROR: screen %d (%s) wrote %u bytes outside the band buffer (offset %ld)\n",
                  s + 1, dash::SCREEN_NAMES[s], (unsigned)bad, badAt);
    renderer.resetGuards();
  }

  // 3. stats: for the SYS screen once a second, and the Serial log every 5 s
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
    Serial.printf("[FPS] screen %d: avg %u ms, max %u ms per frame | free %u, lowest %u, block %u\n",
                  screen + 1, (unsigned)(statSum / statFrames), (unsigned)statMax, ESP.getFreeHeap(),
                  ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
    statFrames = statSum = statMax = 0;
    statAt = millis();
  }

  if (spent < FRAME_MS) delay(FRAME_MS - spent);   // ~30 frames per second
}
