// FOCUS//SE dashboard - Arduino Uno R3, black & white composite video, fake data
//
// Video (TVout, pins are fixed on the Uno):
//   Pin 9 --[ 1k  ]--+
//                    +--> RCA center pin
//   Pin 7 --[ 470 ]--+
//   GND ----------------> RCA shell
//
// Button: pin 2 -> pushbutton -> GND (no resistor, internal pull-up)
//   short press = next screen, hold 1 s = auto-cycle on/off
//
// Screens and fake data live in dash_uno.h.

#include <TVout.h>
#include <fontALL.h>

TVout tv;

#include "dash_uno.h"

// ---- settings to tweak -------------------------------------------------
#define VIDEO_MODE     NTSC   // PAL if the picture rolls
#define BUTTON_PIN     2
#define AUTO_CYCLE_MS  6000
// ------------------------------------------------------------------------

uint8_t screen = 0;
bool autoCycle = false;
uint32_t lastCycle = 0;

void nextScreen(uint32_t now)
{
  screen = (screen + 1) % ud::SCREEN_COUNT;
  lastCycle = now;
  ud::enterScreen(screen);
}

// Debounced button: short press = next screen, long press = auto-cycle
void handleButton(uint32_t now)
{
  static bool lastRaw = false, stable = false, longFired = false;
  static uint32_t changedAt = 0, pressedAt = 0;

  bool raw = digitalRead(BUTTON_PIN) == LOW;
  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = now;
  }
  if (now - changedAt > 30 && raw != stable) {
    stable = raw;
    if (stable) {
      pressedAt = now;
      longFired = false;
    } else if (!longFired) {
      nextScreen(now);
    }
  }
  if (stable && !longFired && now - pressedAt > 1000) {
    longFired = true;
    autoCycle = !autoCycle;
    lastCycle = now;
  }
}

void setup()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  tv.begin(VIDEO_MODE, ud::W, ud::H);
  tv.clear_screen();

  // boot splash, one line at a time
  for (uint8_t i = 0; i < 6; i++) {
    ud::bootFrame(i);
    tv.delay(220);
  }
  tv.delay(400);
  ud::enterScreen(screen);
}

void loop()
{
  static uint32_t last = tv.millis();
  uint32_t now = tv.millis();
  float dt = (now - last) / 1000.0f;
  last = now;

  ud::sim.update(dt);
  handleButton(now);
  if (autoCycle && now - lastCycle > AUTO_CYCLE_MS) nextScreen(now);

  ud::drawScreen(screen, now);
  tv.delay_frame(2);   // ~30 updates per second, leaves the CPU room for video
}
