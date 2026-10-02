// Live desktop preview of the ESP32 dashboard (fake driving data).
// Same drawing code AND the same strip renderer as the ESP32, in an SDL
// window scaled to the TV's 4:3 shape.
//
//   SPACE = the GPIO27 button (next screen)
//   close window = quit
//
// Build and run: ./run.sh        (240x240, like the WROOM board)
//                ./run.sh 360    (360x240, like a PSRAM board)

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <stdlib.h>
#include "dash.h"
#include "dash_fx.h"
#include "render.h"

static LGFX* lcd = nullptr;
static StripRenderer renderer;
static dash::Simulator sim;
static fx::DualState dual;
static int screen = 0;

void setup()
{
  const char* env = getenv("DASH_W");
  int w = env && atoi(env) == 360 ? 360 : 240;
  dash::setScreen(w, 4.0f / 3.0f);
  // window = 720x480: 240 px wide pixels are stretched 3x, 360 px ones 2x
  lcd = new LGFX(w, dash::H, w == 240 ? 3 : 2, 2);
  lgfx::Panel_sdl::addKeyCodeMapping(SDLK_SPACE, 27);
  lcd->init();
  lcd->setColorDepth(8);
  renderer.begin(lcd, w, dash::H);
  printf("%dx%d preview. SPACE = next screen (like the GPIO27 button)\n", w, dash::H);
}

void loop()
{
  static uint32_t last = lgfx::millis();
  static bool wasDown = false;
  uint32_t now = lgfx::millis();
  float dt = (now - last) / 1000.0f;
  last = now;

  sim.update(dt);
  fx::updateDual(dual, sim, dt);

  bool down = !lgfx::gpio_in(27);              // active low, like the real button
  if (down && !wasDown) {
    screen = (screen + 1) % dash::SCREEN_COUNT;
    printf("screen %d: %s\n", screen + 1, dash::SCREEN_NAMES[screen]);
  }
  wasDown = down;

  int s = screen;
  renderer.render([&](lgfx::LovyanGFX& g) {
    if (s == 0) fx::drawDual(g, sim, now, dual);
    else dash::draw(g, s, sim, now);
  });

  uint32_t spent = lgfx::millis() - now;
  if (spent < 33) lgfx::delay(33 - spent);
}
