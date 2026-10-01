// Live desktop preview of the ESP32 dashboard (fake driving data).
// Uses the same drawing code as the ESP32 sketch, shown in an SDL window.
//
//   SPACE = the GPIO27 button (next screen)
//   Q / close window = quit
//
// Build and run: ./run.sh

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include "dash.h"
#include "dash_fx.h"

static LGFX lcd(dash::W, dash::H, 3);          // 3x window scaling
static LGFX_Sprite canvas(&lcd);               // 8-bit RGB332, same as the ESP32
static dash::Simulator sim;
static fx::DualState dual;

static const int SCREENS = dash::SCREEN_COUNT;
static int screen = 0;

void setup()
{
  lgfx::Panel_sdl::addKeyCodeMapping(SDLK_SPACE, 27);
  lcd.init();
  canvas.setColorDepth(8);
  canvas.createSprite(dash::W, dash::H);
  printf("SPACE = next screen (like the GPIO27 button)\n");
}

void loop()
{
  static uint32_t last = lgfx::millis();
  static bool wasDown = false;
  uint32_t now = lgfx::millis();
  float dt = (now - last) / 1000.0f;
  last = now;

  sim.update(dt);

  bool down = !lgfx::gpio_in(27);              // active low, like the real button
  if (down && !wasDown) {
    screen = (screen + 1) % SCREENS;
    printf("screen %d\n", screen + 1);
  }
  wasDown = down;

  if (screen == 0) fx::drawDual(canvas, sim, now, dual, dt);
  else dash::draw(canvas, screen, sim, now);
  canvas.pushSprite(0, 0);

  uint32_t spent = lgfx::millis() - now;
  if (spent < 33) lgfx::delay(33 - spent);
}
