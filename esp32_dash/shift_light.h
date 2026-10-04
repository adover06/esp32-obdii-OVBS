// Shift lights: LM3914 10-LED bar + one RGB LED, driven from the live RPM.
//
// Wiring (middle breadboard, see the wiring diagram):
//   GPIO26 (DAC) -> LM3914 pin 5 (SIG), a plain wire
//   GPIO32 / GPIO33 / GPIO13 -> RGB red / green / blue, 270 ohm on the long leg
//
// Bar: empty at BAR_START_RPM, full at dash::SHIFT_RPM (the same shift point
// the screens flash at). The LM3914 lights LED k above k x 0.125 V, so the
// DAC aims for the middle of each step and small errors never flicker an LED.
//
// RGB: off, then solid blue from WARN_PCT of the shift point, then strobing
// red / yellow at and above it. The LED has one shared resistor, so only one
// colour may be on at a time; yellow is made by switching red/green every
// millisecond (a timer does that, independent of the drawing loop).
#pragma once
#include "esp_timer.h"

namespace shift {

constexpr int BAR_PIN = 26;          // DAC2
constexpr int R_PIN = 32, G_PIN = 33, B_PIN = 13;
constexpr bool COMMON_ANODE = false; // true if the RGB long leg goes to 3V3
constexpr float BAR_START_RPM = 900;    // just above idle (780), so the bar is dark at idle
constexpr int WARN_PCT = 90;

enum Colour { OFF, RED, GREEN, BLUE, YELLOW };
static volatile Colour colour = OFF;

static void writeRgb(bool r, bool g, bool b)
{
  digitalWrite(R_PIN, r != COMMON_ANODE);
  digitalWrite(G_PIN, g != COMMON_ANODE);
  digitalWrite(B_PIN, b != COMMON_ANODE);
}

// every 1 ms (esp_timer task)
static void tick(void*)
{
  static bool phase = false;
  phase = !phase;
  switch (colour) {
    case RED:    writeRgb(true, false, false); break;
    case GREEN:  writeRgb(false, true, false); break;
    case BLUE:   writeRgb(false, false, true); break;
    case YELLOW: writeRgb(phase, !phase, false); break;
    default:     writeRgb(false, false, false); break;
  }
}

// Light n LEDs (0..10).
static void setBar(int n)
{
  n = constrain(n, 0, 10);
  float volts = (n == 0) ? 0.0f : (n + 0.5f) * 0.125f;
  dacWrite(BAR_PIN, constrain((int)(volts / 3.3f * 255 + 0.5f), 0, 255));
}

static int ledsFor(float rpm)
{
  if (rpm <= BAR_START_RPM) return 0;
  float frac = (rpm - BAR_START_RPM) / (dash::SHIFT_RPM - BAR_START_RPM);
  return constrain((int)ceilf(frac * 10.0f), 0, 10);
}

// Call once at boot: pins and the RGB timer.
static void begin()
{
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  writeRgb(false, false, false);
  esp_timer_create_args_t args = {};
  args.callback = tick;
  args.name = "shiftrgb";
  esp_timer_handle_t timer;
  if (esp_timer_create(&args, &timer) == ESP_OK) esp_timer_start_periodic(timer, 1000);

  setBar(0);   // the dashboard's startup sweep doubles as the light check
}

// Call every frame with the current RPM (0 or idle when there's no data).
static void update(float rpm, uint32_t ms)
{
  if (!isfinite(rpm)) rpm = 0;
  setBar(ledsFor(rpm));
  if (rpm >= dash::SHIFT_RPM) colour = ((ms / 70) % 2) ? RED : YELLOW;   // strobe ~7 Hz
  else if (rpm >= dash::SHIFT_RPM * WARN_PCT / 100) colour = BLUE;
  else colour = OFF;
}

}  // namespace shift
