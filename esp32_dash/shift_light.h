// Shift lights, driven from the live RPM.
//
// Two kinds of hardware (SHIFT_HW below):
//
// SHIFT_HW_WS2812 (default): one or more addressable LED sticks (WS2812B /
//   SK6812, e.g. the 8-LED sticks). ONE data wire, sticks chained:
//     ESP32 GPIO14 -> 270 ohm -> stick 1 DIN,  stick 1 DOUT -> stick 2 DIN
//     5V -> VIN, GND -> GND on every stick
//   Layout: the first LEDs are an RPM bar (green, yellow, red by position),
//   the last SHIFT_LEDS are a separate shift light: blue near the shift point,
//   strobing red/yellow at and above it. MIRROR shows the same pattern on
//   every stick (e.g. one stick each side of the screen).
//   Driven by our own small RMT driver below (no library needed).
//
// SHIFT_HW_LM3914: the LM3914 10-LED bar (PWM on GPIO14 through a 10k/1uF
//   filter to LM3914 pin 5) plus an RGB LED on GPIO32/33/13.
//   (GPIO14 sends a few pulses during boot, so the bar may flicker once.)
//
// Bar: empty at BAR_START_RPM, full at dash::SHIFT_RPM (normal or sport).
// The RPM-array switch sets `enabled`; off = everything dark.
#pragma once
#include "esp_timer.h"
#include "esp32-hal-rmt.h"

namespace shift {

#define SHIFT_HW_WS2812 1
#define SHIFT_HW_LM3914 2
#ifndef SHIFT_HW
#define SHIFT_HW SHIFT_HW_WS2812
#endif

// ---- settings ----------------------------------------------------------------
constexpr int   DATA_PIN = 14;            // WS2812 data, or the LM3914's PWM
constexpr float BAR_START_RPM = 900;      // just above idle (780), so the bar is dark at idle
constexpr int   WARN_PCT = 90;            // shift light turns blue at this % of the shift point

// WS2812 sticks
constexpr int   LED_COUNT   = 16;         // all LEDs on the chain (2 sticks x 8)
constexpr int   STICK_LEDS  = 8;          // LEDs per stick (for MIRROR)
constexpr bool  MIRROR      = false;      // true: every stick shows the same 8-LED pattern
constexpr int   SHIFT_LEDS  = 2;          // last LEDs of the pattern = shift light
constexpr uint8_t BRIGHTNESS = 48;        // 0-255; keeps current low (each LED up to 60 mA at 255 white)
constexpr uint8_t UNLIT_DIM  = 6;         // unlit bar LEDs glow at this % of their colour (0 = off)
constexpr float GREEN_UNTIL  = 0.55f;     // bar position (0-1) where green ends
constexpr float YELLOW_UNTIL = 0.80f;     // ...where yellow ends; red after that
constexpr bool  RGBW         = false;     // true if the sticks are RGBW (4 bytes per LED)

// LM3914 + RGB LED
constexpr int R_PIN = 32, G_PIN = 33, B_PIN = 13;
constexpr bool COMMON_ANODE = false;      // RGB LED long leg to 3V3 instead of GND

static bool enabled = true;               // the RPM-array switch

static int ledsFor(float rpm, int n)      // RPM -> how many of n bar LEDs are lit
{
  if (rpm <= BAR_START_RPM) return 0;
  float frac = (rpm - BAR_START_RPM) / (dash::SHIFT_RPM - BAR_START_RPM);
  return constrain((int)ceilf(frac * n), 0, n);
}

#if SHIFT_HW == SHIFT_HW_WS2812
// ---- WS2812 / SK6812 driver on the RMT peripheral ----------------------------
// Each bit is one RMT symbol: a high pulse then a low pulse. At a 10 MHz RMT
// clock (0.1 us ticks): "0" = 0.4 us high + 0.9 us low, "1" = 0.8 us high +
// 0.4 us low - inside both WS2812B and SK6812 timing. Colors go out G, R, B.
// The latch (>80 us low) happens by itself: frames are tens of ms apart.
constexpr int BYTES_PER_LED = RGBW ? 4 : 3;
static rmt_data_t symbols[LED_COUNT * BYTES_PER_LED * 8];
static uint32_t frame[LED_COUNT];          // 0x00RRGGBB per LED
static uint32_t shown[LED_COUNT];          // last frame actually sent
static bool rmtOk = false;

static void show()
{
  if (!rmtOk || memcmp(frame, shown, sizeof(frame)) == 0) return;   // only send changes
  int k = 0;
  for (int i = 0; i < LED_COUNT; i++) {
    uint32_t c = frame[i];
    uint8_t r = (uint8_t)(((c >> 16) & 0xFF) * BRIGHTNESS / 255);
    uint8_t g = (uint8_t)(((c >> 8) & 0xFF) * BRIGHTNESS / 255);
    uint8_t b = (uint8_t)((c & 0xFF) * BRIGHTNESS / 255);
    uint8_t bytes[4] = { g, r, b, 0 };
    for (int j = 0; j < BYTES_PER_LED; j++)
      for (int bit = 7; bit >= 0; bit--) {
        bool one = bytes[j] & (1 << bit);
        symbols[k].level0 = 1;
        symbols[k].duration0 = one ? 8 : 4;
        symbols[k].level1 = 0;
        symbols[k].duration1 = one ? 4 : 9;
        k++;
      }
  }
  rmtWrite(DATA_PIN, symbols, k, 10);
  memcpy(shown, frame, sizeof(frame));
}

static uint32_t scale(uint32_t c, uint8_t pct)
{
  return ((((c >> 16) & 0xFF) * pct / 100) << 16) | ((((c >> 8) & 0xFF) * pct / 100) << 8) | ((c & 0xFF) * pct / 100);
}

static void begin()
{
  rmtOk = rmtInit(DATA_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  for (int i = 0; i < LED_COUNT; i++) { frame[i] = 0; shown[i] = 1; }   // force the first send
  show();
}

// Call every frame with the current RPM (0 or idle when there's no data).
static void update(float rpm, uint32_t ms)
{
  if (!isfinite(rpm)) rpm = 0;
  const int seg = MIRROR ? STICK_LEDS : LED_COUNT;   // LEDs in one pattern
  const int barN = seg - SHIFT_LEDS;
  int lit = ledsFor(rpm, barN);
  uint32_t shiftColor = 0;
  if (rpm >= dash::SHIFT_RPM) shiftColor = ((ms / 70) % 2) ? 0xFF0000 : 0xFFC000;   // red / yellow strobe ~7 Hz
  else if (rpm >= dash::SHIFT_RPM * WARN_PCT / 100) shiftColor = 0x0040FF;           // blue: get ready

  for (int i = 0; i < LED_COUNT; i++) {
    int p = MIRROR ? i % STICK_LEDS : i;             // position within the pattern
    uint32_t c;
    if (p < barN) {
      float pos = (p + 0.5f) / barN;
      uint32_t zone = pos < GREEN_UNTIL ? 0x00FF00 : pos < YELLOW_UNTIL ? 0xFFB000 : 0xFF0000;
      c = p < lit ? zone : scale(zone, UNLIT_DIM);
    } else {
      c = shiftColor;
    }
    frame[i] = enabled ? c : 0;
  }
  show();
}

#else
// ---- LM3914 bar + RGB LED -------------------------------------------------------------
// The LM3914 lights LED k above k x 0.125 V. PWM at 78 kHz through 10k/1uF
// gives a steady voltage (~10 mV ripple); we aim mid-step so nothing flickers.
enum Colour { OFF, RED, GREEN, BLUE, YELLOW };
static volatile Colour colour = OFF;

static void writeRgb(bool r, bool g, bool b)
{
  digitalWrite(R_PIN, r != COMMON_ANODE);
  digitalWrite(G_PIN, g != COMMON_ANODE);
  digitalWrite(B_PIN, b != COMMON_ANODE);
}

// every 1 ms: the RGB LED has one shared resistor, so only one colour may be
// on at a time; yellow = red and green alternating faster than the eye sees
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

static void setBar(int n)
{
  n = constrain(n, 0, 10);
  float volts = (n == 0) ? 0.0f : (n + 0.5f) * 0.125f;
  ledcWrite(DATA_PIN, constrain((int)(volts / 3.3f * 1023 + 0.5f), 0, 1023));
}

static void begin()
{
  ledcAttach(DATA_PIN, 78125, 10);   // 80 MHz / 1024 = 78 kHz, 10-bit duty
  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);
  writeRgb(false, false, false);
  esp_timer_create_args_t args = {};
  args.callback = tick;
  args.name = "shiftrgb";
  esp_timer_handle_t timer;
  if (esp_timer_create(&args, &timer) == ESP_OK) esp_timer_start_periodic(timer, 1000);
  setBar(0);
}

static void update(float rpm, uint32_t ms)
{
  if (!enabled) { setBar(0); colour = OFF; return; }
  if (!isfinite(rpm)) rpm = 0;
  setBar(ledsFor(rpm, 10));
  if (rpm >= dash::SHIFT_RPM) colour = ((ms / 70) % 2) ? RED : YELLOW;
  else if (rpm >= dash::SHIFT_RPM * WARN_PCT / 100) colour = BLUE;
  else colour = OFF;
}
#endif

}  // namespace shift
