// Shift lights: addressable LED sticks (WS2812B / SK6812), driven from the
// live RPM.
//
// Wiring - ONE data wire, sticks chained:
//   ESP32 GPIO14 -> 270 ohm -> stick 1 DIN,   stick 1 DOUT -> stick 2 DIN
//   5V -> ESP32 VIN, GND -> GND on every stick
//
// Layout: the first LEDs are an RPM bar (green, yellow, red by position);
// the last SHIFT_LEDS are a separate shift light: blue near the shift point,
// strobing red/yellow at and above it. MIRROR shows the same pattern on every
// stick (e.g. one stick each side of the screen).
//
// Bar: empty at BAR_START_RPM, full at dash::SHIFT_RPM (normal or sport).
// The RPM-lights switch sets `enabled`; off = all dark.
// Driven by our own small RMT driver (no library needed).
#pragma once
#include "esp32-hal-rmt.h"

namespace shift {

// ---- settings ----------------------------------------------------------------
constexpr int   DATA_PIN = 14;
constexpr float BAR_START_RPM = 900;      // just above idle (780), so the bar is dark at idle
constexpr int   WARN_PCT = 90;            // shift light turns blue at this % of the shift point
constexpr int   LED_COUNT   = 16;         // all LEDs on the chain (2 sticks x 8)
constexpr int   STICK_LEDS  = 8;          // LEDs per stick (for MIRROR)
constexpr bool  MIRROR      = false;      // true: every stick shows the same 8-LED pattern
constexpr int   SHIFT_LEDS  = 2;          // last LEDs of the pattern = shift light
constexpr uint8_t BRIGHTNESS = 48;        // 0-255; keeps current low (each LED up to 60 mA at 255 white)
constexpr uint8_t UNLIT_DIM  = 6;         // unlit bar LEDs glow at this % of their colour (0 = off)
constexpr float GREEN_UNTIL  = 0.55f;     // bar position (0-1) where green ends
constexpr float YELLOW_UNTIL = 0.80f;     // ...where yellow ends; red after that
constexpr bool  RGBW         = false;     // true if the sticks are RGBW (4 bytes per LED)

static bool enabled = true;               // the RPM-lights switch

static int ledsFor(float rpm, int n)      // RPM -> how many of n bar LEDs are lit
{
  if (rpm <= BAR_START_RPM) return 0;
  float frac = (rpm - BAR_START_RPM) / (dash::SHIFT_RPM - BAR_START_RPM);
  return constrain((int)ceilf(frac * n), 0, n);
}

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

}  // namespace shift
