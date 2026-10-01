// Dashboard renderer + fake car data.
//
// Everything here draws into a LovyanGFX canvas (a 4-bit palette sprite),
// so the exact same code runs on the ESP32 and in the desktop preview.
// Include <LovyanGFX.hpp> before this file.
#pragma once

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace dash {

using Gfx = lgfx::LovyanGFX;
using Datum = lgfx::textdatum_t;

constexpr int W = 360;
constexpr int H = 240;

// ---- car constants (2000 Focus Zetec, 5-speed manual) --------------------
constexpr float RPM_MAX   = 7000;
constexpr float SHIFT_RPM = 6000;   // shift light starts flashing here
constexpr float IDLE_RPM  = 780;
// mph per 1000 rpm in each gear (index 0 = neutral)
constexpr float GEAR_MPH_PER_K[6] = { 0, 5.6f, 10.1f, 14.6f, 19.6f, 24.3f };

// ---- colors ------------------------------------------------------------------
// These screens draw into an 8-bit RGB332 canvas (the same format as the video
// output), where a uint8_t color IS the RGB332 value: RRRGGGBB.
// Shown next to each is the full RGB color it produces.
enum : uint8_t {
  BG       = 0x00, // 000000
  PANEL    = 0x05, // 002455 navy
  GRID     = 0x29, // 244955 dark teal
  DIM      = 0x4E, // 496DAA labels
  TEXT     = 0xBB, // B6DBFF
  WHITE    = 0xFF, // FFFFFF
  CYAN     = 0x1F, // 00FFFF main accent
  TEAL     = 0x16, // 00B6AA
  GREEN    = 0x3D, // 24FF55
  YELLOW   = 0xF8, // FFDB00
  ORANGE   = 0xEC, // FF6D00
  RED      = 0xE5, // FF2455
  MAGENTA  = 0xE2, // FF00AA
  PURPLE   = 0x67, // 6D24FF
  PHOS_DK  = 0x04, // 002400 terminal background band
  PHOS_MID = 0x35, // 24B655 terminal dim green
};

// ---- data --------------------------------------------------------------------
struct CarData {
  float rpm = IDLE_RPM, mph = 0, throttle = 0, load = 20, timing = 12;
  float coolantF = 150, iatF = 88, maf = 3, volts = 14.1f, stft = 0;
  float mpg = 0;          // derived: 710.7 * mph / MAF (gasoline, stoich)
  int gear = 0;           // 0 = neutral / clutch in
  float peakRpm = 0, peakMph = 0;
};

// Fixed-size ring buffer of samples for the graphs.
struct History {
  static constexpr int N = 360;
  float v[N] = {};
  int head = 0, count = 0;
  void push(float x) { v[head] = x; head = (head + 1) % N; if (count < N) count++; }
  // i = 0 is the oldest sample still stored
  float at(int i) const { return v[(head - count + i + N * 2) % N]; }
};

struct EventLog {
  static constexpr int N = 6;
  char lines[N][48] = {};
  int count = 0;
  void add(float t, const char* fmt, ...) {
    // shift up and append at the end
    if (count == N) { memmove(lines[0], lines[1], sizeof(lines[0]) * (N - 1)); count--; }
    int m = (int)(t / 60), s = (int)t % 60, ds = (int)(t * 10) % 10;
    int n = snprintf(lines[count], sizeof(lines[0]), "%02d:%02d.%d ", m, s, ds);
    va_list ap; va_start(ap, fmt);
    vsnprintf(lines[count] + n, sizeof(lines[0]) - n, fmt, ap);
    va_end(ap);
    count++;
  }
};

// One value from the car plus how its last request went (for the DIAG screen).
struct Reading {
  const char* name = "";
  const char* unit = "";
  uint8_t decimals = 0;
  float value = 0;
  char status[10] = "--";     // "OK", "NO DATA", "TIMEOUT", ...
  uint8_t statusColor = DIM;
  uint32_t lastOkMs = 0;      // millis() of the last good answer, 0 = never
};

// Everything the screens draw from. Filled in by the Simulator (fake data)
// or by the OBD link (obd.h). The screens don't know which.
struct Telemetry {
  CarData d;
  History rpmHist, mphHist, thrHist, loadHist;
  EventLog log;
  float clock = 0;                  // seconds since start
  const char* sourceTag = "SIM";    // header tag: SIM / LIVE / NO LINK
  uint8_t sourceColor = YELLOW;

  // connection and per-value status, for the DIAG screen
  char linkLine[48] = "";
  uint8_t linkColor = DIM;
  char elmLine[48] = "";
  uint8_t elmColor = DIM;
  float repliesPerSec = 0;
  char memLine[40] = "";
  static constexpr int MAX_READINGS = 14;
  Reading readings[MAX_READINGS];
  int readingCount = 0;             // 0 = no OBD (simulator)
  bool milOn = false;               // check engine light
  int codeCount = 0;
  char codes[8][6] = {};
  int codesListed = 0;
  bool codesRead = false;

  // Call once per frame after d is updated: feeds the graphs and peaks.
  void sample(float dt) {
    if (d.rpm > d.peakRpm) d.peakRpm = d.rpm;
    if (d.mph > d.peakMph) d.peakMph = d.mph;
    sampleT += dt;
    while (sampleT >= 0.1f) {         // graphs sample at 10 Hz
      sampleT -= 0.1f;
      rpmHist.push(d.rpm);
      mphHist.push(d.mph);
      thrHist.push(d.throttle);
      loadHist.push(d.load);
    }
  }

private:
  float sampleT = 0;
};

// ---- fake driving ------------------------------------------------------------
// Loops: idle -> pull through the gears -> cruise -> brake -> idle.
// Alternates between a hard pull (shift at redline) and a gentle one.
class Simulator : public Telemetry {
public:
  Simulator() { log.add(0, "LINK UP  J1850 PWM (SIM)"); }

  void update(float dt) {
    if (dt > 0.1f) dt = 0.1f;   // don't jump after a stall
    clock += dt;
    phaseT += dt;
    float targetThr = 0;

    switch (phase) {
    case IDLE:
      d.gear = 0;
      d.mph = 0;
      d.rpm = approach(d.rpm, IDLE_RPM + 15 * noise(3.1f), dt * 4);
      if (phaseT > 4) {
        hard = !hard;
        shiftAt = hard ? 6200 : 3000;
        d.gear = 1;
        setPhase(PULL);
        log.add(clock, hard ? "LAUNCH  WOT" : "LAUNCH  EASY");
      }
      break;

    case PULL: {
      if (shiftPause > 0) {
        shiftPause -= dt;
        targetThr = 0;
      } else {
        targetThr = hard ? 94 : 42;
      }
      static const float accel[6] = { 0, 8.5f, 6.0f, 4.2f, 3.0f, 2.3f };
      d.mph += (d.throttle / 100.0f * accel[d.gear] * (hard ? 1.0f : 1.6f) - 0.0004f * d.mph * d.mph) * dt;
      float geared = d.mph * 1000 / GEAR_MPH_PER_K[d.gear];
      if (d.gear == 1 && geared < 1400) geared = 1400 + d.throttle * 8;  // clutch slip
      d.rpm = shiftPause > 0 ? approach(d.rpm, geared, dt * 10) : geared;
      if (d.rpm >= shiftAt && d.gear < 5) {
        if (d.rpm > SHIFT_RPM) log.add(clock, "SHIFT %d>%d  @%d  REDLINE", d.gear, d.gear + 1, (int)d.rpm);
        else log.add(clock, "SHIFT %d>%d  @%d", d.gear, d.gear + 1, (int)d.rpm);
        d.gear++;
        shiftPause = 0.35f;
      }
      // easy pulls top out near 61 mph in 5th, so cruise starts at 58
      if (d.mph >= 58) { setPhase(CRUISE); log.add(clock, "CRUISE  65 MPH"); }
      break;
    }

    case CRUISE:
      // short-shift up to 5th, then hold ~65 mph
      if (d.gear < 5 && phaseT > 0.5f * d.gear) d.gear++;
      targetThr = 18 + 4 * sinf(clock * 0.7f);
      d.mph = approach(d.mph, 65 + 2 * sinf(clock * 0.3f), dt * 0.6f);
      d.rpm = approach(d.rpm, d.mph * 1000 / GEAR_MPH_PER_K[d.gear], dt * 6);
      if (phaseT > 8) { setPhase(DECEL); log.add(clock, "DECEL  FUEL CUT"); }
      break;

    case DECEL:
      targetThr = 0;
      d.mph -= (3.5f + d.mph * 0.03f) * dt;
      if (d.gear > 1 && d.rpm < 1300) d.gear--;
      if (d.mph < 8) d.gear = 0;
      if (d.gear > 0) d.rpm = approach(d.rpm, d.mph * 1000 / GEAR_MPH_PER_K[d.gear], dt * 8);
      else d.rpm = approach(d.rpm, IDLE_RPM, dt * 3);
      if (d.mph <= 0) {
        d.mph = 0;
        setPhase(IDLE);
        log.add(clock, "IDLE  %d RPM", (int)IDLE_RPM);
      }
      break;
    }

    // smooth the throttle like a real foot
    d.throttle = approach(d.throttle, targetThr, dt * 8);
    bool fuelCut = phase == DECEL && d.gear > 0;

    d.load = fuelCut ? 8 + 2 * noise(1.3f)
           : clampf(18 + d.throttle * 0.78f + d.rpm / RPM_MAX * 6, 0, 100);
    d.timing = clampf(10 + d.rpm / 6500 * 22 - d.load * 0.1f + 0.8f * noise(5.7f), -5, 45);
    d.maf = 1.2f + d.rpm * d.load / 100 * 0.019f;
    d.mpg = d.mph > 1 ? clampf(710.7f * d.mph / d.maf, 0, 99.9f) : 0;
    d.stft = fuelCut ? 0 : 2.5f * noise(0.9f) + 1.0f * noise(7.3f);

    // coolant warms up to the thermostat and hovers there
    float before = d.coolantF;
    d.coolantF = approach(d.coolantF, 196 + 3 * noise(0.05f), dt * 0.05f);
    if (before < 190 && d.coolantF >= 190) log.add(clock, "COOLANT  AT TEMP  %dF", (int)d.coolantF);
    d.iatF = approach(d.iatF, d.mph < 5 ? 104 : 86, dt * 0.04f);
    d.volts = (d.rpm < 1000 ? 13.9f : 14.3f) + 0.05f * noise(2.2f);

    sample(dt);
  }

  static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

private:
  enum Phase { IDLE, PULL, CRUISE, DECEL };
  Phase phase = IDLE;
  float phaseT = 0, shiftPause = 0, shiftAt = 3000;
  bool hard = false;

  void setPhase(Phase p) { phase = p; phaseT = 0; }

  // exponential ease toward a target; k = rate per second
  static float approach(float x, float target, float k) {
    if (k > 1) k = 1;
    return x + (target - x) * k;
  }

  // smooth pseudo-random wobble in [-1, 1]
  float noise(float speed) const {
    float t = clock * speed;
    return 0.6f * sinf(t * 1.7f + speed) + 0.4f * sinf(t * 3.9f + speed * 2.3f);
  }
};

// ---- drawing helpers --------------------------------------------------------
static inline float frac(float v, float lo, float hi) { return Simulator::clampf((v - lo) / (hi - lo), 0, 1); }

static void txt(Gfx& g, const char* s, int x, int y, uint8_t color,
                const lgfx::IFont* font = &fonts::Font0, Datum datum = Datum::top_left, float size = 1)
{
  g.setFont(font);
  g.setTextSize(size);
  g.setTextDatum(datum);
  g.setTextColor(color);
  g.drawString(s, x, y);
}

static uint8_t zoneRpm(float pos) {
  float r = pos * RPM_MAX;
  return r < 4500 ? GREEN : r < 5500 ? YELLOW : r < SHIFT_RPM ? ORANGE : RED;
}
static uint8_t zoneCyan(float)  { return CYAN; }
static uint8_t zoneTeal(float)  { return TEAL; }
static uint8_t zoneHeat(float pos) { return pos < 0.6f ? TEAL : pos < 0.85f ? YELLOW : RED; }
static uint8_t zoneMag(float)   { return MAGENTA; }
static uint8_t zoneRed(float)   { return RED; }
static uint8_t zoneWhite(float) { return WHITE; }

// btop-style segmented meter (horizontal)
static void segBar(Gfx& g, int x, int y, int w, int h, float f, uint8_t (*zone)(float),
                   int seg = 4, int gap = 1, uint8_t off = GRID)
{
  int n = (w + gap) / (seg + gap);
  int lit = (int)(Simulator::clampf(f, 0, 1) * n + 0.5f);
  for (int i = 0; i < n; i++) {
    g.fillRect(x + i * (seg + gap), y, seg, h, i < lit ? zone((i + 0.5f) / n) : off);
  }
}

// segmented meter (vertical, fills from the bottom)
static void segVBar(Gfx& g, int x, int y, int w, int h, float f, uint8_t (*zone)(float),
                    int seg = 3, int gap = 1)
{
  int n = (h + gap) / (seg + gap);
  int lit = (int)(Simulator::clampf(f, 0, 1) * n + 0.5f);
  for (int i = 0; i < n; i++) {
    int yy = y + h - (i + 1) * (seg + gap) + gap;
    g.fillRect(x, yy, w, seg, i < lit ? zone((i + 0.5f) / n) : (uint8_t)GRID);
  }
}

// Box with a title notch in the top border and bright corner brackets.
static void panel(Gfx& g, int x, int y, int w, int h, const char* title, uint8_t accent)
{
  g.drawRect(x, y, w, h, (uint8_t)GRID);
  const int c = 6;
  g.drawFastHLine(x, y, c, accent);             g.drawFastVLine(x, y, c, accent);
  g.drawFastHLine(x + w - c, y, c, accent);     g.drawFastVLine(x + w - 1, y, c, accent);
  g.drawFastHLine(x, y + h - 1, c, accent);     g.drawFastVLine(x, y + h - c, c, accent);
  g.drawFastHLine(x + w - c, y + h - 1, c, accent); g.drawFastVLine(x + w - 1, y + h - c, c, accent);
  if (title && *title) {
    g.setFont(&fonts::Font0);
    g.setTextSize(1);
    int tw = g.textWidth(title);
    g.fillRect(x + 8, y - 3, tw + 6, 8, (uint8_t)BG);
    txt(g, title, x + 11, y - 3, accent);
  }
}

// Scrolling line graph, newest sample on the right.
static void graph(Gfx& g, int x, int y, int w, int h, const History& hist, float maxV,
                  uint8_t line, uint8_t fill, float mark = -1)
{
  for (int i = 1; i < 4; i++) {
    int gy = y + h * i / 4;
    for (int xx = x; xx < x + w; xx += 4) g.drawPixel(xx, gy, (uint8_t)GRID);
  }
  for (int xx = x + w - 1; xx > x; xx -= 50) {
    for (int yy = y; yy < y + h; yy += 4) g.drawPixel(xx, yy, (uint8_t)GRID);
  }
  int n = hist.count < w ? hist.count : w;
  int prevY = 0;
  for (int i = 0; i < n; i++) {
    int col = x + w - n + i;
    float v = hist.at(hist.count - n + i);
    int py = y + h - 1 - (int)(frac(v, 0, maxV) * (h - 1));
    g.drawFastVLine(col, py, y + h - py, fill);
    if (i > 0) g.drawLine(col - 1, prevY, col, py, line);
    else g.drawPixel(col, py, line);
    prevY = py;
  }
  if (mark >= 0) {
    int my = y + h - 1 - (int)(frac(mark, 0, maxV) * (h - 1));
    for (int xx = x; xx < x + w; xx += 6) g.drawFastHLine(xx, my, 3, (uint8_t)RED);
  }
}

// small "°F" style unit: tiny circle + letter
static void degUnit(Gfx& g, int x, int y, const char* letter, uint8_t color)
{
  g.drawCircle(x + 1, y + 1, 1, color);
  txt(g, letter, x + 4, y, color);
}

static bool shiftFlash(const CarData& d, uint32_t ms) { return d.rpm >= SHIFT_RPM && (ms / 100) % 2 == 0; }

static void header(Gfx& g, int idx, int count, const char* name, const Telemetry& t, uint32_t ms)
{
  g.fillRect(0, 0, W, 13, (uint8_t)PANEL);
  g.drawFastHLine(0, 13, W, (uint8_t)GRID);
  g.fillRect(0, 0, 3, 13, (uint8_t)CYAN);
  txt(g, "FOCUS//SE", 8, 3, CYAN);
  txt(g, name, 70, 3, WHITE);

  // page dots
  for (int i = 0; i < count; i++) {
    int px = W / 2 - count * 5 + i * 10 + 18;
    if (i == idx) g.fillRect(px, 4, 6, 6, (uint8_t)CYAN);
    else g.drawRect(px, 4, 6, 6, (uint8_t)DIM);
  }

  char buf[24];
  int m = (int)(t.clock / 60), s = (int)t.clock % 60;
  snprintf(buf, sizeof(buf), "%02d:%02d", m, s);
  txt(g, buf, W - 6, 3, TEXT, &fonts::Font0, Datum::top_right);
  txt(g, t.sourceTag, W - 44, 3, t.sourceColor, &fonts::Font0, Datum::top_right);
  if ((ms / 500) % 2) g.fillRect(W - 50 - g.textWidth(t.sourceTag), 5, 4, 4, t.sourceColor);
}

// ---- screen 1: HUD -----------------------------------------------------------
// Driver focus: shift light bar, huge RPM, gear.
static void drawHud(Gfx& g, const Telemetry& s, uint32_t ms)
{
  const CarData& d = s.d;
  char buf[16];

  // shift light bar: fills by rpm, whole bar flashes past the shift point
  bool flash = shiftFlash(d, ms);
  if (d.rpm >= SHIFT_RPM) segBar(g, 8, 19, 344, 16, 1.0f, flash ? zoneRed : zoneWhite, 6, 2);
  else segBar(g, 8, 19, 344, 16, d.rpm / RPM_MAX, zoneRpm, 6, 2);
  for (int k = 0; k <= 7; k++) {
    snprintf(buf, sizeof(buf), "%d", k);
    int tx = 8 + 344 * k / 7;
    txt(g, buf, tx, 38, k >= 6 ? RED : DIM, &fonts::Font0, k == 0 ? Datum::top_left : k == 7 ? Datum::top_right : Datum::top_center);
  }

  // big RPM with ghosted 7-segment digits behind it
  panel(g, 8, 52, 244, 76, "ENGINE RPM", CYAN);
  txt(g, "8888", 242, 58, GRID, &fonts::Font7, Datum::top_right, 1.3f);
  snprintf(buf, sizeof(buf), "%d", (int)(d.rpm / 10) * 10);
  txt(g, buf, 242, 58, d.rpm >= SHIFT_RPM ? (flash ? RED : WHITE) : WHITE, &fonts::Font7, Datum::top_right, 1.3f);
  snprintf(buf, sizeof(buf), "PEAK %d", (int)d.peakRpm);
  txt(g, buf, 14, 116, DIM);

  // gear
  panel(g, 258, 52, 94, 76, "GEAR", CYAN);
  if (d.gear == 0) snprintf(buf, sizeof(buf), "N");
  else snprintf(buf, sizeof(buf), "%d", d.gear);
  txt(g, buf, 305, 92, d.gear == 0 ? YELLOW : CYAN, &fonts::AsciiFont24x48, Datum::middle_center, 1.25f);

  // middle row: speed / load / timing
  panel(g, 8, 136, 128, 44, "SPEED", TEAL);
  snprintf(buf, sizeof(buf), "%d", (int)(d.mph + 0.5f));
  txt(g, buf, 96, 143, WHITE, &fonts::AsciiFont8x16, Datum::top_right, 2);
  txt(g, "MPH", 100, 165, DIM);

  panel(g, 142, 136, 102, 44, "LOAD", TEAL);
  snprintf(buf, sizeof(buf), "%d%%", (int)d.load);
  txt(g, buf, 236, 143, TEXT, &fonts::AsciiFont8x16, Datum::top_right);
  segBar(g, 148, 164, 90, 8, d.load / 100, zoneTeal);

  panel(g, 250, 136, 102, 44, "TIMING", TEAL);
  snprintf(buf, sizeof(buf), "%.1f", d.timing);
  txt(g, buf, 306, 143, TEXT, &fonts::AsciiFont8x16, Datum::top_right);
  txt(g, "BTDC", 344, 147, DIM, &fonts::Font0, Datum::top_right);
  segBar(g, 256, 164, 90, 8, d.timing / 40, zoneTeal);

  // bottom row: four small stats
  struct Stat { const char* name; float v; const char* fmt; float lo, hi; uint8_t (*zone)(float); bool warn; };
  Stat stats[4] = {
    { "CLT",  d.coolantF, "%.0f",   100, 240, zoneHeat, d.coolantF > 225 },
    { "BATT", d.volts,    "%.1fV",  11,  15,  zoneCyan, d.volts < 12.5f },
    { "THR",  d.throttle, "%.0f%%", 0,   100, zoneMag,  false },
    { "MPG",  d.mpg,      "%.1f",   0,   50,  zoneCyan, false },
  };
  for (int i = 0; i < 4; i++) {
    int x = 8 + i * 87;
    panel(g, x, 188, 83, 44, stats[i].name, stats[i].warn ? RED : DIM);
    if (i == 3 && d.mph < 1) snprintf(buf, sizeof(buf), "--");
    else snprintf(buf, sizeof(buf), stats[i].fmt, stats[i].v);
    txt(g, buf, x + 76, 195, stats[i].warn ? RED : TEXT, &fonts::AsciiFont8x16, Datum::top_right);
    if (i == 0) degUnit(g, x + 6, 199, "F", DIM);
    segBar(g, x + 6, 216, 71, 8, frac(stats[i].v, stats[i].lo, stats[i].hi), stats[i].zone);
  }
}

// ---- screen 2: GRID ----------------------------------------------------------
// Everything at once, btop style.
static void drawGrid(Gfx& g, const Telemetry& s, uint32_t)
{
  const CarData& d = s.d;
  struct Cell { const char* name; float v; const char* fmt; const char* unit; float lo, hi; uint8_t (*zone)(float); uint8_t color; };
  Cell cells[12] = {
    { "RPM",       d.rpm,         "%.0f",  "",    0,   RPM_MAX, zoneRpm,  WHITE },
    { "SPEED",     d.mph,         "%.0f",  "MPH", 0,   120,     zoneCyan, WHITE },
    { "GEAR",      (float)d.gear, "%.0f",  "",    0,   5,       zoneCyan, CYAN },
    { "THROTTLE",  d.throttle,    "%.0f",  "%",   0,   100,     zoneMag,  TEXT },
    { "LOAD",      d.load,        "%.0f",  "%",   0,   100,     zoneTeal, TEXT },
    { "TIMING",    d.timing,      "%.1f",  "DEG", 0,   40,      zoneTeal, TEXT },
    { "MAF",       d.maf,         "%.1f",  "G/S", 0,   120,     zoneTeal, TEXT },
    { "MPG",       d.mpg,         "%.1f",  "",    0,   50,      zoneCyan, GREEN },
    { "COOLANT",   d.coolantF,    "%.0f",  "F",   100, 240,     zoneHeat, (uint8_t)(d.coolantF > 225 ? RED : TEXT) },
    { "INTAKE",    d.iatF,        "%.0f",  "F",   40,  160,     zoneHeat, TEXT },
    { "BATTERY",   d.volts,       "%.1f",  "V",   11,  15,      zoneCyan, (uint8_t)(d.volts < 12.5f ? RED : TEXT) },
    { "FUEL TRIM", d.stft,        "%+.1f", "%",   -10, 10,      zoneCyan, TEXT },
  };
  char buf[16];
  for (int i = 0; i < 12; i++) {
    int x = 8 + (i % 4) * 87;
    int y = 22 + (i / 4) * 72;
    const Cell& c = cells[i];
    panel(g, x, y, 83, 66, c.name, i < 3 ? CYAN : DIM);

    if (i == 2 && d.gear == 0) snprintf(buf, sizeof(buf), "N");
    else if (i == 7 && d.mph < 1) snprintf(buf, sizeof(buf), "--");
    else snprintf(buf, sizeof(buf), c.fmt, c.v);
    txt(g, buf, x + 76, y + 12, c.color, &fonts::AsciiFont8x16, Datum::top_right, 2);
    txt(g, c.unit, x + 6, y + 36, DIM);

    if (i == 11) {
      // fuel trim: bar grows out from the center
      int cx = x + 41;
      g.drawFastVLine(cx, y + 48, 12, (uint8_t)DIM);
      int len = (int)(frac(fabsf(c.v), 0, 10) * 34);
      if (c.v >= 0) g.fillRect(cx + 2, y + 50, len, 8, (uint8_t)CYAN);
      else g.fillRect(cx - 1 - len, y + 50, len, 8, (uint8_t)ORANGE);
    } else {
      segBar(g, x + 6, y + 50, 71, 8, frac(c.v, c.lo, c.hi), c.zone);
    }
  }
}

// ---- screen 3: SCOPE ---------------------------------------------------------
// Oscilloscope-style history graphs.
static void drawScope(Gfx& g, const Telemetry& s, uint32_t)
{
  const CarData& d = s.d;
  char buf[24];

  panel(g, 8, 20, 344, 124, "RPM // 34 SEC", CYAN);
  graph(g, 10, 26, 340, 116, s.rpmHist, RPM_MAX, CYAN, PANEL, SHIFT_RPM);
  txt(g, "7K", 12, 27, DIM);
  txt(g, "SHIFT", 12, 26 + 116 - (int)(SHIFT_RPM / RPM_MAX * 115) - 10, RED);
  snprintf(buf, sizeof(buf), "%d", (int)d.rpm);
  g.fillRect(262, 28, 86, 20, (uint8_t)BG);
  txt(g, buf, 344, 30, d.rpm >= SHIFT_RPM ? RED : WHITE, &fonts::AsciiFont8x16, Datum::top_right);
  snprintf(buf, sizeof(buf), "G%d", d.gear);
  txt(g, d.gear ? buf : "N", 268, 30, YELLOW, &fonts::AsciiFont8x16);

  panel(g, 8, 152, 170, 82, "THROTTLE %", MAGENTA);
  graph(g, 10, 158, 166, 74, s.thrHist, 100, MAGENTA, PURPLE);
  snprintf(buf, sizeof(buf), "%d", (int)d.throttle);
  txt(g, buf, 172, 160, WHITE, &fonts::AsciiFont8x16, Datum::top_right);

  panel(g, 182, 152, 170, 82, "SPEED MPH", GREEN);
  graph(g, 184, 158, 166, 74, s.mphHist, 100, GREEN, PHOS_DK);
  snprintf(buf, sizeof(buf), "%d", (int)(d.mph + 0.5f));
  txt(g, buf, 346, 160, WHITE, &fonts::AsciiFont8x16, Datum::top_right);
}

// ---- screen 4: TERM ----------------------------------------------------------
// Green phosphor terminal readout with an event log.
static void termBar(char* out, float f, int width)
{
  int lit = (int)(Simulator::clampf(f, 0, 1) * width + 0.5f);
  out[0] = '[';
  for (int i = 0; i < width; i++) out[1 + i] = i < lit ? '#' : '.';
  out[width + 1] = ']';
  out[width + 2] = 0;
}

static void drawTerm(Gfx& g, const Telemetry& s, uint32_t ms)
{
  const CarData& d = s.d;
  g.fillRect(0, 0, W, 13, (uint8_t)PHOS_MID);
  txt(g, "FOCUS-SE OBD-II MON v0.1   J1850-PWM", 4, 3, BG);
  txt(g, "5/7 TERM", W - 4, 3, BG, &fonts::Font0, Datum::top_right);

  struct Row { const char* name; const char* unit; float f; bool warn; char val[12]; };
  Row rows[9] = {
    { "RPM",      "",    d.rpm / RPM_MAX,             d.rpm >= SHIFT_RPM, "" },
    { "SPEED",    "MPH", d.mph / 120,                 false,              "" },
    { "GEAR",     "",    0,                           false,              "" },
    { "THROTTLE", "%",   d.throttle / 100,            false,              "" },
    { "LOAD",     "%",   d.load / 100,                false,              "" },
    { "TIMING",   "DEG", d.timing / 40,               false,              "" },
    { "MAF",      "G/S", d.maf / 120,                 false,              "" },
    { "COOLANT",  "F",   frac(d.coolantF, 100, 240),  d.coolantF > 225,   "" },
    { "BATTERY",  "V",   frac(d.volts, 11, 15),       d.volts < 12.5f,    "" },
  };
  snprintf(rows[0].val, 12, "%d", (int)d.rpm);
  snprintf(rows[1].val, 12, "%d", (int)(d.mph + 0.5f));
  snprintf(rows[3].val, 12, "%d", (int)d.throttle);
  snprintf(rows[4].val, 12, "%d", (int)d.load);
  snprintf(rows[5].val, 12, "%.1f", d.timing);
  snprintf(rows[6].val, 12, "%.1f", d.maf);
  snprintf(rows[7].val, 12, "%d", (int)d.coolantF);
  snprintf(rows[8].val, 12, "%.2f", d.volts);

  char bar[32];
  for (int i = 0; i < 9; i++) {
    int y = 17 + i * 16;
    if (i % 2) g.fillRect(0, y, W, 16, (uint8_t)PHOS_DK);
    uint8_t vc = rows[i].warn ? YELLOW : GREEN;
    txt(g, ">", 4, y, PHOS_MID, &fonts::AsciiFont8x16);
    txt(g, rows[i].name, 16, y, PHOS_MID, &fonts::AsciiFont8x16);
    txt(g, rows[i].val, 144, y, vc, &fonts::AsciiFont8x16, Datum::top_right);
    txt(g, rows[i].unit, 150, y, PHOS_MID, &fonts::AsciiFont8x16);
    if (i == 2) {
      // gear selector strip:  N 1 2 [3] 4 5
      int gx = 188;
      for (int k = 0; k <= 5; k++) {
        char c[2] = { k ? (char)('0' + k) : 'N', 0 };
        if (k == d.gear) {
          g.fillRect(gx - 3, y, 14, 16, (uint8_t)GREEN);
          txt(g, c, gx, y, BG, &fonts::AsciiFont8x16);
        } else {
          txt(g, c, gx, y, PHOS_MID, &fonts::AsciiFont8x16);
        }
        gx += 24;
      }
    } else {
      termBar(bar, rows[i].f, 18);
      txt(g, bar, 184, y, vc, &fonts::AsciiFont8x16);
    }
  }

  // event log
  int ly = 17 + 9 * 16 + 4;
  g.drawFastHLine(0, ly, W, (uint8_t)PHOS_MID);
  g.fillRect(10, ly - 4, 70, 9, (uint8_t)BG);
  txt(g, " EVENT LOG", 10, ly - 3, GREEN);
  int shown = s.log.count < 5 ? s.log.count : 5;
  for (int i = 0; i < shown; i++) {
    const char* line = s.log.lines[s.log.count - shown + i];
    txt(g, line, 6, ly + 7 + i * 10, i == shown - 1 ? GREEN : PHOS_MID);
  }
  // prompt with blinking cursor
  int py = ly + 7 + 5 * 10 + 1;
  txt(g, "focus@esp32:~$", 6, py, GREEN);
  if ((ms / 400) % 2) g.fillRect(6 + 15 * 6, py, 6, 8, (uint8_t)GREEN);
}

// ---- screen 5: ARC -----------------------------------------------------------
// Radial segmented tach with side meters.
static void drawArc(Gfx& g, const Telemetry& s, uint32_t ms)
{
  const CarData& d = s.d;
  char buf[16];
  const int cx = 180, cy = 132;
  const float a0 = 135, span = 270;   // 0 deg = 3 o'clock, clockwise
  const int segs = 42;
  bool flash = shiftFlash(d, ms);

  // painted redline band on the outside
  g.fillArc(cx, cy, 106, 104, a0 + span * SHIFT_RPM / RPM_MAX, a0 + span, (uint8_t)RED);

  int lit = (int)(frac(d.rpm, 0, RPM_MAX) * segs + 0.5f);
  for (int i = 0; i < segs; i++) {
    float sa = a0 + span * i / segs;
    float ea = sa + span / segs - 1.6f;
    uint8_t c = GRID;
    if (i < lit) c = d.rpm >= SHIFT_RPM ? (flash ? RED : WHITE) : zoneRpm((i + 0.5f) / segs);
    g.fillArc(cx, cy, 100, 84, sa, ea, c);
  }
  // 1000 rpm labels
  for (int k = 0; k <= 7; k++) {
    float a = (a0 + span * k / 7) * 3.14159265f / 180;
    snprintf(buf, sizeof(buf), "%d", k);
    txt(g, buf, cx + (int)(74 * cosf(a)), cy + (int)(74 * sinf(a)), k >= 6 ? RED : DIM, &fonts::Font0, Datum::middle_center);
  }

  // center readout
  snprintf(buf, sizeof(buf), "%d", (int)(d.rpm / 10) * 10);
  txt(g, buf, cx, cy - 6, flash ? RED : WHITE, &fonts::Font7, Datum::middle_center);
  txt(g, "RPM", cx, cy + 24, DIM, &fonts::Font0, Datum::middle_center);
  if (d.gear == 0) snprintf(buf, sizeof(buf), "N");
  else snprintf(buf, sizeof(buf), "%d", d.gear);
  txt(g, buf, cx, cy + 66, d.gear ? CYAN : YELLOW, &fonts::AsciiFont24x48, Datum::middle_center);

  // left: throttle + load vertical meters
  txt(g, "THR", 23, 24, DIM, &fonts::Font0, Datum::top_center);
  txt(g, "LOAD", 50, 24, DIM, &fonts::Font0, Datum::top_center);
  segVBar(g, 16, 36, 14, 170, d.throttle / 100, zoneMag);
  segVBar(g, 43, 36, 14, 170, d.load / 100, zoneTeal);
  snprintf(buf, sizeof(buf), "%d", (int)d.throttle);
  txt(g, buf, 23, 212, TEXT, &fonts::Font0, Datum::top_center);
  snprintf(buf, sizeof(buf), "%d", (int)d.load);
  txt(g, buf, 50, 212, TEXT, &fonts::Font0, Datum::top_center);

  // right: speed + small stats
  txt(g, "SPEED", 352, 22, DIM, &fonts::Font0, Datum::top_right);
  snprintf(buf, sizeof(buf), "%d", (int)(d.mph + 0.5f));
  txt(g, buf, 352, 32, WHITE, &fonts::AsciiFont8x16, Datum::top_right, 2);
  txt(g, "MPH", 352, 66, DIM, &fonts::Font0, Datum::top_right);

  struct Mini { const char* name; bool warn; char val[10]; };
  Mini minis[3] = {
    { "CLT F",  d.coolantF > 225, "" },
    { "BATT V", d.volts < 12.5f,  "" },
    { "MPG",    false,            "" },
  };
  snprintf(minis[0].val, 10, "%d", (int)d.coolantF);
  snprintf(minis[1].val, 10, "%.1f", d.volts);
  if (d.mph < 1) snprintf(minis[2].val, 10, "--");
  else snprintf(minis[2].val, 10, "%.1f", d.mpg);
  for (int i = 0; i < 3; i++) {
    int y = 96 + i * 44;
    g.drawFastHLine(300, y, 52, (uint8_t)GRID);
    txt(g, minis[i].name, 352, y + 4, DIM, &fonts::Font0, Datum::top_right);
    txt(g, minis[i].val, 352, y + 16, minis[i].warn ? RED : TEXT, &fonts::AsciiFont8x16, Datum::top_right);
  }
}

// ---- boot splash -------------------------------------------------------------
static void drawBoot(Gfx& g, uint32_t ms)
{
  g.fillScreen((uint8_t)BG);
  static const char* lines[] = {
    "FOCUS//SE  DASH OS",
    "",
    "CVBS OUT ........ NTSC 360x240",
    "COLOR ........... RGB332 / 256",
    "OBD-II .......... J1850 PWM",
    "DATA SOURCE ..... SIMULATOR",
    "SCREENS ......... 7",
    "",
    "READY",
  };
  int shown = (int)(ms / 140);
  if (shown > 9) shown = 9;
  for (int i = 0; i < shown; i++) {
    txt(g, lines[i], 24, 40 + i * 18, i == 0 ? CYAN : i == 8 ? GREEN : TEXT, &fonts::AsciiFont8x16);
  }
  if ((ms / 300) % 2) g.fillRect(24, 40 + shown * 18, 8, 16, (uint8_t)CYAN);
}

// ---- screen 7: DIAG -----------------------------------------------------------
// Connection status and every OBD value with its last result, like the test sketch.
static void drawDiag(Gfx& g, const Telemetry& s, uint32_t ms)
{
  if (s.readingCount == 0) {
    txt(g, "SIMULATED DATA", W / 2, 100, YELLOW, &fonts::Font2, Datum::middle_center);
    txt(g, "no OBD link in this mode", W / 2, 122, DIM, &fonts::Font2, Datum::middle_center);
    return;
  }
  char b[72];
  txt(g, s.linkLine, 8, 17, s.linkColor, &fonts::Font2);
  if (s.elmColor == GREEN) snprintf(b, sizeof(b), "%s   %.1f replies/s", s.elmLine, s.repliesPerSec);
  else snprintf(b, sizeof(b), "%s", s.elmLine);
  txt(g, b, 8, 32, s.elmColor, &fonts::Font2);

  txt(g, s.memLine, 352, 19, DIM, &fonts::Font0, Datum::top_right);
  txt(g, "VALUE", 8, 49, DIM);
  txt(g, "READING", 196, 49, DIM, &fonts::Font0, Datum::top_right);
  txt(g, "STATUS", 210, 49, DIM);
  txt(g, "AGE", 352, 49, DIM, &fonts::Font0, Datum::top_right);
  g.drawFastHLine(8, 59, 344, (uint8_t)GRID);

  for (int i = 0; i < s.readingCount; i++) {
    const Reading& r = s.readings[i];
    int y = 62 + i * 14;
    txt(g, r.name, 8, y, TEXT, &fonts::Font2);
    if (r.lastOkMs == 0) snprintf(b, sizeof(b), "--");
    else snprintf(b, sizeof(b), "%.*f %s", r.decimals, r.value, r.unit);
    txt(g, b, 196, y, WHITE, &fonts::Font2, Datum::top_right);
    txt(g, r.status, 210, y, r.statusColor, &fonts::Font2);
    if (r.lastOkMs == 0) snprintf(b, sizeof(b), "--");
    else snprintf(b, sizeof(b), "%.1fs", (ms - r.lastOkMs) / 1000.0f);
    txt(g, b, 352, y, DIM, &fonts::Font2, Datum::top_right);
  }

  // check engine light + stored codes
  int y = 62 + s.readingCount * 14 + 4;
  int n = snprintf(b, sizeof(b), "MIL %s   CODES:", s.milOn ? "ON" : "off");
  if (!s.codesRead) snprintf(b + n, sizeof(b) - n, " not read yet");
  else if (s.codesListed == 0) snprintf(b + n, sizeof(b) - n, " none");
  else for (int c = 0; c < s.codesListed && n < (int)sizeof(b) - 7; c++) n += snprintf(b + n, sizeof(b) - n, " %s", s.codes[c]);
  txt(g, b, 8, y, s.milOn || s.codesListed ? ORANGE : TEXT, &fonts::Font2);
}

// ---- entry point -------------------------------------------------------------
// Screen 0 is the dual-dial FX screen, drawn by fx::drawDual in dash_fx.h.
constexpr int SCREEN_COUNT = 7;
static const char* SCREEN_NAMES[SCREEN_COUNT] = { "DUAL", "HUD", "GRID", "SCOPE", "TERM", "ARC", "DIAG" };

static void draw(Gfx& g, int screen, const Telemetry& s, uint32_t ms)
{
  g.fillScreen((uint8_t)BG);
  if (screen != 4) header(g, screen, SCREEN_COUNT, SCREEN_NAMES[screen], s, ms);
  switch (screen) {
  case 1: drawHud(g, s, ms); break;
  case 2: drawGrid(g, s, ms); break;
  case 3: drawScope(g, s, ms); break;
  case 4: drawTerm(g, s, ms); break;
  case 5: drawArc(g, s, ms); break;
  case 6: drawDiag(g, s, ms); break;
  }
}

} // namespace dash
