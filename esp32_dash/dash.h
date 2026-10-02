// Dashboard screens + fake car data.
//
// Everything here draws into a LovyanGFX target with an 8-bit (RGB332) color
// format, so the exact same code runs on the ESP32 and in the desktop preview.
// Screens are laid out from the screen width W (240 or 360); the height is
// always 240. Drawing must be stateless (only reads Telemetry + time), because
// the strip renderer (render.h) calls it once per strip each frame.
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

// ---- screen geometry ---------------------------------------------------------
constexpr int H = 240;          // always 240 lines (one NTSC field)
constexpr int MAX_W = 360;
static int W = 360;             // 240 on a WROOM (RAM), 360 with PSRAM
static bool NARROW = false;     // W < 300: compact layouts
static float PX_ASPECT = 0.889f;  // displayed width / height of ONE pixel

// displayAspect: shape of the screen the picture fills (4:3 for a normal
// composite input; some head units stretch it to 5:3 or 16:9).
static void setScreen(int w, float displayAspect)
{
  W = w;
  NARROW = w < 300;
  PX_ASPECT = displayAspect * H / w;
}

static int margin() { return NARROW ? 5 : 8; }

// A circle that should LOOK round is an ellipse in pixels: x radius = y radius / pixel aspect.
static float rx(float ry) { return ry / PX_ASPECT; }

constexpr float DEG2RAD = 0.01745329f;
constexpr float RAD2DEG = 57.2957795f;

// ---- car constants (2000 Focus Zetec) -------------------------------------------
constexpr float RPM_MAX   = 7000;
constexpr float SHIFT_RPM = 6000;   // shift light starts flashing here
constexpr float IDLE_RPM  = 780;
// simulator only: mph per 1000 rpm in each gear (index 0 = neutral)
constexpr float GEAR_MPH_PER_K[6] = { 0, 5.6f, 10.1f, 14.6f, 19.6f, 24.3f };

// ---- colors ------------------------------------------------------------------
// The canvas is RGB332, where a uint8_t color IS the RGB332 value: RRRGGGBB.
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
  int gear = 0;           // 0 = stopped / neutral
  float peakRpm = 0, peakMph = 0;
};

// Ring buffer of samples for the graphs, stored as 16-bit to save RAM.
// scale: stored = value * scale (rpm: 1, percent/mph: 10).
struct History {
  static constexpr int N = MAX_W;
  uint16_t v[N] = {};
  int head = 0, count = 0;
  float scale = 1;
  void push(float x) {
    float s = x * scale;
    v[head] = s < 0 ? 0 : s > 65535 ? 65535 : (uint16_t)(s + 0.5f);
    head = (head + 1) % N;
    if (count < N) count++;
  }
  // i = 0 is the oldest sample still stored
  float at(int i) const { return v[(head - count + i + N * 2) % N] / scale; }
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
  char memLine[48] = "";
  static constexpr int MAX_READINGS = 14;
  Reading readings[MAX_READINGS];
  int readingCount = 0;             // 0 = no OBD (simulator)
  bool milOn = false;               // check engine light
  int codeCount = 0;
  char codes[8][6] = {};
  int codesListed = 0;
  bool codesRead = false;

  Telemetry() { mphHist.scale = thrHist.scale = loadHist.scale = 10; }

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

// Angles in the designs are as seen on the screen (0 = 3 o'clock, clockwise).
// With non-square pixels, LovyanGFX's ellipse-arc fill wants the angle in
// pixel space, so convert: same direction from the center, squashed by rx/ry.
static float pixAngle(float deg, float rxPix, float ryPix)
{
  float a = deg * DEG2RAD;
  float p = atan2f(ryPix * sinf(a), rxPix * cosf(a)) * RAD2DEG;
  return p < 0 ? p + 360 : p;
}

// Ring segment that looks circular on screen. rOut/rIn are in vertical pixels.
template <typename C>
static void ringArc(Gfx& g, int cx, int cy, float rOut, float rIn, float a0, float a1, C color)
{
  float rox = rx(rOut), rix = rx(rIn);
  float p0 = 0, p1 = 360;
  if (a1 - a0 < 359.9f) { p0 = pixAngle(a0, rox, rOut); p1 = pixAngle(a1, rox, rOut); }
  g.fillEllipseArc(cx, cy, (int)(rox + 0.5f), (int)(rix + 0.5f), (int)(rOut + 0.5f), (int)(rIn + 0.5f), p0, p1, color);
}

// Thick line from a to b, ar/br = half-width at each end (a tapered "wedge").
// Built from plain lines and filled triangles because LovyanGFX's
// drawWedgeLine/drawWideLine replace the clip rectangle with their own
// bounding box, which would write outside the strip renderer's buffer.
// NEVER use drawWedgeLine, drawWideLine or drawSmoothLine in screen code.
template <typename C>
static void wedge(Gfx& g, float ax, float ay, float bx, float by, float ar, float br, C color)
{
  float dx = bx - ax, dy = by - ay, len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) { g.drawPixel((int)lroundf(ax), (int)lroundf(ay), color); return; }
  float nx = -dy / len, ny = dx / len;             // unit normal
  if (ar < 1.2f && br < 1.2f) {
    // thin: one line, plus a neighbor line if it's meant to be ~2 px wide
    g.drawLine((int)lroundf(ax), (int)lroundf(ay), (int)lroundf(bx), (int)lroundf(by), color);
    if (ar >= 0.75f || br >= 0.75f)
      g.drawLine((int)lroundf(ax + nx), (int)lroundf(ay + ny), (int)lroundf(bx + nx), (int)lroundf(by + ny), color);
    return;
  }
  int x1 = (int)lroundf(ax + nx * ar), y1 = (int)lroundf(ay + ny * ar);
  int x2 = (int)lroundf(ax - nx * ar), y2 = (int)lroundf(ay - ny * ar);
  int x3 = (int)lroundf(bx - nx * br), y3 = (int)lroundf(by - ny * br);
  int x4 = (int)lroundf(bx + nx * br), y4 = (int)lroundf(by + ny * br);
  g.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  g.fillTriangle(x1, y1, x3, y3, x4, y4, color);
}

// point on a screen-round circle of radius r (vertical pixels) at angle deg
static float ex(int cx, float r, float deg) { return cx + rx(r) * cosf(deg * DEG2RAD); }
static float ey(int cy, float r, float deg) { return cy + r * sinf(deg * DEG2RAD); }

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
  if (seg < 1) seg = 1;
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
  if (seg < 1) seg = 1;
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
    g.fillRect(x + 6, y - 3, tw + 6, 8, (uint8_t)BG);
    txt(g, title, x + 9, y - 3, accent);
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

// Top strip: name, page dots, data source, uptime.
static void header(Gfx& g, int idx, int count, const char* name, const Telemetry& t, uint32_t ms)
{
  g.fillRect(0, 0, W, 13, (uint8_t)PANEL);
  g.drawFastHLine(0, 13, W, (uint8_t)GRID);
  g.fillRect(0, 0, 3, 13, (uint8_t)CYAN);

  int dotsX;
  if (NARROW) {
    txt(g, name, 7, 3, WHITE);                       // no room for the brand
    dotsX = 44;
  } else {
    txt(g, "FOCUS//SE", 8, 3, CYAN);
    txt(g, name, 70, 3, WHITE);
    dotsX = W / 2 - count * 5 + 18;
  }
  int pitch = NARROW ? 7 : 10;
  for (int i = 0; i < count; i++) {
    int px = dotsX + i * pitch;
    if (i == idx) g.fillRect(px, 4, 5, 5, (uint8_t)CYAN);
    else g.drawRect(px, 4, 5, 5, (uint8_t)DIM);
  }

  char buf[24];
  int m = (int)(t.clock / 60), s = (int)t.clock % 60;
  snprintf(buf, sizeof(buf), "%02d:%02d", m, s);
  txt(g, buf, W - 4, 3, TEXT, &fonts::Font0, Datum::top_right);
  int tagRight = W - 40;
  txt(g, t.sourceTag, tagRight, 3, t.sourceColor, &fonts::Font0, Datum::top_right);
  if ((ms / 500) % 2) g.fillRect(tagRight - g.textWidth(t.sourceTag) - 7, 5, 4, 4, t.sourceColor);
}

// ---- screen 2: HUD -----------------------------------------------------------
// Driver focus: shift light bar, huge RPM, gear.
static void drawHud(Gfx& g, const Telemetry& s, uint32_t ms)
{
  const CarData& d = s.d;
  const int M = margin(), inner = W - 2 * M;
  char buf[16];

  // shift light bar: fills by rpm, whole bar flashes past the shift point
  bool flash = shiftFlash(d, ms);
  if (d.rpm >= SHIFT_RPM) segBar(g, M, 19, inner, 16, 1.0f, flash ? zoneRed : zoneWhite, 6, 2);
  else segBar(g, M, 19, inner, 16, d.rpm / RPM_MAX, zoneRpm, 6, 2);
  for (int k = 0; k <= 7; k++) {
    snprintf(buf, sizeof(buf), "%d", k);
    int tx = M + inner * k / 7;
    txt(g, buf, tx, 38, k >= 6 ? RED : DIM, &fonts::Font0, k == 0 ? Datum::top_left : k == 7 ? Datum::top_right : Datum::top_center);
  }

  // big RPM with ghosted 7-segment digits behind it, gear box to the right
  const int gearW = NARROW ? 62 : 94;
  const int rpmW = inner - gearW - 6;
  const float digitScale = NARROW ? 1.0f : 1.2f;
  const int digitTop = NARROW ? 64 : 62;
  panel(g, M, 52, rpmW, 76, "ENGINE RPM", CYAN);
  txt(g, "8888", M + rpmW - 10, digitTop, GRID, &fonts::Font7, Datum::top_right, digitScale);
  snprintf(buf, sizeof(buf), "%d", (int)(d.rpm / 10) * 10);
  txt(g, buf, M + rpmW - 10, digitTop, d.rpm >= SHIFT_RPM ? (flash ? RED : WHITE) : WHITE, &fonts::Font7, Datum::top_right, digitScale);
  snprintf(buf, sizeof(buf), "PEAK %d", (int)d.peakRpm);
  txt(g, buf, M + 6, 116, DIM);

  int gx = M + rpmW + 6;
  panel(g, gx, 52, gearW, 76, "GEAR", CYAN);
  if (d.gear == 0) snprintf(buf, sizeof(buf), "N");
  else snprintf(buf, sizeof(buf), "%d", d.gear);
  txt(g, buf, gx + gearW / 2, 92, d.gear == 0 ? YELLOW : CYAN, &fonts::AsciiFont24x48, Datum::middle_center, NARROW ? 1.0f : 1.25f);

  // middle row: speed / load / timing
  const int gap = 6, w3 = (inner - 2 * gap) / 3;
  int x0 = M, x1 = M + w3 + gap, x2 = M + 2 * (w3 + gap);
  panel(g, x0, 136, w3, 44, "SPEED", TEAL);
  snprintf(buf, sizeof(buf), "%d", (int)(d.mph + 0.5f));
  txt(g, buf, x0 + w3 - (NARROW ? 6 : 32), 143, WHITE, &fonts::AsciiFont8x16, Datum::top_right, 2);
  if (!NARROW) txt(g, "MPH", x0 + w3 - 28, 165, DIM);

  panel(g, x1, 136, w3, 44, "LOAD", TEAL);
  snprintf(buf, sizeof(buf), "%d%%", (int)d.load);
  txt(g, buf, x1 + w3 - 6, 143, TEXT, &fonts::AsciiFont8x16, Datum::top_right);
  segBar(g, x1 + 6, 164, w3 - 12, 8, d.load / 100, zoneTeal);

  panel(g, x2, 136, w3, 44, "TIMING", TEAL);
  snprintf(buf, sizeof(buf), "%.1f", d.timing);
  txt(g, buf, x2 + w3 - (NARROW ? 6 : 46), 143, TEXT, &fonts::AsciiFont8x16, Datum::top_right);
  if (!NARROW) txt(g, "BTDC", x2 + w3 - 8, 147, DIM, &fonts::Font0, Datum::top_right);
  segBar(g, x2 + 6, 164, w3 - 12, 8, d.timing / 40, zoneTeal);

  // bottom row: four small stats
  struct Stat { const char* name; float v; const char* fmt; float lo, hi; uint8_t (*zone)(float); bool warn; };
  Stat stats[4] = {
    { "CLT",  d.coolantF, "%.0f",   100, 240, zoneHeat, d.coolantF > 225 },
    { "BATT", d.volts,    "%.1fV",  11,  15,  zoneCyan, d.volts < 12.5f },
    { "THR",  d.throttle, "%.0f%%", 0,   100, zoneMag,  false },
    { "MPG",  d.mpg,      "%.1f",   0,   50,  zoneCyan, false },
  };
  const int gap4 = NARROW ? 4 : 4, w4 = (inner - 3 * gap4) / 4;
  for (int i = 0; i < 4; i++) {
    int x = M + i * (w4 + gap4);
    panel(g, x, 188, w4, 44, stats[i].name, stats[i].warn ? RED : DIM);
    if (i == 3 && d.mph < 1) snprintf(buf, sizeof(buf), "--");
    else snprintf(buf, sizeof(buf), stats[i].fmt, stats[i].v);
    txt(g, buf, x + w4 - 5, 195, stats[i].warn ? RED : TEXT, &fonts::AsciiFont8x16, Datum::top_right);
    if (i == 0 && !NARROW) degUnit(g, x + 6, 199, "F", DIM);
    segBar(g, x + 5, 216, w4 - 10, 8, frac(stats[i].v, stats[i].lo, stats[i].hi), stats[i].zone);
  }
}

// ---- screen 3: GRID ----------------------------------------------------------
// Everything at once, btop style: 4x3 cells (3x4 on a narrow screen).
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
  const int M = margin(), cols = NARROW ? 3 : 4, rows = 12 / cols;
  const int gap = 4, top = 20, bottom = H - 4;
  const int cw = (W - 2 * M - (cols - 1) * gap) / cols;
  const int ch = (bottom - top - (rows - 1) * gap) / rows;
  char buf[16];
  for (int i = 0; i < 12; i++) {
    int x = M + (i % cols) * (cw + gap);
    int y = top + (i / cols) * (ch + gap);
    const Cell& c = cells[i];
    panel(g, x, y, cw, ch, c.name, i < 3 ? CYAN : DIM);

    if (i == 2 && d.gear == 0) snprintf(buf, sizeof(buf), "N");
    else if (i == 7 && d.mph < 1) snprintf(buf, sizeof(buf), "--");
    else snprintf(buf, sizeof(buf), c.fmt, c.v);
    int vy = y + (ch - 32) / 2 - 3;               // 32 px tall value, centered above the bar
    txt(g, buf, x + cw - 6, vy, c.color, &fonts::AsciiFont8x16, Datum::top_right, 2);
    int valueLeft = x + cw - 6 - g.textWidth(buf);
    g.setFont(&fonts::Font0);
    g.setTextSize(1);
    if (*c.unit && x + 6 + g.textWidth(c.unit) + 3 <= valueLeft)   // only if it fits beside the value
      txt(g, c.unit, x + 6, vy + 24, DIM);

    int by = y + ch - 12;
    if (i == 11) {
      // fuel trim: bar grows out from the center
      int cx = x + cw / 2, half = cw / 2 - 8;
      g.fillRect(x + 6, by, cw - 12, 8, (uint8_t)GRID);
      g.drawFastVLine(cx, by - 2, 12, (uint8_t)DIM);
      int len = (int)(frac(fabsf(c.v), 0, 10) * half);
      if (len < 2 && fabsf(c.v) >= 0.05f) len = 2;
      if (c.v >= 0) g.fillRect(cx + 2, by, len, 8, (uint8_t)CYAN);
      else g.fillRect(cx - 1 - len, by, len, 8, (uint8_t)ORANGE);
    } else {
      segBar(g, x + 6, by, cw - 12, 8, frac(c.v, c.lo, c.hi), c.zone);
    }
  }
}

// ---- screen 4: SCOPE ---------------------------------------------------------
// Oscilloscope-style history graphs.
static void drawScope(Gfx& g, const Telemetry& s, uint32_t)
{
  const CarData& d = s.d;
  const int M = margin(), inner = W - 2 * M;
  char buf[24];

  snprintf(buf, sizeof(buf), "RPM // %d SEC", (inner - 4) / 10);
  panel(g, M, 20, inner, 124, buf, CYAN);
  graph(g, M + 2, 26, inner - 4, 116, s.rpmHist, RPM_MAX, CYAN, PANEL, SHIFT_RPM);
  txt(g, "SHIFT", M + 4, 26 + 116 - (int)(SHIFT_RPM / RPM_MAX * 115) - 10, RED);
  snprintf(buf, sizeof(buf), "%d", (int)d.rpm);
  g.fillRect(W - M - 96, 27, 92, 20, (uint8_t)BG);
  txt(g, buf, W - M - 8, 30, d.rpm >= SHIFT_RPM ? RED : WHITE, &fonts::AsciiFont8x16, Datum::top_right);
  snprintf(buf, sizeof(buf), "G%d", d.gear);
  txt(g, d.gear ? buf : "N", W - M - 84, 30, YELLOW, &fonts::AsciiFont8x16);

  const int gap = 4, half = (inner - gap) / 2;
  int xa = M, xb = M + half + gap;
  panel(g, xa, 152, half, 82, NARROW ? "THROTTLE" : "THROTTLE %", MAGENTA);
  graph(g, xa + 2, 158, half - 4, 74, s.thrHist, 100, MAGENTA, PURPLE);
  snprintf(buf, sizeof(buf), "%d", (int)d.throttle);
  txt(g, buf, xa + half - 6, 160, WHITE, &fonts::AsciiFont8x16, Datum::top_right);

  panel(g, xb, 152, half, 82, NARROW ? "MPH" : "SPEED MPH", GREEN);
  graph(g, xb + 2, 158, half - 4, 74, s.mphHist, 100, GREEN, PHOS_DK);
  snprintf(buf, sizeof(buf), "%d", (int)(d.mph + 0.5f));
  txt(g, buf, xb + half - 6, 160, WHITE, &fonts::AsciiFont8x16, Datum::top_right);
}

// ---- screen 5: TERM ----------------------------------------------------------
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
  txt(g, NARROW ? "OBD-II MON  J1850" : "FOCUS-SE OBD-II MON v0.1   J1850-PWM", 4, 3, BG);
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

  // columns: name | value | unit | bar (narrow screens use a small unit font
  // and a shorter bar)
  const int nameX = NARROW ? 12 : 16, valR = NARROW ? 124 : 144;
  const int unitX = NARROW ? 127 : 150, barX = NARROW ? 150 : 184;
  const int cells = NARROW ? 9 : 18;
  char bar[32];
  for (int i = 0; i < 9; i++) {
    int y = 17 + i * 16;
    if (i % 2) g.fillRect(0, y, W, 16, (uint8_t)PHOS_DK);
    uint8_t vc = rows[i].warn ? YELLOW : GREEN;
    txt(g, ">", NARROW ? 2 : 4, y, PHOS_MID, &fonts::AsciiFont8x16);
    const char* name = rows[i].name;
    if (NARROW && i == 8) name = "BATT";            // leave a gap before the value
    txt(g, name, nameX, y, PHOS_MID, &fonts::AsciiFont8x16);
    txt(g, rows[i].val, valR, y, vc, &fonts::AsciiFont8x16, Datum::top_right);
    if (NARROW) txt(g, rows[i].unit, unitX, y + 5, GREEN);
    else txt(g, rows[i].unit, unitX, y, PHOS_MID, &fonts::AsciiFont8x16);
    if (i == 2) {
      // gear selector strip:  N 1 2 [3] 4 5
      int gx = barX + 4, step = NARROW ? 14 : 24;
      for (int k = 0; k <= 5; k++) {
        char c[2] = { k ? (char)('0' + k) : 'N', 0 };
        if (k == d.gear) {
          g.fillRect(gx - 3, y, 14, 16, (uint8_t)GREEN);
          txt(g, c, gx, y, BG, &fonts::AsciiFont8x16);
        } else {
          txt(g, c, gx, y, PHOS_MID, &fonts::AsciiFont8x16);
        }
        gx += step;
      }
    } else {
      termBar(bar, rows[i].f, cells);
      txt(g, bar, barX, y, vc, &fonts::AsciiFont8x16);
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

// ---- screen 6: ARC -----------------------------------------------------------
// Radial segmented tach with side meters.
static void drawArc(Gfx& g, const Telemetry& s, uint32_t ms)
{
  const CarData& d = s.d;
  const int M = margin();
  char buf[16];
  const int cx = W / 2, cy = 132;
  const float a0 = 135, span = 270;   // 0 deg = 3 o'clock, clockwise
  const int segs = 42;
  const float rOut = 100, rIn = 84;   // vertical pixels; x radius follows the pixel aspect
  bool flash = shiftFlash(d, ms);

  // painted redline band on the outside
  ringArc(g, cx, cy, rOut + 6, rOut + 4, a0 + span * SHIFT_RPM / RPM_MAX, a0 + span, (uint8_t)RED);

  int lit = (int)(frac(d.rpm, 0, RPM_MAX) * segs + 0.5f);
  for (int i = 0; i < segs; i++) {
    float sa = a0 + span * i / segs;
    float ea = sa + span / segs - 1.6f;
    uint8_t c = GRID;
    if (i < lit) c = d.rpm >= SHIFT_RPM ? (flash ? RED : WHITE) : zoneRpm((i + 0.5f) / segs);
    ringArc(g, cx, cy, rOut, rIn, sa, ea, c);
  }
  // 1000 rpm labels
  for (int k = 0; k <= 7; k++) {
    float a = a0 + span * k / 7;
    snprintf(buf, sizeof(buf), "%d", k);
    txt(g, buf, (int)ex(cx, 74, a), (int)ey(cy, 74, a), k >= 6 ? RED : DIM, &fonts::Font0, Datum::middle_center);
  }

  // center readout (a smaller font when the ring is narrow)
  snprintf(buf, sizeof(buf), "%d", (int)(d.rpm / 10) * 10);
  if (NARROW) txt(g, buf, cx, cy - 6, flash ? RED : WHITE, &fonts::Orbitron_Light_24, Datum::middle_center);
  else txt(g, buf, cx, cy - 6, flash ? RED : WHITE, &fonts::Font7, Datum::middle_center);
  txt(g, "RPM", cx, cy + (NARROW ? 18 : 26), DIM, &fonts::Font0, Datum::middle_center);   // below the big digits
  if (d.gear == 0) snprintf(buf, sizeof(buf), "N");
  else snprintf(buf, sizeof(buf), "%d", d.gear);
  txt(g, buf, cx, cy + 66, d.gear ? CYAN : YELLOW, &fonts::AsciiFont24x48, Datum::middle_center);

  // left: throttle + load vertical meters
  const int lw = NARROW ? 10 : 14, l1 = M + (NARROW ? 3 : 8), l2 = l1 + lw + (NARROW ? 8 : 13);
  txt(g, "THR", l1 + lw / 2, 24, DIM, &fonts::Font0, Datum::top_center);
  txt(g, NARROW ? "LD" : "LOAD", l2 + lw / 2, 24, DIM, &fonts::Font0, Datum::top_center);
  segVBar(g, l1, 36, lw, 170, d.throttle / 100, zoneMag);
  segVBar(g, l2, 36, lw, 170, d.load / 100, zoneTeal);
  snprintf(buf, sizeof(buf), "%d", (int)d.throttle);
  txt(g, buf, l1 + lw / 2, 212, TEXT, &fonts::Font0, Datum::top_center);
  snprintf(buf, sizeof(buf), "%d", (int)d.load);
  txt(g, buf, l2 + lw / 2, 212, TEXT, &fonts::Font0, Datum::top_center);

  // right: speed + small stats
  const int R = W - M, colW = NARROW ? 40 : 52;
  txt(g, "SPEED", R, 22, DIM, &fonts::Font0, Datum::top_right);
  snprintf(buf, sizeof(buf), "%d", (int)(d.mph + 0.5f));
  if (NARROW) txt(g, buf, R, 32, WHITE, &fonts::AsciiFont8x16, Datum::top_right);
  else txt(g, buf, R, 32, WHITE, &fonts::AsciiFont8x16, Datum::top_right, 2);
  txt(g, "MPH", R, NARROW ? 50 : 66, DIM, &fonts::Font0, Datum::top_right);

  struct Mini { const char* name; bool warn; char val[10]; };
  Mini minis[3] = {
    { NARROW ? "CLT" : "CLT F",   d.coolantF > 225, "" },
    { NARROW ? "BATT" : "BATT V", d.volts < 12.5f,  "" },
    { "MPG",                      false,            "" },
  };
  snprintf(minis[0].val, 10, "%d", (int)d.coolantF);
  snprintf(minis[1].val, 10, "%.1f", d.volts);
  if (d.mph < 1) snprintf(minis[2].val, 10, "--");
  else snprintf(minis[2].val, 10, "%.1f", d.mpg);
  for (int i = 0; i < 3; i++) {
    int y = 96 + i * 44;
    g.drawFastHLine(R - colW, y, colW, (uint8_t)GRID);
    txt(g, minis[i].name, R, y + 4, DIM, &fonts::Font0, Datum::top_right);
    txt(g, minis[i].val, R, y + 16, minis[i].warn ? RED : TEXT, &fonts::AsciiFont8x16, Datum::top_right);
  }
}

// ---- boot splash -------------------------------------------------------------
static void drawBoot(Gfx& g, uint32_t ms, const char* modeLine)
{
  g.fillScreen((uint8_t)BG);
  const char* lines[] = {
    "FOCUS//SE  DASH OS",
    "",
    modeLine,
    "OBD-II ... J1850 PWM",
    "SCREENS .. 7",
    "",
    "READY",
  };
  const int n = sizeof(lines) / sizeof(lines[0]);
  const int x = NARROW ? 12 : 24;
  int shown = (int)(ms / 180);
  if (shown > n) shown = n;
  for (int i = 0; i < shown; i++) {
    txt(g, lines[i], x, 50 + i * 20, i == 0 ? CYAN : i == n - 1 ? GREEN : TEXT, &fonts::AsciiFont8x16);
  }
  if ((ms / 300) % 2) g.fillRect(x, 50 + shown * 20, 8, 16, (uint8_t)CYAN);
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
  const int M = margin();
  char b[80];
  txt(g, s.linkLine, M, 16, s.linkColor, &fonts::Font2);
  txt(g, s.elmLine, M, 31, s.elmColor, &fonts::Font2);
  if (s.elmColor == GREEN) snprintf(b, sizeof(b), "%.1f replies/s  %s", s.repliesPerSec, s.memLine);
  else snprintf(b, sizeof(b), "%s", s.memLine);
  txt(g, b, M, 47, TEXT);

  // columns: name | reading | status | age
  const int readR = NARROW ? 136 : 196, statX = NARROW ? 142 : 210, ageR = W - M;
  txt(g, "VALUE", M, 58, DIM);
  txt(g, "READING", readR, 58, DIM, &fonts::Font0, Datum::top_right);
  txt(g, "STATUS", statX, 58, DIM);
  txt(g, "AGE", ageR, 58, DIM, &fonts::Font0, Datum::top_right);
  g.drawFastHLine(M, 67, W - 2 * M, (uint8_t)GRID);

  for (int i = 0; i < s.readingCount; i++) {
    const Reading& r = s.readings[i];
    int y = 69 + i * 14;
    txt(g, r.name, M, y, TEXT, &fonts::Font2);
    if (r.lastOkMs == 0) snprintf(b, sizeof(b), "--");
    else snprintf(b, sizeof(b), "%.*f %s", r.decimals, r.value, r.unit);
    txt(g, b, readR, y, WHITE, &fonts::Font2, Datum::top_right);
    if (r.lastOkMs == 0) snprintf(b, sizeof(b), "--");
    else snprintf(b, sizeof(b), "%.1fs", ms > r.lastOkMs ? (ms - r.lastOkMs) / 1000.0f : 0.0f);  // reading may be newer than this frame
    if (NARROW) {   // small font so status and age both fit
      txt(g, r.status, statX, y + 4, r.statusColor);
      txt(g, b, ageR, y + 4, TEXT, &fonts::Font0, Datum::top_right);
    } else {
      txt(g, r.status, statX, y, r.statusColor, &fonts::Font2);
      txt(g, b, ageR, y, DIM, &fonts::Font2, Datum::top_right);
    }
  }

  // check engine light + stored codes
  int y = 69 + s.readingCount * 14 + 3;
  int n = snprintf(b, sizeof(b), "MIL %s  CODES:", s.milOn ? "ON" : "off");
  if (!s.codesRead) snprintf(b + n, sizeof(b) - n, " not read yet");
  else if (s.codesListed == 0) snprintf(b + n, sizeof(b) - n, " none");
  else for (int c = 0; c < s.codesListed && n < (int)sizeof(b) - 7; c++) n += snprintf(b + n, sizeof(b) - n, " %s", s.codes[c]);
  txt(g, b, M, y, s.milOn || s.codesListed ? ORANGE : TEXT, &fonts::Font0);
}

// ---- entry point -------------------------------------------------------------
// Screen 0 is the dual-dial FX screen, drawn by fx::drawDual in dash_fx.h.
constexpr int SCREEN_COUNT = 7;
static const char* SCREEN_NAMES[SCREEN_COUNT] = { "DUAL", "HUD", "GRID", "SCOPE", "TERM", "ARC", "DIAG" };

// Draws screens 1-6. Stateless: safe to call once per strip.
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
