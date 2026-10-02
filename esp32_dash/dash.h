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

// ---- RPM feel (tune these) ------------------------------------------------------
// The engine's real redline is ~6500, but in normal hard driving the automatic
// shifts well before that, so with a true-to-life scale the bar barely left
// green. These make the gauges read "hotter": the dial ends at RPM_MAX, the
// color steps through yellow and orange sooner, and everything turns red and
// flashes from SHIFT_RPM up.
constexpr float RPM_MAX     = 6000;   // full scale of dials and shift bars
constexpr float YELLOW_RPM  = 3200;   // green -> yellow
constexpr float ORANGE_RPM  = 4000;   // yellow -> orange
constexpr float SHIFT_RPM   = 4600;   // red + flashing shift light
constexpr float IDLE_RPM    = 780;
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
  uint32_t okCount = 0, errCount = 0;
};

// Everything the screens draw from. Filled in by the Simulator (fake data)
// or by the OBD link (obd.h). The screens don't know which.
struct Telemetry {
  CarData d;
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

  // link details, for the SYS screen (filled by obd.h)
  char adapterInfo[24] = "";        // adapter's answer to ATI, e.g. "ELM327 v1.5"
  char protocolInfo[28] = "";       // answer to ATDP, e.g. "SAE J1850 PWM"
  uint32_t linkUpMs = 0;            // millis() when the car link came up, 0 = down
  uint32_t connects = 0, drops = 0; // successful links / Bluetooth drops since boot
  uint32_t okTotal = 0, errTotal = 0;

  // ESP32 health, for the SYS screen (filled by the sketch once a second)
  uint32_t freeHeap = 0, minFreeHeap = 0, maxBlock = 0, obdStackFree = 0;
  float frameMs = 0;
  int videoW = 0;

  // Call once per frame after d is updated: tracks peaks.
  void sample(float) {
    if (d.rpm > d.peakRpm) d.peakRpm = d.rpm;
    if (d.mph > d.peakMph) d.peakMph = d.mph;
  }
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
  return r < YELLOW_RPM ? GREEN : r < ORANGE_RPM ? YELLOW : r < SHIFT_RPM ? ORANGE : RED;
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
  const int kMax = (int)(RPM_MAX / 1000);
  for (int k = 0; k <= kMax; k++) {
    float a = a0 + span * k / kMax;
    snprintf(buf, sizeof(buf), "%d", k);
    txt(g, buf, (int)ex(cx, 74, a), (int)ey(cy, 74, a), k * 1000 >= SHIFT_RPM ? RED : DIM, &fonts::Font0, Datum::middle_center);
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
    "SCREENS .. 4",
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
  const int readR = NARROW ? 136 : 196, statX = NARROW ? 142 : 204, ageR = W - M;
  txt(g, "VALUE", M, 58, DIM);
  txt(g, "READING", readR, 58, DIM, &fonts::Font0, Datum::top_right);
  txt(g, "STATUS", statX, 58, DIM);
  txt(g, "AGE", ageR, 58, DIM, &fonts::Font0, Datum::top_right);
  if (!NARROW) txt(g, "OK%", 292, 58, DIM, &fonts::Font0, Datum::top_right);
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
      uint32_t tries = r.okCount + r.errCount;   // success rate of this value
      if (tries) {
        snprintf(b, sizeof(b), "%u", (unsigned)(r.okCount * 100 / tries));
        txt(g, b, 292, y, r.okCount * 10 >= tries * 9 ? TEXT : ORANGE, &fonts::Font2, Datum::top_right);
      }
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

// ---- screen 4: SYS ------------------------------------------------------------
// Everything about the link and the ESP32 that DIAG has no room for.
static void sysRow(Gfx& g, int y, const char* label, const char* value, uint8_t color = TEXT)
{
  txt(g, label, margin(), y, DIM);
  txt(g, value, NARROW ? 70 : 96, y, color);
}

static void sysSection(Gfx& g, int y, const char* title)
{
  txt(g, title, margin(), y, CYAN);
  g.drawFastHLine(margin() + (int)strlen(title) * 6 + 4, y + 3, W - 2 * margin() - (int)strlen(title) * 6 - 4, (uint8_t)GRID);
}

static void fmtDuration(char* out, size_t n, uint32_t ms)
{
  uint32_t s = ms / 1000;
  if (s >= 3600) snprintf(out, n, "%uh %02um", (unsigned)(s / 3600), (unsigned)(s / 60 % 60));
  else snprintf(out, n, "%um %02us", (unsigned)(s / 60), (unsigned)(s % 60));
}

static void drawSys(Gfx& g, const Telemetry& s, uint32_t ms)
{
  char b[64], t[24];
  int y = 18;

  sysSection(g, y, "LINK"); y += 11;
  if (s.readingCount == 0) {
    sysRow(g, y, "mode", "simulated data", YELLOW); y += 10;
  } else {
    sysRow(g, y, "adapter", s.adapterInfo[0] ? s.adapterInfo : "--"); y += 10;
    sysRow(g, y, "protocol", s.protocolInfo[0] ? s.protocolInfo : "--"); y += 10;
    if (s.linkUpMs) { fmtDuration(t, sizeof(t), ms > s.linkUpMs ? ms - s.linkUpMs : 0); snprintf(b, sizeof(b), "up %s", t); }
    else snprintf(b, sizeof(b), "DOWN");
    sysRow(g, y, "link", b, s.linkUpMs ? GREEN : RED); y += 10;
    snprintf(b, sizeof(b), "%u connects, %u drops", (unsigned)s.connects, (unsigned)s.drops);
    sysRow(g, y, "history", b, s.drops ? ORANGE : TEXT); y += 10;
    uint32_t tries = s.okTotal + s.errTotal;
    snprintf(b, sizeof(b), "%u ok, %u err (%u%%)", (unsigned)s.okTotal, (unsigned)s.errTotal,
             tries ? (unsigned)(s.okTotal * 100 / tries) : 0u);
    sysRow(g, y, "requests", b); y += 10;
    snprintf(b, sizeof(b), "%.1f replies/s", s.repliesPerSec);
    sysRow(g, y, "rate", b); y += 10;
  }

  y += 3; sysSection(g, y, "ESP32"); y += 11;
  snprintf(b, sizeof(b), "%uK  (lowest %uK)", (unsigned)(s.freeHeap / 1024), (unsigned)(s.minFreeHeap / 1024));
  sysRow(g, y, "free RAM", b, s.minFreeHeap && s.minFreeHeap < 16 * 1024 ? RED : TEXT); y += 10;
  snprintf(b, sizeof(b), "%uK", (unsigned)(s.maxBlock / 1024));
  sysRow(g, y, "largest", b); y += 10;
  if (s.frameMs > 0) snprintf(b, sizeof(b), "%.0f ms (%.0f fps)", s.frameMs, 1000.0f / (s.frameMs > 33 ? s.frameMs : 33));
  else snprintf(b, sizeof(b), "--");
  sysRow(g, y, "frame", b); y += 10;
  if (s.readingCount) {
    snprintf(b, sizeof(b), "%u bytes free", (unsigned)s.obdStackFree);
    sysRow(g, y, "OBD stack", b, s.obdStackFree && s.obdStackFree < 1024 ? RED : TEXT); y += 10;
  }
  fmtDuration(t, sizeof(t), ms);
  snprintf(b, sizeof(b), "%s   video %dx%d", t, s.videoW, H);
  sysRow(g, y, "uptime", b); y += 10;
  snprintf(b, sizeof(b), "%d rpm  %d mph", (int)s.d.peakRpm, (int)(s.d.peakMph + 0.5f));
  sysRow(g, y, "peaks", b); y += 10;

  y += 3; sysSection(g, y, "EVENTS"); y += 11;
  int room = (H - 4 - y) / 10;
  int shown = s.log.count < room ? s.log.count : room;
  for (int i = 0; i < shown; i++) {
    const char* line = s.log.lines[s.log.count - shown + i];
    txt(g, line, margin(), y, i == shown - 1 ? GREEN : PHOS_MID);
    y += 10;
  }
}

// ---- entry point -------------------------------------------------------------
// Screen 0 is the dual-dial FX screen, drawn by fx::drawDual in dash_fx.h.
// (HUD, GRID and SCOPE are archived in archive/dash_archive_screens.h.)
constexpr int SCREEN_COUNT = 4;
static const char* SCREEN_NAMES[SCREEN_COUNT] = { "DUAL", "ARC", "DIAG", "SYS" };

// Draws screens 1-3. Stateless: safe to call once per strip.
static void draw(Gfx& g, int screen, const Telemetry& s, uint32_t ms)
{
  g.fillScreen((uint8_t)BG);
  header(g, screen, SCREEN_COUNT, SCREEN_NAMES[screen], s, ms);
  switch (screen) {
  case 1: drawArc(g, s, ms); break;
  case 2: drawDiag(g, s, ms); break;
  case 3: drawSys(g, s, ms); break;
  }
}

} // namespace dash
