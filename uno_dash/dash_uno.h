// Uno dashboard: fake car data + 5 black & white screens for TVout.
//
// Needs a global `TVout tv` and <fontALL.h> before this file is included.
// The desktop preview includes it with a mock TVout, so keep it free of
// Arduino-only calls (no Serial, pinMode, etc).
//
// RAM is the tight resource here: the 120x96 screen alone takes 1440 of the
// Uno's 2048 bytes. All fixed text lives in flash (PSTR) and nothing redraws
// the whole screen per frame: each screen draws its frame once, then only
// overwrites the parts that change, which also keeps it flicker-free.
//
// TVout has no bounds checks on lines/text, so every coordinate below is
// kept inside 0..119 x 0..95 on purpose.
#pragma once

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace ud {

const uint8_t W = 120;
const uint8_t H = 96;

// ---- car constants (2000 Focus Zetec, 5-speed manual) --------------------
const float RPM_MAX = 7000;
const float SHIFT_RPM = 6000;
const float IDLE_RPM = 780;
// mph per 1000 rpm in each gear (index 0 = neutral)
const float MPH_PER_K[6] = { 0, 5.6f, 10.1f, 14.6f, 19.6f, 24.3f };

static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static float frac(float v, float lo, float hi) { return clampf((v - lo) / (hi - lo), 0, 1); }

// ---- event log (shown on the TERM screen) ---------------------------------
const uint8_t LOG_N = 2;
const uint8_t LOG_COLS = 30;   // 30 chars x 4 px = full width
char logLines[LOG_N][LOG_COLS + 1];
uint8_t logCount = 0;
bool logDirty = true;
float simClock = 0;

// fmtP must be a PSTR() format string
static void logEvent(const char* fmtP, ...)
{
  if (logCount == LOG_N) {
    memmove(logLines[0], logLines[1], sizeof(logLines[0]) * (LOG_N - 1));
    logCount--;
  }
  char* line = logLines[logCount];
  int n = snprintf_P(line, LOG_COLS + 1, PSTR("%03u "), (unsigned)simClock);
  va_list ap;
  va_start(ap, fmtP);
  vsnprintf_P(line + n, LOG_COLS + 1 - n, fmtP, ap);
  va_end(ap);
  // pad with spaces so the new line fully overwrites the old one
  for (uint8_t i = strlen(line); i < LOG_COLS; i++) line[i] = ' ';
  line[LOG_COLS] = 0;
  logCount++;
  logDirty = true;
}

// ---- fake driving ------------------------------------------------------------
// Loops: idle -> pull through the gears -> cruise -> brake -> idle.
// Alternates between a hard pull (shift near redline) and an easy one.
struct Car {
  float rpm = IDLE_RPM, mph = 0, thr = 0, load = 20, timing = 12;
  float clt = 150, iat = 88, volts = 14.1f, maf = 3;
  uint8_t gear = 0;          // 0 = neutral / clutch in
  uint16_t peak = 0;
};

class Sim {
public:
  Car d;

  void update(float dt) {
    if (dt > 0.1f) dt = 0.1f;
    simClock += dt;
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
        logEvent(hard ? PSTR("LAUNCH WOT") : PSTR("LAUNCH EASY"));
      }
      break;

    case PULL: {
      targetThr = shiftPause > 0 ? 0 : (hard ? 94 : 42);
      if (shiftPause > 0) shiftPause -= dt;
      static const float accel[6] = { 0, 8.5f, 6.0f, 4.2f, 3.0f, 2.3f };
      d.mph += (d.thr / 100.0f * accel[d.gear] * (hard ? 1.0f : 1.6f) - 0.0004f * d.mph * d.mph) * dt;
      float geared = d.mph * 1000 / MPH_PER_K[d.gear];
      if (d.gear == 1 && geared < 1400) geared = 1400 + d.thr * 8;  // clutch slip
      d.rpm = shiftPause > 0 ? approach(d.rpm, geared, dt * 10) : geared;
      if (d.rpm >= shiftAt && d.gear < 5) {
        logEvent(PSTR("SHIFT %u>%u AT %u%s"), d.gear, d.gear + 1, (unsigned)d.rpm,
                 d.rpm > SHIFT_RPM ? " REDLINE" : "");
        d.gear++;
        shiftPause = 0.35f;
      }
      // easy pulls top out near 61 mph in 5th, so cruise starts at 58
      if (d.mph >= 58) { setPhase(CRUISE); logEvent(PSTR("CRUISE 65 MPH")); }
      break;
    }

    case CRUISE:
      // short-shift up to 5th, then hold ~65 mph
      if (d.gear < 5 && phaseT > 0.5f * d.gear) d.gear++;
      targetThr = 18 + 4 * sin(simClock * 0.7f);
      d.mph = approach(d.mph, 65 + 2 * sin(simClock * 0.3f), dt * 0.6f);
      d.rpm = approach(d.rpm, d.mph * 1000 / MPH_PER_K[d.gear], dt * 6);
      if (phaseT > 8) { setPhase(DECEL); logEvent(PSTR("DECEL FUEL CUT")); }
      break;

    case DECEL:
      targetThr = 0;
      d.mph -= (3.5f + d.mph * 0.03f) * dt;
      if (d.gear > 1 && d.rpm < 1300) d.gear--;
      if (d.mph < 8) d.gear = 0;
      if (d.gear > 0) d.rpm = approach(d.rpm, d.mph * 1000 / MPH_PER_K[d.gear], dt * 8);
      else d.rpm = approach(d.rpm, IDLE_RPM, dt * 3);
      if (d.mph <= 0) {
        d.mph = 0;
        setPhase(IDLE);
        logEvent(PSTR("IDLE %u RPM"), (unsigned)IDLE_RPM);
      }
      break;
    }

    d.thr = approach(d.thr, targetThr, dt * 8);
    bool fuelCut = phase == DECEL && d.gear > 0;
    d.load = fuelCut ? 8 + 2 * noise(1.3f) : clampf(18 + d.thr * 0.78f + d.rpm / RPM_MAX * 6, 0, 100);
    d.timing = clampf(10 + d.rpm / 6500 * 22 - d.load * 0.1f + 0.8f * noise(5.7f), 0, 45);
    d.maf = 1.2f + d.rpm * d.load / 100 * 0.019f;

    float before = d.clt;
    d.clt = approach(d.clt, 196 + 3 * noise(0.05f), dt * 0.05f);
    if (before < 190 && d.clt >= 190) logEvent(PSTR("COOLANT AT TEMP %uF"), (unsigned)d.clt);
    d.iat = approach(d.iat, d.mph < 5 ? 104 : 86, dt * 0.04f);
    d.volts = (d.rpm < 1000 ? 13.9f : 14.3f) + 0.05f * noise(2.2f);
    if (d.rpm > d.peak) d.peak = (uint16_t)d.rpm;
  }

private:
  enum Phase : uint8_t { IDLE, PULL, CRUISE, DECEL };
  Phase phase = IDLE;
  float phaseT = 0, shiftPause = 0, shiftAt = 3000;
  bool hard = false;

  void setPhase(Phase p) { phase = p; phaseT = 0; }

  static float approach(float x, float target, float k) {
    if (k > 1) k = 1;
    return x + (target - x) * k;
  }

  // smooth pseudo-random wobble in [-1, 1]
  static float noise(float speed) {
    float t = simClock * speed;
    return 0.6f * sin(t * 1.7f + speed) + 0.4f * sin(t * 3.9f + speed * 2.3f);
  }
};

Sim sim;

// ---- text helpers ------------------------------------------------------------
const unsigned char* curFont = font4x6;

static void font(const unsigned char* f) { tv.select_font(f); curFont = f; }
static uint8_t fontW() { return pgm_read_byte(curFont); }

// TVout's 4x6 'R' has two stray pixels past its right edge on the bottom
// row. The next character covers them; at the end of a string, clear them.
static void fixTrailingR(uint8_t x, uint8_t y, char last)
{
  if (last == 'R' && curFont == font4x6 && x + 4 <= W) tv.draw_row(y + 4, x, x + 4, BLACK);
}

// text stored in flash: textP(x, y, PSTR("HELLO"))
static void textP(uint8_t x, uint8_t y, const char* p)
{
  char c, last = 0;
  while ((c = pgm_read_byte(p++))) { tv.print_char(x, y, c); x += fontW(); last = c; }
  fixTrailingR(x, y, last);
}

static void text(uint8_t x, uint8_t y, const char* s)
{
  char last = 0;
  while (*s) { last = *s; tv.print_char(x, y, *s++); x += fontW(); }
  fixTrailingR(x, y, last);
}

// Right-align s in a field `width` chars wide that ends at x (exclusive),
// padding with spaces so shorter values erase longer old ones.
static void field(uint8_t x, uint8_t y, const char* s, uint8_t width)
{
  char buf[8];
  uint8_t len = strlen(s);
  if (len > width) len = width;
  memset(buf, ' ', width - len);
  memcpy(buf + width - len, s, len);
  buf[width] = 0;
  text(x - width * fontW(), y, buf);
}

static void fieldInt(uint8_t x, uint8_t y, long v, uint8_t width)
{
  char b[8];
  ltoa(v, b, 10);
  field(x, y, b, width);
}

// one decimal place, AVR printf has no %f
static void fieldDec1(uint8_t x, uint8_t y, float v, uint8_t width)
{
  char b[8];
  long t = lround(v * 10);
  bool neg = t < 0;
  if (neg) t = -t;
  snprintf_P(b, sizeof(b), PSTR("%s%ld.%ld"), neg ? "-" : "", t / 10, t % 10);
  field(x, y, b, width);
}

// ---- shape helpers -----------------------------------------------------------
// fill [x, x+w) x [y, y+h)
static void fillBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t c)
{
  for (uint8_t r = y; r < y + h; r++) tv.draw_row(r, x, x + w, c);
}

// progress bar interior; draw its outline once in the static pass
static void bar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, float f)
{
  uint8_t lit = (uint8_t)(clampf(f, 0, 1) * w + 0.5f);
  if (lit) fillBox(x, y, lit, h, WHITE);
  if (lit < w) fillBox(x + lit, y, w - lit, h, BLACK);
}

// outline that sits 1 px around a bar() of the same x/y/w/h
static void barFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  tv.draw_rect(x - 1, y - 1, w + 1, h + 1, WHITE);
}

// segmented meter: lit segments are solid, unlit ones leave a base line
static void segBar(uint8_t x, uint8_t y, uint8_t n, uint8_t h, float f, bool allOn = false)
{
  uint8_t lit = allOn ? n : (uint8_t)(clampf(f, 0, 1) * n + 0.5f);
  for (uint8_t i = 0; i < n; i++) {
    uint8_t sx = x + i * 4;
    fillBox(sx, y, 3, h - 1, i < lit ? WHITE : BLACK);
    tv.draw_row(y + h - 1, sx, sx + 3, WHITE);
  }
}

// 7-segment digit. d: 0-9, 10 = 'n' (neutral), -1 = blank.
// Off segments are drawn first so lit ones win at the shared corners,
// and nothing is cleared first, so there's no flicker.
const uint8_t SEGS[11] PROGMEM = {
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x54
};

static void digit(uint8_t x, uint8_t y, int8_t d, uint8_t w, uint8_t h, uint8_t t)
{
  uint8_t s = d < 0 ? 0 : pgm_read_byte(SEGS + d);
  uint8_t mid = y + (h - t) / 2;
  uint8_t top = mid - y + t;        // height of the upper verticals
  uint8_t low = y + h - mid;        // height of the lower verticals
  for (uint8_t pass = 0; pass < 2; pass++) {
    uint8_t want = pass;            // pass 0 = off segments, pass 1 = on
    uint8_t c = pass ? WHITE : BLACK;
    if (((s >> 0) & 1) == want) fillBox(x, y, w, t, c);                 // a
    if (((s >> 1) & 1) == want) fillBox(x + w - t, y, t, top, c);       // b
    if (((s >> 2) & 1) == want) fillBox(x + w - t, mid, t, low, c);     // c
    if (((s >> 3) & 1) == want) fillBox(x, y + h - t, w, t, c);         // d
    if (((s >> 4) & 1) == want) fillBox(x, mid, t, low, c);             // e
    if (((s >> 5) & 1) == want) fillBox(x, y, t, top, c);               // f
    if (((s >> 6) & 1) == want) fillBox(x, mid, w, t, c);               // g
  }
}

// right-aligned number in 7-seg digits, leading zeros blanked
static void bigNumber(uint8_t x, uint8_t y, uint16_t v, uint8_t digits,
                      uint8_t w, uint8_t h, uint8_t t, uint8_t gap)
{
  for (int8_t i = digits - 1; i >= 0; i--) {
    int8_t dg = v % 10;
    bool blank = (v == 0 && i != digits - 1);
    digit(x + i * (w + gap), y, blank ? -1 : dg, w, h, t);
    v /= 10;
  }
}

static bool shiftFlash(uint32_t ms) { return sim.d.rpm >= SHIFT_RPM && (ms / 100) % 2 == 0; }

// ---- header -----------------------------------------------------------------
const uint8_t SCREEN_COUNT = 5;

static void header(uint8_t idx, const char* nameP)
{
  font(font4x6);
  textP(0, 0, PSTR("FOCUS//SE"));
  textP(44, 0, nameP);
  for (uint8_t i = 0; i < SCREEN_COUNT; i++) {
    uint8_t px = 98 + i * 4;
    if (i == idx) fillBox(px, 1, 3, 3, WHITE);
    else tv.draw_rect(px, 1, 2, 2, WHITE);
  }
  tv.draw_row(7, 0, W, WHITE);
}

// ---- screen 1: HUD ------------------------------------------------------------
static void hudStatic()
{
  header(0, PSTR("HUD"));
  font(font4x6);
  for (uint8_t k = 0; k <= 7; k++) tv.print_char(4 + 112 * k / 7 - (k == 7 ? 3 : 0), 19, '0' + k);
  tv.draw_rect(84, 26, 32, 30, WHITE);           // gear box
  textP(92, 58, PSTR("GEAR"));
  textP(4, 57, PSTR("RPM"));
  textP(36, 57, PSTR("PEAK"));
  static const char names[4][4] PROGMEM = { "SPD", "THR", "CLT", "BAT" };
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t x = i % 2 ? 62 : 2, y = i < 2 ? 66 : 82;
    textP(x, y, names[i]);
    barFrame(x, y + 9, 54, 4);
  }
}

static void hudDynamic(uint32_t ms)
{
  const Car& d = sim.d;
  bool over = d.rpm >= SHIFT_RPM;
  // shift light: fills with rpm, whole bar blinks past the shift point
  if (over) segBar(4, 10, 28, 8, 0, shiftFlash(ms));
  else segBar(4, 10, 28, 8, d.rpm / RPM_MAX);

  bigNumber(4, 28, (uint16_t)d.rpm / 10 * 10, 4, 14, 26, 3, 4);
  digit(92, 28, d.gear ? d.gear : 10, 16, 26, 4);

  font(font4x6);
  fieldInt(72, 57, d.peak, 4);

  font(font6x8);
  fieldInt(56, 66, lround(d.mph), 3);
  fieldInt(116, 66, lround(d.thr), 3);
  fieldInt(56, 82, lround(d.clt), 3);
  fieldDec1(116, 82, d.volts, 4);
  bar(2, 75, 54, 4, d.mph / 100);
  bar(62, 75, 54, 4, d.thr / 100);
  bar(2, 91, 54, 4, frac(d.clt, 100, 240));
  bar(62, 91, 54, 4, frac(d.volts, 11, 15));
}

// ---- screen 2: GRID -----------------------------------------------------------
static void gridStatic()
{
  header(1, PSTR("GRID"));
  tv.draw_column(39, 9, 95, WHITE);
  tv.draw_column(79, 9, 95, WHITE);
  tv.draw_row(37, 0, W, WHITE);
  tv.draw_row(65, 0, W, WHITE);
  static const char names[9][5] PROGMEM = {
    "RPM", "MPH", "GEAR", "THR", "LOAD", "TIMG", "CLT", "IAT", "BATT"
  };
  font(font4x6);
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t x = (i % 3) * 40, y = 10 + (i / 3) * 28;
    textP(x + 2, y, names[i]);
    barFrame(x + 2, y + 19, 35, 4);
  }
}

static void gridDynamic(uint32_t)
{
  const Car& d = sim.d;
  font(font6x8);
  // row 1
  fieldInt(38, 18, lround(d.rpm), 4);
  fieldInt(78, 18, lround(d.mph), 3);
  if (d.gear) fieldInt(118, 18, d.gear, 1); else field(118, 18, "N", 1);
  // row 2
  fieldInt(38, 46, lround(d.thr), 3);
  fieldInt(78, 46, lround(d.load), 3);
  fieldDec1(118, 46, d.timing, 4);
  // row 3
  fieldInt(38, 74, lround(d.clt), 3);
  fieldInt(78, 74, lround(d.iat), 3);
  fieldDec1(118, 74, d.volts, 4);

  float f[9] = {
    d.rpm / RPM_MAX, d.mph / 120, d.gear / 5.0f,
    d.thr / 100, d.load / 100, d.timing / 40,
    frac(d.clt, 100, 240), frac(d.iat, 40, 160), frac(d.volts, 11, 15)
  };
  for (uint8_t i = 0; i < 9; i++) bar((i % 3) * 40 + 2, 10 + (i / 3) * 28 + 19, 35, 4, f[i]);
}

// ---- screen 3: SCOPE ----------------------------------------------------------
// Two sweep traces, like an oscilloscope: the pen moves right and erases
// a few columns ahead of itself, so nothing has to scroll.
const uint8_t GX = 16;                 // graph area starts here
const uint8_t GW = W - GX;             // 104 columns = 10.4 s at 10 Hz
const uint8_t RPM_T = 10, RPM_B = 60;  // rpm trace rows (inclusive)
const uint8_t THR_T = 76, THR_B = 94;  // throttle trace rows (row 95 would make TVout touch 1 byte past the buffer)
uint8_t sweepX = 0, lastRpmY = RPM_B, lastThrY = THR_B;
uint32_t lastSample = 0;

static uint8_t rpmY(float rpm) { return RPM_B - (uint8_t)(frac(rpm, 0, RPM_MAX) * (RPM_B - RPM_T)); }
static uint8_t thrY(float thr) { return THR_B - (uint8_t)(frac(thr, 0, 100) * (THR_B - THR_T)); }

static void scopeGridColumn(uint8_t col)
{
  uint8_t x = GX + col;
  tv.draw_column(x, RPM_T, RPM_B, BLACK);
  tv.draw_column(x, THR_T, THR_B, BLACK);
  if (col % 2 == 0) {
    tv.set_pixel(x, rpmY(SHIFT_RPM), WHITE);              // dashed shift line
    tv.set_pixel(x, rpmY(RPM_MAX / 2), col % 8 == 0);     // dotted mid line
  }
  if (col % 26 == 0) {
    for (uint8_t y = RPM_T; y <= RPM_B; y += 4) tv.set_pixel(x, y, WHITE);
  }
  if (col % 8 == 0) tv.set_pixel(x, (THR_T + THR_B) / 2, WHITE);
}

static void scopeStatic()
{
  header(2, PSTR("SCOPE"));
  font(font4x6);
  textP(0, RPM_T, PSTR("7K"));
  textP(0, rpmY(SHIFT_RPM) - 2, PSTR("SH"));
  textP(0, RPM_B - 5, PSTR("0"));
  textP(0, THR_T, PSTR("TH"));
  textP(0, THR_B - 5, PSTR("0"));
  tv.draw_column(GX - 2, RPM_T, RPM_B, WHITE);
  tv.draw_column(GX - 2, THR_T, THR_B, WHITE);
  textP(26, 65, PSTR("RPM"));
  textP(76, 65, PSTR("MPH"));
  for (uint8_t c = 0; c < GW; c++) scopeGridColumn(c);
  sweepX = 0;
  lastRpmY = rpmY(sim.d.rpm);
  lastThrY = thrY(sim.d.thr);
}

static void scopeDynamic(uint32_t ms)
{
  const Car& d = sim.d;
  font(font6x8);
  fieldInt(24, 63, lround(d.rpm), 4);
  fieldInt(74, 63, lround(d.mph), 3);
  if (d.gear) { field(110, 63, "G", 1); fieldInt(118, 63, d.gear, 1); }
  else field(118, 63, " N", 2);

  if (ms - lastSample < 100) return;
  lastSample = ms;

  // clear a gap ahead of the pen
  for (uint8_t k = 1; k <= 3; k++) scopeGridColumn((sweepX + k) % GW);

  uint8_t ry = rpmY(d.rpm), ty = thrY(d.thr);
  uint8_t x = GX + sweepX;
  if (sweepX == 0) {
    tv.set_pixel(x, ry, WHITE);
    tv.set_pixel(x, ty, WHITE);
  } else {
    tv.draw_line(x - 1, lastRpmY, x, ry, WHITE);
    tv.draw_line(x - 1, lastThrY, x, ty, WHITE);
  }
  lastRpmY = ry;
  lastThrY = ty;
  sweepX = (sweepX + 1) % GW;
}

// ---- screen 4: TERM -------------------------------------------------------------
// Green-screen style text readout (30 x 16 chars in the 4x6 font).
uint8_t termGear = 255;

static void termStatic()
{
  font(font4x6);
  textP(0, 0, PSTR("FOCUS-SE OBD2 MON  J1850 PWM"));
  tv.draw_row(7, 0, W, WHITE);
  static const char rows[9][6] PROGMEM = {
    "RPM", "MPH", "GEAR", "THR", "LOAD", "TIMNG", "MAF", "CLT", "BATT"
  };
  static const char units[9][4] PROGMEM = { "", "", "", "PCT", "PCT", "DEG", "G/S", "F", "V" };
  for (uint8_t i = 0; i < 9; i++) {
    uint8_t y = 9 + i * 7;
    textP(0, y, rows[i]);
    if (i != 2) {
      tv.print_char(44, y, '[');
      tv.print_char(96, y, ']');
    }
    textP(104, y, units[i]);
  }
  tv.draw_row(73, 0, W, WHITE);
  textP(0, 89, PSTR("$"));
  termGear = 255;
  logDirty = true;
}

// 12 character cells between the brackets: solid block or a dot
static void termBar(uint8_t y, float f)
{
  uint8_t lit = (uint8_t)(clampf(f, 0, 1) * 12 + 0.5f);
  for (uint8_t i = 0; i < 12; i++) {
    uint8_t x = 48 + i * 4;
    fillBox(x, y, 3, 5, i < lit ? WHITE : BLACK);
    if (i >= lit) tv.set_pixel(x + 1, y + 4, WHITE);
  }
}

static void termDynamic(uint32_t ms)
{
  const Car& d = sim.d;
  font(font4x6);
  fieldInt(40, 9, lround(d.rpm), 5);       termBar(9, d.rpm / RPM_MAX);
  fieldInt(40, 16, lround(d.mph), 5);      termBar(16, d.mph / 120);
  fieldInt(40, 30, lround(d.thr), 5);      termBar(30, d.thr / 100);
  fieldInt(40, 37, lround(d.load), 5);     termBar(37, d.load / 100);
  fieldDec1(40, 44, d.timing, 5);          termBar(44, d.timing / 40);
  fieldDec1(40, 51, d.maf, 5);             termBar(51, d.maf / 120);
  fieldInt(40, 58, lround(d.clt), 5);      termBar(58, frac(d.clt, 100, 240));
  fieldDec1(40, 65, d.volts, 5);           termBar(65, frac(d.volts, 11, 15));

  // gear selector:  N 1 2 3 4 5 with the current one inverted
  if (d.gear != termGear) {
    termGear = d.gear;
    for (uint8_t k = 0; k <= 5; k++) {
      uint8_t x = 48 + k * 8;
      tv.print_char(x, 23, k ? '0' + k : 'N');
      if (k == d.gear) fillBox(x - 1, 22, 6, 7, INVERT);
      else fillBox(x - 1, 22, 6, 1, BLACK);
    }
  }

  if (logDirty) {
    logDirty = false;
    for (uint8_t i = 0; i < LOG_N; i++) {
      if (i < logCount) text(0, 75 + i * 7, logLines[i]);
    }
  }
  // blinking cursor
  fillBox(8, 89, 4, 6, (ms / 400) % 2 ? WHITE : BLACK);
}

// ---- screen 5: GAUGE ------------------------------------------------------------
// Needle tach with tick marks, side meters.
const uint8_t CX = 60, CY = 54, R = 38, NEEDLE = 19;
uint8_t needleX = CX, needleY = CY;

static float rpmAngle(float rpm) { return (135 + 270 * frac(rpm, 0, RPM_MAX)) * 0.0174533f; }

static void gaugeStatic()
{
  header(4, PSTR("GAUGE"));
  font(font4x6);
  // outer dotted arc, with a double dotted band through the redline
  for (uint8_t i = 0; i <= 70; i++) {
    float rpm = RPM_MAX * i / 70;
    float a = rpmAngle(rpm);
    tv.set_pixel(CX + R * cos(a) + 0.5f, CY + R * sin(a) + 0.5f, WHITE);
    if (rpm >= SHIFT_RPM) tv.set_pixel(CX + (R - 2) * cos(a) + 0.5f, CY + (R - 2) * sin(a) + 0.5f, WHITE);
  }
  // major ticks every 1000 rpm with labels, a dot every 500
  for (uint8_t k = 0; k <= 14; k++) {
    float a = rpmAngle(RPM_MAX * k / 14);
    float c = cos(a), s = sin(a);
    if (k % 2) {
      tv.set_pixel(CX + (R - 4) * c + 0.5f, CY + (R - 4) * s + 0.5f, WHITE);
    } else {
      tv.draw_line(CX + 30 * c + 0.5f, CY + 30 * s + 0.5f, CX + (R - 3) * c + 0.5f, CY + (R - 3) * s + 0.5f, WHITE);
      tv.print_char(CX + 24 * c - 1.5f, CY + 24 * s - 2.5f, '0' + k / 2);
    }
  }
  textP(54, 82, PSTR("RPM"));

  textP(2, 10, PSTR("THR"));
  barFrame(5, 18, 7, 70);
  textP(100, 10, PSTR("MPH"));
  textP(100, 32, PSTR("GEAR"));
  textP(100, 64, PSTR("CLT"));
  needleX = CX;
  needleY = CY;
}

static void gaugeDynamic(uint32_t ms)
{
  const Car& d = sim.d;
  // erase old needle (2 px wide), draw new one, then the hub on top
  tv.draw_line(CX, CY, needleX, needleY, BLACK);
  tv.draw_line(CX + 1, CY, needleX + 1, needleY, BLACK);
  float a = rpmAngle(d.rpm);
  needleX = CX + NEEDLE * cos(a);
  needleY = CY + NEEDLE * sin(a);
  tv.draw_line(CX, CY, needleX, needleY, WHITE);
  tv.draw_line(CX + 1, CY, needleX + 1, needleY, WHITE);
  tv.draw_circle(CX, CY, 3, WHITE, shiftFlash(ms) ? BLACK : WHITE);

  font(font6x8);
  fieldInt(72, 72, lround(d.rpm), 4);
  fieldInt(119, 18, lround(d.mph), 3);
  fieldInt(119, 71, lround(d.clt), 3);
  digit(104, 40, d.gear ? d.gear : 10, 11, 20, 3);

  // vertical throttle bar, fills from the bottom
  uint8_t lit = (uint8_t)(frac(d.thr, 0, 100) * 70 + 0.5f);
  if (lit < 70) fillBox(5, 18, 7, 70 - lit, BLACK);
  if (lit) fillBox(5, 18 + 70 - lit, 7, lit, WHITE);
  font(font4x6);
  fieldInt(14, 90, lround(d.thr), 3);
}

// ---- boot splash ----------------------------------------------------------------
static void bootFrame(uint8_t line)
{
  font(font4x6);
  switch (line) {
  case 0: textP(8, 12, PSTR("FOCUS//SE DASH OS")); break;
  case 1: textP(8, 26, PSTR("CVBS OUT ... NTSC 120X96")); break;
  case 2: textP(8, 34, PSTR("OBD-II ..... J1850 PWM")); break;
  case 3: textP(8, 42, PSTR("DATA ....... SIMULATOR")); break;
  case 4: textP(8, 50, PSTR("SCREENS .... 5")); break;
  case 5: textP(8, 64, PSTR("READY")); break;
  }
}

// ---- entry points ---------------------------------------------------------------
static void enterScreen(uint8_t screen)
{
  tv.clear_screen();
  switch (screen) {
  case 0: hudStatic(); break;
  case 1: gridStatic(); break;
  case 2: scopeStatic(); break;
  case 3: termStatic(); break;
  case 4: gaugeStatic(); break;
  }
}

static void drawScreen(uint8_t screen, uint32_t ms)
{
  switch (screen) {
  case 0: hudDynamic(ms); break;
  case 1: gridDynamic(ms); break;
  case 2: scopeDynamic(ms); break;
  case 3: termDynamic(ms); break;
  case 4: gaugeDynamic(ms); break;
  }
}

} // namespace ud
