// "FX" style screens: anti-aliased, gradient, glow look.
//
// Draws with real RGB colors, so the target must be an 8-bit (RGB332) or
// deeper canvas, not the 16-color palette sprite used by dash.h.
// Include <LovyanGFX.hpp> and dash.h (for Telemetry) before this file.
#pragma once

namespace fx {

using Gfx = lgfx::LovyanGFX;
using Datum = lgfx::textdatum_t;
using dash::Telemetry;
using dash::CarData;

constexpr int W = 360;
constexpr int H = 240;

// ---- colors ------------------------------------------------------------------
constexpr uint32_t BG      = 0x000000;
constexpr uint32_t DOT     = 0x002455;   // background grid dots
constexpr uint32_t TRACK   = 0x002455;   // unfilled arc track (darker values round to black in RGB332)
constexpr uint32_t RING    = 0x2449AA;   // thin outer rings
constexpr uint32_t LABEL   = 0x6D92FF;
constexpr uint32_t TEXT    = 0xDBEEFF;
constexpr uint32_t WHITE   = 0xFFFFFF;
constexpr uint32_t CYAN    = 0x00DBFF;
constexpr uint32_t VIOLET  = 0x6D49FF;
constexpr uint32_t MAGENTA = 0xFF00AA;
constexpr uint32_t RED     = 0xFF2424;
constexpr uint32_t AMBER   = 0xFFB600;
constexpr uint32_t GREEN   = 0x24FF6D;

static uint32_t mix(uint32_t a, uint32_t b, float t)
{
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  int r = ((a >> 16) & 255) + (((int)((b >> 16) & 255) - (int)((a >> 16) & 255)) * t);
  int g = ((a >> 8) & 255) + (((int)((b >> 8) & 255) - (int)((a >> 8) & 255)) * t);
  int bl = (a & 255) + (((int)(b & 255) - (int)(a & 255)) * t);
  return (uint32_t)(r << 16 | g << 8 | bl);
}

static uint32_t dim(uint32_t c, float k) { return mix(BG, c, k); }

struct Stop { float at; uint32_t color; };

// color along a list of gradient stops, f in [0,1]
template <size_t N>
static uint32_t ramp(const Stop (&stops)[N], float f)
{
  if (f <= stops[0].at) return stops[0].color;
  for (size_t i = 1; i < N; i++) {
    if (f <= stops[i].at) {
      float t = (f - stops[i - 1].at) / (stops[i].at - stops[i - 1].at);
      return mix(stops[i - 1].color, stops[i].color, t);
    }
  }
  return stops[N - 1].color;
}

static const Stop RPM_RAMP[] = {
  { 0.00f, CYAN }, { 0.55f, VIOLET }, { 0.78f, MAGENTA }, { 0.86f, RED }, { 1.00f, RED },
};
static const Stop MPH_RAMP[] = {
  { 0.00f, CYAN }, { 0.60f, 0x00B6FF }, { 1.00f, VIOLET },
};
static const Stop SHIFT_RAMP[] = {
  { 0.00f, GREEN }, { 0.60f, 0xB6FF00 }, { 0.75f, AMBER }, { 0.86f, RED }, { 1.00f, RED },
};

static void txt(Gfx& g, const char* s, int x, int y, uint32_t color,
                const lgfx::IFont* font, Datum datum = Datum::top_left, float size = 1)
{
  g.setFont(font);
  g.setTextSize(size);
  g.setTextDatum(datum);
  g.setTextColor(color);
  g.drawString(s, x, y);
}

static float rad(float deg) { return deg * 0.01745329f; }

// ---- background ------------------------------------------------------------
static void backdrop(Gfx& g)
{
  g.fillScreen(BG);
  for (int y = 6; y < H; y += 12) {
    for (int x = 6; x < W; x += 12) g.drawPixel(x, y, DOT);
  }
}

// ---- shift bar: slanted LED segments ------------------------------------------
// Every segment is always visible in a dim version of its color, lit ones glow.
static void shiftBar(Gfx& g, int x, int y, int w, int h, float f, bool flash, bool over)
{
  const int n = 32;
  const float pitch = (float)w / n;
  const int slant = 4;
  int lit = (int)(f * n + 0.5f);
  for (int i = 0; i < n; i++) {
    float pos = (i + 0.5f) / n;
    uint32_t c = ramp(SHIFT_RAMP, pos);
    bool on = i < lit;
    if (over) { on = true; c = flash ? RED : WHITE; }
    uint32_t fillc = on ? c : dim(c, 0.18f);
    int x0 = x + (int)(i * pitch);
    int x1 = x + (int)((i + 1) * pitch) - 2;
    // parallelogram leaning right
    g.fillTriangle(x0 + slant, y, x1 + slant, y, x0, y + h - 1, fillc);
    g.fillTriangle(x1 + slant, y, x1, y + h - 1, x0, y + h - 1, fillc);
    if (on) g.drawFastHLine(x0 + slant, y, x1 - x0, mix(c, WHITE, 0.6f));  // bright top edge
  }
}

// ---- dial ----------------------------------------------------------------------
struct DialStyle {
  const Stop* ramp; size_t rampN;
  float maxV;
  float redFrom;       // < 0 for none
  int majorEvery;      // label step in value units
  float labelDiv;      // label = value / labelDiv
};

static uint32_t rampN(const Stop* s, size_t n, float f)
{
  if (f <= s[0].at) return s[0].color;
  for (size_t i = 1; i < n; i++) {
    if (f <= s[i].at) return mix(s[i - 1].color, s[i].color, (f - s[i - 1].at) / (s[i].at - s[i - 1].at));
  }
  return s[n - 1].color;
}

// 240-degree dial, open at the bottom. value drives a glowing gradient arc
// plus a bright cursor; the big number sits in the middle.
static void dial(Gfx& g, int cx, int cy, int R, float value, const DialStyle& st,
                 const char* bigText, const char* unit, bool flash, const lgfx::IFont* bigFont)
{
  const float A0 = 150, SPAN = 240;
  const int rOut = R, rArc1 = R - 5, rArc0 = R - 15;
  float f = value / st.maxV;
  if (f < 0) f = 0;
  if (f > 1) f = 1;

  // outer hairline ring + track
  g.fillArc(cx, cy, rOut, rOut - 1, A0, A0 + SPAN, RING);
  g.fillArc(cx, cy, rArc1, rArc0, A0, A0 + SPAN, TRACK);

  // redline zone marked on the outer ring
  if (st.redFrom > 0) {
    g.fillArc(cx, cy, rOut + 1, rOut - 2, A0 + SPAN * st.redFrom / st.maxV, A0 + SPAN, RED);
  }

  // value arc: soft halo first, then the bright core, drawn in small slices
  // so the color follows the gradient
  float end = A0 + SPAN * f;
  for (float a = A0; a < end; a += 2.0f) {
    float a1 = a + 2.2f < end ? a + 2.2f : end;
    uint32_t c = rampN(st.ramp, st.rampN, (a - A0) / SPAN);
    if (flash) c = RED;
    g.fillArc(cx, cy, rArc1 + 2, rArc0 - 2, a, a1, dim(c, 0.35f));
    g.fillArc(cx, cy, rArc1, rArc0, a, a1, c);
  }

  // ticks: majors are notches cut through the arc, minors are short marks
  // just inside it; labels sit right inside the arc to keep the center clear
  for (int v = 0; v <= (int)st.maxV; v += st.majorEvery / 2) {
    float a = rad(A0 + SPAN * v / st.maxV);
    float c = cosf(a), s = sinf(a);
    bool major = v % st.majorEvery == 0;
    bool red = st.redFrom > 0 && v >= st.redFrom;
    if (major) {
      g.drawWedgeLine(cx + (rArc0 - 1) * c, cy + (rArc0 - 1) * s, cx + (rArc1 + 1) * c, cy + (rArc1 + 1) * s, 0.9f, 0.9f, BG);
      char b[6];
      snprintf(b, sizeof(b), "%d", (int)(v / st.labelDiv));
      float rl = rArc0 - 9;
      txt(g, b, cx + rl * c, cy + rl * s, red ? RED : LABEL, &fonts::Font0, Datum::middle_center);
    } else {
      float r0 = rArc0 - 4, r1 = rArc0 - 2;
      g.drawWedgeLine(cx + r0 * c, cy + r0 * s, cx + r1 * c, cy + r1 * s, 0.6f, 0.6f, red ? RED : RING);
    }
  }

  // cursor at the tip of the arc
  {
    float a = rad(end);
    float c = cosf(a), s = sinf(a);
    g.drawWedgeLine(cx + (rArc0 - 4) * c, cy + (rArc0 - 4) * s,
                    cx + (rOut + 2) * c, cy + (rOut + 2) * s, 1.8f, 1.2f, WHITE);
  }

  // center readout
  txt(g, bigText, cx, cy - 2, flash ? RED : WHITE, bigFont, Datum::middle_center);
  txt(g, unit, cx, cy + 22, LABEL, &fonts::Font0, Datum::middle_center);
}

// ---- small chamfered info tile ---------------------------------------------------
static void chamferBox(Gfx& g, int x, int y, int w, int h, uint32_t c)
{
  const int k = 5;
  g.drawLine(x + k, y, x + w - 1, y, c);
  g.drawLine(x + w - 1, y, x + w - 1, y + h - 1 - k, c);
  g.drawLine(x + w - 1, y + h - 1 - k, x + w - 1 - k, y + h - 1, c);
  g.drawLine(x + w - 1 - k, y + h - 1, x, y + h - 1, c);
  g.drawLine(x, y + h - 1, x, y + k, c);
  g.drawLine(x, y + k, x + k, y, c);
}

static void infoTile(Gfx& g, int x, int y, int w, const char* label, const char* value,
                     float f, uint32_t color, bool warn)
{
  const int h = 26;
  chamferBox(g, x, y, w, h, warn ? RED : RING);
  txt(g, label, x + 7, y + 4, LABEL, &fonts::Font0);
  txt(g, value, x + w - 6, y + 3, warn ? RED : TEXT, &fonts::Font2, Datum::top_right);
  // thin gradient meter along the bottom
  int mw = w - 14;
  int lit = (int)(mw * (f < 0 ? 0 : f > 1 ? 1 : f));
  g.fillRect(x + 7, y + h - 6, mw, 2, TRACK);
  for (int i = 0; i < lit; i++) g.drawFastVLine(x + 7 + i, y + h - 6, 2, mix(dim(color, 0.5f), color, (float)i / mw));
}

// ---- gear badge: hexagon ------------------------------------------------------
static void gearBadge(Gfx& g, int cx, int cy, int r, int gear)
{
  int px[6], py[6];
  for (int i = 0; i < 6; i++) {
    float a = rad(60 * i + 30);
    px[i] = cx + r * cosf(a);
    py[i] = cy + r * sinf(a);
  }
  for (int i = 0; i < 6; i++) {
    int j = (i + 1) % 6;
    g.fillTriangle(cx, cy, px[i], py[i], px[j], py[j], 0x00122A);
  }
  for (int i = 0; i < 6; i++) {
    int j = (i + 1) % 6;
    g.drawWideLine(px[i], py[i], px[j], py[j], 1.0f, gear ? CYAN : AMBER);
  }
  char b[2] = { gear ? (char)('0' + gear) : 'N', 0 };
  txt(g, b, cx, cy + 1, gear ? WHITE : AMBER, &fonts::Orbitron_Light_24, Datum::middle_center);
}

// ---- the dual-dial screen ---------------------------------------------------------
struct DualState {
  float rpmShown = 0, mphShown = 0;
};

// Needles glide toward the latest reading so slow OBD updates still look smooth.
static void drawDual(Gfx& g, const Telemetry& s, uint32_t ms, DualState& st, float dt)
{
  const CarData& d = s.d;
  float k = dt * 10;
  if (k > 1) k = 1;
  st.rpmShown += (d.rpm - st.rpmShown) * k;
  st.mphShown += (d.mph - st.mphShown) * k;

  bool over = d.rpm >= dash::SHIFT_RPM;
  bool flash = over && (ms / 100) % 2 == 0;

  backdrop(g);
  shiftBar(g, 14, 8, 332, 12, st.rpmShown / dash::RPM_MAX, flash, over);

  static const DialStyle rpmStyle = { RPM_RAMP, 5, dash::RPM_MAX, dash::SHIFT_RPM, 1000, 1000 };
  static const DialStyle mphStyle = { MPH_RAMP, 3, 120, -1, 20, 1 };
  char b[12];
  snprintf(b, sizeof(b), "%d", (int)(st.rpmShown / 10) * 10);
  dial(g, 90, 134, 81, st.rpmShown, rpmStyle, b, "RPM", flash, &fonts::Orbitron_Light_24);
  snprintf(b, sizeof(b), "%d", (int)(st.mphShown + 0.5f));
  dial(g, 270, 134, 81, st.mphShown, mphStyle, b, "MPH", false, &fonts::Orbitron_Light_32);

  gearBadge(g, 180, 52, 20, d.gear);

  // where the data comes from: LIVE / NO LINK / SIM
  uint32_t tagColor = s.sourceColor == dash::GREEN ? GREEN : s.sourceColor == dash::RED ? RED : AMBER;
  txt(g, s.sourceTag, 14, 26, tagColor, &fonts::Font0);

  char v[3][12];
  snprintf(v[0], 12, "%d F", (int)d.coolantF);
  snprintf(v[1], 12, "%.1f V", d.volts);
  snprintf(v[2], 12, "%d %%", (int)d.throttle);
  infoTile(g, 14, 208, 106, "COOLANT", v[0], dash::frac(d.coolantF, 100, 240), CYAN, d.coolantF > 225);
  infoTile(g, 127, 208, 106, "BATTERY", v[1], dash::frac(d.volts, 11, 15), CYAN, d.volts < 12.5f);
  infoTile(g, 240, 208, 106, "THROTTLE", v[2], d.throttle / 100, MAGENTA, false);
}

} // namespace fx
