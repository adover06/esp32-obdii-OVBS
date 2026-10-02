// "FX" style screen: anti-aliased, gradient, glow look (the dual-dial screen).
//
// Draws with real RGB colors into the 8-bit (RGB332) target. Sizes come from
// dash::W, and every circle uses dash::rx() so it looks round on the TV even
// when pixels aren't square. Drawing is stateless; the needle smoothing lives
// in updateDual(), which runs once per frame (not once per strip).
// Include <LovyanGFX.hpp> and dash.h before this file.
#pragma once

namespace fx {

using Gfx = lgfx::LovyanGFX;
using Datum = lgfx::textdatum_t;
using dash::Telemetry;
using dash::CarData;
using dash::W;
using dash::H;

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
static uint32_t ramp(const Stop* s, size_t n, float f)
{
  if (f <= s[0].at) return s[0].color;
  for (size_t i = 1; i < n; i++) {
    if (f <= s[i].at) return mix(s[i - 1].color, s[i].color, (f - s[i - 1].at) / (s[i].at - s[i - 1].at));
  }
  return s[n - 1].color;
}

// gradients follow the tunable thresholds in dash.h (RPM feel)
constexpr float F_YELLOW = dash::YELLOW_RPM / dash::RPM_MAX;
constexpr float F_ORANGE = dash::ORANGE_RPM / dash::RPM_MAX;
constexpr float F_SHIFT  = dash::SHIFT_RPM  / dash::RPM_MAX;
static const Stop RPM_RAMP[] = {
  { 0.00f, CYAN }, { F_YELLOW, VIOLET }, { F_ORANGE, MAGENTA }, { F_SHIFT, RED }, { 1.00f, RED },
};
static const Stop MPH_RAMP[] = {
  { 0.00f, CYAN }, { 0.60f, 0x00B6FF }, { 1.00f, VIOLET },
};
static const Stop SHIFT_RAMP[] = {
  { 0.00f, GREEN }, { F_YELLOW, 0xB6FF00 }, { F_ORANGE, AMBER }, { F_SHIFT, RED }, { 1.00f, RED },
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
  const int n = dash::NARROW ? 24 : 32;
  const float pitch = (float)w / n;
  const int slant = dash::NARROW ? 3 : 4;
  int lit = (int)(f * n + 0.5f);
  for (int i = 0; i < n; i++) {
    float pos = (i + 0.5f) / n;
    uint32_t c = ramp(SHIFT_RAMP, 5, pos);
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

// 240-degree dial, open at the bottom. value drives a glowing gradient arc
// plus a bright cursor; the big number sits in the middle.
// R is the radius in vertical pixels; the x radius follows the pixel aspect.
static void dial(Gfx& g, int cx, int cy, float R, float value, const DialStyle& st,
                 const char* bigText, const char* unit, bool flash, const lgfx::IFont* bigFont)
{
  using dash::ringArc;
  using dash::ex;
  using dash::ey;
  const float A0 = 150, SPAN = 240;
  const float rOut = R, rArc1 = R - 5, rArc0 = R - 15;
  float f = value / st.maxV;
  if (f < 0) f = 0;
  if (f > 1) f = 1;

  // outer hairline ring + track
  ringArc(g, cx, cy, rOut, rOut - 1, A0, A0 + SPAN, RING);
  ringArc(g, cx, cy, rArc1, rArc0, A0, A0 + SPAN, TRACK);

  // redline zone marked on the outer ring
  if (st.redFrom > 0) ringArc(g, cx, cy, rOut + 1, rOut - 2, A0 + SPAN * st.redFrom / st.maxV, A0 + SPAN, RED);

  // value arc: soft halo first, then the bright core, drawn in small slices
  // so the color follows the gradient
  float end = A0 + SPAN * f;
  for (float a = A0; a < end; a += 2.0f) {
    float a1 = a + 2.2f < end ? a + 2.2f : end;
    uint32_t c = ramp(st.ramp, st.rampN, (a - A0) / SPAN);
    if (flash) c = RED;
    ringArc(g, cx, cy, rArc1 + 2, rArc0 - 2, a, a1, dim(c, 0.35f));
    ringArc(g, cx, cy, rArc1, rArc0, a, a1, c);
  }

  // ticks: majors are notches cut through the arc, minors are short marks
  // just inside it; labels sit right inside the arc to keep the center clear
  for (int v = 0; v <= (int)st.maxV; v += st.majorEvery / 2) {
    float a = A0 + SPAN * v / st.maxV;
    bool major = v % st.majorEvery == 0;
    bool red = st.redFrom > 0 && v >= st.redFrom;
    if (major) {
      dash::wedge(g, ex(cx, rArc0 - 1, a), ey(cy, rArc0 - 1, a), ex(cx, rArc1 + 1, a), ey(cy, rArc1 + 1, a), 0.9f, 0.9f, BG);
      // on a narrow screen the number fills the middle: skip labels level with it
      if (dash::NARROW && fabsf(sinf(a * dash::DEG2RAD)) < 0.3f) continue;
      char b[6];
      snprintf(b, sizeof(b), "%d", (int)(v / st.labelDiv));
      float rl = rArc0 - 9;
      txt(g, b, (int)ex(cx, rl, a), (int)ey(cy, rl, a), red ? RED : LABEL, &fonts::Font0, Datum::middle_center);
    } else {
      dash::wedge(g, ex(cx, rArc0 - 4, a), ey(cy, rArc0 - 4, a), ex(cx, rArc0 - 2, a), ey(cy, rArc0 - 2, a), 0.6f, 0.6f, red ? RED : RING);
    }
  }

  // cursor at the tip of the arc
  dash::wedge(g, ex(cx, rArc0 - 1, end), ey(cy, rArc0 - 1, end), ex(cx, rOut + 2, end), ey(cy, rOut + 2, end), 1.8f, 1.2f, WHITE);

  // center readout on a black backing so the cursor/ticks never cut through it;
  // at redline it alternates red/white (red alone smears into the red arc)
  g.setFont(bigFont);
  g.setTextSize(1);
  int tw = g.textWidth(bigText) + 4, th = g.fontHeight();
  g.fillRect(cx - tw / 2, cy - 2 - th / 2, tw, th, BG);
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

// Label + value + thin gradient meter. On a narrow screen the value goes on
// its own line under the label.
static void infoTile(Gfx& g, int x, int y, int w, int h, const char* label, const char* value,
                     float f, uint32_t color, bool warn)
{
  chamferBox(g, x, y, w, h, warn ? RED : RING);
  txt(g, label, x + 6, y + 4, LABEL, &fonts::Font0);
  if (dash::NARROW) txt(g, value, x + w - 5, y + 11, warn ? RED : TEXT, &fonts::Font2, Datum::top_right);
  else txt(g, value, x + w - 6, y + 3, warn ? RED : TEXT, &fonts::Font2, Datum::top_right);
  int mw = w - 12;
  int lit = (int)(mw * (f < 0 ? 0 : f > 1 ? 1 : f));
  g.fillRect(x + 6, y + h - 6, mw, 2, TRACK);
  for (int i = 0; i < lit; i++) g.drawFastVLine(x + 6 + i, y + h - 6, 2, mix(dim(color, 0.5f), color, (float)i / mw));
}

// ---- gear badge: hexagon (round-looking on any pixel aspect) ------------------
static void gearBadge(Gfx& g, int cx, int cy, float r, int gear)
{
  int px[6], py[6];
  for (int i = 0; i < 6; i++) {
    float a = 60 * i + 30;
    px[i] = (int)dash::ex(cx, r, a);
    py[i] = (int)dash::ey(cy, r, a);
  }
  for (int i = 0; i < 6; i++) {
    int j = (i + 1) % 6;
    g.fillTriangle(cx, cy, px[i], py[i], px[j], py[j], 0x00122A);
  }
  for (int i = 0; i < 6; i++) {
    int j = (i + 1) % 6;
    dash::wedge(g, px[i], py[i], px[j], py[j], 1.0f, 1.0f, gear ? CYAN : AMBER);
  }
  char b[2] = { gear ? (char)('0' + gear) : 'N', 0 };
  txt(g, b, cx, cy + 1, gear ? WHITE : AMBER, &fonts::Orbitron_Light_24, Datum::middle_center);
}

// ---- the dual-dial screen ---------------------------------------------------------
struct DualState {
  float rpmShown = 0, mphShown = 0;
};

// Once per frame: needles glide toward the latest reading so slow OBD
// updates still look smooth.
static void updateDual(DualState& st, const Telemetry& s, float dt)
{
  float k = dt * 10;
  if (k > 1) k = 1;
  st.rpmShown += (s.d.rpm - st.rpmShown) * k;
  st.mphShown += (s.d.mph - st.mphShown) * k;
}

// Stateless drawing: safe to call once per strip.
static void drawDual(Gfx& g, const Telemetry& s, uint32_t ms, const DualState& st)
{
  const CarData& d = s.d;
  const int M = dash::margin();
  bool over = d.rpm >= dash::SHIFT_RPM;
  bool flash = over && (ms / 100) % 2 == 0;

  backdrop(g);
  shiftBar(g, M + 4, 8, W - 2 * M - 12, 12, st.rpmShown / dash::RPM_MAX, flash, over);

  // two dials side by side, as large as the width allows (radius in vertical px)
  const int gap = 6;
  float rX = (W - 2 * M - gap) / 4.0f;                 // horizontal radius that fits
  float R = rX * dash::PX_ASPECT;                       // matching vertical radius
  if (R > 81) R = 81;
  const int cy = 134;
  const int cxL = (int)(W / 2 - gap / 2 - dash::rx(R));
  const int cxR = (int)(W / 2 + gap / 2 + dash::rx(R));

  static const DialStyle rpmStyle = { RPM_RAMP, 5, dash::RPM_MAX, dash::SHIFT_RPM, 1000, 1000 };
  static const DialStyle mphStyle = { MPH_RAMP, 3, 120, -1, 20, 1 };
  char b[12];
  snprintf(b, sizeof(b), "%d", (int)(st.rpmShown / 10) * 10);
  dial(g, cxL, cy, R, st.rpmShown, rpmStyle, b, "RPM", flash, &fonts::Orbitron_Light_24);
  snprintf(b, sizeof(b), "%d", (int)(st.mphShown + 0.5f));
  dial(g, cxR, cy, R, st.mphShown, mphStyle, b, "MPH", false, &fonts::Orbitron_Light_32);

  gearBadge(g, W / 2, 50, 20, d.gear);

  // where the data comes from: LIVE / NO LINK / SIM
  uint32_t tagColor = s.sourceColor == dash::GREEN ? GREEN : s.sourceColor == dash::RED ? RED : AMBER;
  txt(g, s.sourceTag, M + 4, 26, tagColor, &fonts::Font0);

  // three info tiles along the bottom
  char v[3][12];
  snprintf(v[0], 12, "%d F", (int)d.coolantF);
  snprintf(v[1], 12, "%.1f V", d.volts);
  snprintf(v[2], 12, "%d %%", (int)d.throttle);
  const int th = dash::NARROW ? 32 : 26, tgap = dash::NARROW ? 4 : 7;
  const int tw = (W - 2 * M - 8 - 2 * tgap) / 3, ty = H - th - 6;
  int tx = M + 4;
  infoTile(g, tx, ty, tw, th, "COOLANT", v[0], dash::frac(d.coolantF, 100, 240), CYAN, d.coolantF > 225);
  infoTile(g, tx + tw + tgap, ty, tw, th, "BATTERY", v[1], dash::frac(d.volts, 11, 15), CYAN, d.volts < 12.5f);
  infoTile(g, tx + 2 * (tw + tgap), ty, tw, th, "THROTTLE", v[2], d.throttle / 100, MAGENTA, false);
}

} // namespace fx
