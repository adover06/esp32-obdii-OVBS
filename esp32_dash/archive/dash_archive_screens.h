// ARCHIVED screens: HUD (big RPM), GRID (12 values), SCOPE (history graphs).
//
// Not compiled: the Arduino IDE only builds files in the sketch folder itself
// (and src/), not this archive/ folder. Kept so they can be brought back.
//
// To restore one:
//   1. paste its draw function back into dash.h (inside namespace dash, after
//      the drawing helpers), and add its name/case to SCREEN_NAMES and draw()
//   2. SCOPE also needs the graph history: add
//        History rpmHist, mphHist, thrHist, loadHist;
//      to Telemetry, set mphHist/thrHist/loadHist.scale = 10 in its
//      constructor, and push d.rpm/d.mph/d.throttle/d.load in sample()
//      every 0.1 s (git history has the exact code, commit 6f63d67)
//   3. run dash_preview_app/validate.sh before uploading
//
// These were laid out for both 240 and 360 wide screens (see NARROW in dash.h).

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

