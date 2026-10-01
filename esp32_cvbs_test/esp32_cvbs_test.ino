// ESP32 composite video (CVBS) test pattern
// Step 1 of the car speed display project.
//
// Board:  original ESP32 (ESP32-WROOM-32 / DevKit V1). S2/S3/C3 will NOT work (no I2S-DAC).
// Wiring: GPIO25 -> RCA center pin (video)
//         GND    -> RCA outer shell
//
// Shows color bars, a border to check overscan, and a counting number
// to confirm the frame is updating live.

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ---- settings to tweak -------------------------------------------------
// Most US head units want NTSC. If black looks gray, try NTSC_J.
// If the picture rolls or is black-and-white on a European unit, try PAL.
#define VIDEO_SIGNAL  NTSC
#define VIDEO_PIN     25
#define VIDEO_WIDTH   360   // 720/2
#define VIDEO_HEIGHT  240   // 480/2 (use 288 for PAL)
#define OUTPUT_LEVEL  128   // raise (e.g. 180-220) if the picture is dim
// ------------------------------------------------------------------------

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_CVBS _panel;

public:
  LGFX(void)
  {
    {
      auto cfg = _panel.config();
      cfg.memory_width  = VIDEO_WIDTH;
      cfg.memory_height = VIDEO_HEIGHT;
      cfg.panel_width   = VIDEO_WIDTH;
      cfg.panel_height  = VIDEO_HEIGHT;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel.config(cfg);
    }
    {
      auto cfg = _panel.config_detail();
      cfg.signal_type  = cfg.signal_type_t::VIDEO_SIGNAL;
      cfg.pin_dac      = VIDEO_PIN;
      cfg.use_psram    = 0;
      cfg.output_level = OUTPUT_LEVEL;
      cfg.chroma_level = 128;
      _panel.config_detail(cfg);
    }
    setPanel(&_panel);
  }
};

LGFX tv;

void drawStaticPattern()
{
  const int w = tv.width();
  const int h = tv.height();

  tv.fillScreen(TFT_BLACK);

  // SMPTE-style color bars across the top half
  const uint32_t bars[] = {
    TFT_WHITE, TFT_YELLOW, TFT_CYAN, TFT_GREEN,
    TFT_MAGENTA, TFT_RED, TFT_BLUE, TFT_BLACK
  };
  const int barW = w / 8;
  for (int i = 0; i < 8; i++) {
    tv.fillRect(i * barW, 0, barW, h / 2, bars[i]);
  }

  // Grayscale ramp
  for (int x = 0; x < w; x++) {
    int v = x * 255 / w;
    tv.drawFastVLine(x, h / 2, 16, tv.color888(v, v, v));
  }

  // Border lines: if you can't see all four edges, the head unit is
  // cropping (overscan) and we'll need to pad the layout later.
  tv.drawRect(0, 0, w, h, TFT_WHITE);
  tv.drawRect(8, 8, w - 16, h - 16, TFT_RED);

  tv.setTextColor(TFT_WHITE, TFT_BLACK);
  tv.setTextDatum(textdatum_t::top_left);
  tv.setFont(&fonts::Font2);
  tv.drawString("ESP32 CVBS OK", 16, h / 2 + 24);
}

void setup()
{
  Serial.begin(115200);
  tv.setColorDepth(8);  // RGB332, ~86 KB framebuffer at 360x240
  tv.init();
  drawStaticPattern();
  Serial.printf("Composite output on GPIO%d, %dx%d\n", VIDEO_PIN, tv.width(), tv.height());
}

void loop()
{
  static uint32_t counter = 0;

  // Fake "speed" that counts 0-199 so we can see live updates
  tv.setFont(&fonts::Font7);  // 7-segment style digits
  tv.setTextDatum(textdatum_t::bottom_right);
  tv.setTextColor(TFT_GREEN, TFT_BLACK);
  tv.setTextPadding(tv.textWidth("888"));
  tv.drawNumber(counter % 200, tv.width() - 16, tv.height() - 16);

  counter++;
  delay(100);
}
