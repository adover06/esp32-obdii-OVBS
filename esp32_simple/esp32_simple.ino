// ============================================================================
//  SIMPLE CAR DASHBOARD  (ESP32 -> composite video)
//
//  A small, easy-to-read version of the dashboard with two screens:
//    Screen 1: two round gauges (RPM and speed) + gear + coolant
//    Screen 2: a list of values with bars
//
//  Wiring:
//    GPIO25  -> RCA center pin (video)
//    GND     -> RCA outer shell
//    Button  -> between GPIO27 and GND (or just use the BOOT button)
//
//  Board in the Arduino IDE: "ESP32-WROOM-DA Module"
//
//  HOW THE PROGRAM IS ORGANIZED (read top to bottom):
//    1. Settings        - pins and colors
//    2. Video setup     - tells the graphics library how to make a TV signal
//    3. Car data        - getRPM(), getSpeedMPH(), ... (FAKE for now)
//    4. Drawing helpers - drawGauge(), drawBarRow(), drawTitleBar()
//    5. The screens     - drawGaugeScreen(), drawInfoScreen()
//    6. Button          - press to switch screens
//    7. setup() / loop() - Arduino runs setup() once, then loop() forever
// ============================================================================

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ============================================================================
//  1. SETTINGS
// ============================================================================

const int VIDEO_PIN       = 25;   // the pin wired to the RCA center
const int BUTTON_PIN      = 27;   // optional external button (to GND)
const int BOOT_BUTTON_PIN = 0;    // the BOOT button already on the board

const int SCREEN_WIDTH  = 360;    // pixels
const int SCREEN_HEIGHT = 240;

// Colors are written like CSS hex colors: 0xRRGGBB.
// They MUST be declared as uint32_t. A bare number like 0x00FFFF would be
// read by the library as a different color format and come out wrong.
const uint32_t BLACK     = 0x000000;
const uint32_t WHITE     = 0xFFFFFF;
const uint32_t CYAN      = 0x00DBFF;
const uint32_t DARK_BLUE = 0x002455;   // "empty" part of gauges and bars
const uint32_t LABEL     = 0x6D92FF;   // light blue for small labels
const uint32_t RED       = 0xFF2424;
const uint32_t AMBER     = 0xFFB600;

const float MAX_RPM      = 7000;
const float REDLINE_RPM  = 6000;
const float MAX_SPEED    = 120;


// ============================================================================
//  2. VIDEO SETUP
//  You don't need to change this. It tells the LovyanGFX library:
//  "make an NTSC TV picture, 360x240 pixels, on GPIO25".
// ============================================================================

class TVScreen : public lgfx::LGFX_Device
{
  lgfx::Panel_CVBS panel;   // CVBS = composite video

public:
  TVScreen()
  {
    auto size = panel.config();
    size.memory_width  = SCREEN_WIDTH;
    size.memory_height = SCREEN_HEIGHT;
    size.panel_width   = SCREEN_WIDTH;
    size.panel_height  = SCREEN_HEIGHT;
    panel.config(size);

    auto signal = panel.config_detail();
    signal.signal_type  = signal.signal_type_t::NTSC;   // US TVs / head units
    signal.pin_dac      = VIDEO_PIN;
    signal.output_level = 128;    // raise to ~200 if the picture is dim
    panel.config_detail(signal);

    setPanel(&panel);
  }
};

TVScreen tv;                 // the real TV output
LGFX_Sprite canvas(&tv);     // an invisible picture we draw on first,
                             // then copy to the TV all at once (no flicker)

int currentScreen = 0;       // 0 = gauges, 1 = info list


// ============================================================================
//  3. CAR DATA
//
//  Every number on the dashboard comes from one of these functions.
//  Right now they return FAKE values that change over time so you can see
//  the screens move. Later, each one gets replaced with a real OBD-II
//  request to the Veepeak adapter. Nothing else in the program changes.
// ============================================================================

// Helper for the fake data: a number that slides 0 -> 1 -> 0 over and over.
// periodSeconds = how long one full up-and-down takes.
float upAndDown(float periodSeconds)
{
  float seconds = millis() / 1000.0;
  float position = fmod(seconds, periodSeconds) / periodSeconds;   // 0.0 to 1.0
  if (position < 0.5) return position * 2;        // going up
  else                return (1 - position) * 2;  // coming down
}

// Engine speed in revolutions per minute.
// TODO: real version = OBD request "010C"
float getRPM()
{
  return 800 + upAndDown(8) * 5500;        // idles at 800, revs to 6300
}

// Vehicle speed in miles per hour.
// TODO: real version = OBD request "010D" (it answers in km/h, * 0.621 for mph)
float getSpeedMPH()
{
  return upAndDown(20) * 75;               // 0 to 75 mph and back
}

// Engine coolant temperature in Fahrenheit.
// TODO: real version = OBD request "0105" (answers in Celsius)
float getCoolantTempF()
{
  float seconds = millis() / 1000.0;
  float warmUp = min(seconds / 60.0, 1.0);  // 0 -> 1 over the first minute
  return 120 + warmUp * 75;                 // cold 120 F, warm 195 F
}

// Throttle pedal position, 0-100 %.
// TODO: real version = OBD request "0111"
float getThrottlePercent()
{
  return upAndDown(8) * 90;                // follows the fake RPM
}

// Battery / charging voltage.
// TODO: real version = adapter command "ATRV"
float getBatteryVolts()
{
  return 14.2;
}

// Current gear. 0 means stopped (park / neutral).
// The car doesn't report this, so it's estimated from speed for now.
// TODO: better version = compare RPM to speed for each gear ratio
int getGear()
{
  float mph = getSpeedMPH();
  if (mph < 1)  return 0;
  if (mph < 15) return 1;
  if (mph < 30) return 2;
  if (mph < 45) return 3;
  return 4;
}


// ============================================================================
//  4. DRAWING HELPERS
//  Small reusable pieces. The screens below are built out of these.
//  Coordinates: (0,0) is the top-left corner, x goes right, y goes down.
// ============================================================================

// A round gauge: a thick arc that fills up as the value rises,
// with the number written in the middle.
void drawGauge(int centerX, int centerY, int radius,
               float value, float maxValue,
               const char* label, uint32_t fillColor)
{
  // Angles: 0 = pointing right, 90 = pointing down (clockwise).
  // Starting at 150 and sweeping 240 degrees leaves a gap at the bottom.
  const float startAngle = 150;
  const float sweep      = 240;

  float fraction = value / maxValue;             // how full, 0.0 to 1.0
  fraction = constrain(fraction, 0.0, 1.0);

  int thickness = 12;

  // empty track (the whole arc, dark)
  canvas.fillArc(centerX, centerY, radius, radius - thickness,
                 startAngle, startAngle + sweep, DARK_BLUE);

  // filled part (only as far as the value reaches)
  canvas.fillArc(centerX, centerY, radius, radius - thickness,
                 startAngle, startAngle + sweep * fraction, fillColor);

  // the number in the middle
  canvas.setFont(&fonts::Orbitron_Light_24);
  canvas.setTextDatum(textdatum_t::middle_center);   // x,y = center of the text
  canvas.setTextColor(WHITE);
  canvas.drawNumber((int)value, centerX, centerY);

  // the label under it
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(LABEL);
  canvas.drawString(label, centerX, centerY + 28);
}

// One row on the info screen:  LABEL   value   [#######-----]
void drawBarRow(int y, const char* label, float value, float maxValue,
                int decimals, const char* unit, uint32_t barColor)
{
  // label on the left
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setTextColor(LABEL);
  canvas.drawString(label, 16, y);

  // value, right-aligned so the digits line up
  char text[16];                                  // room for up to 15 characters
  snprintf(text, sizeof(text), "%.*f %s", decimals, value, unit);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.setTextColor(WHITE);
  canvas.drawString(text, 170, y);

  // the bar: dark background, then the filled part on top
  int barX = 185, barWidth = 160, barHeight = 12;
  float fraction = constrain(value / maxValue, 0.0, 1.0);
  canvas.fillRect(barX, y - barHeight / 2, barWidth, barHeight, DARK_BLUE);
  canvas.fillRect(barX, y - barHeight / 2, barWidth * fraction, barHeight, barColor);
}

// The strip across the top of every screen.
void drawTitleBar(const char* title)
{
  canvas.fillRect(0, 0, SCREEN_WIDTH, 18, DARK_BLUE);

  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(CYAN);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.drawString(title, 8, 9);

  char page[8];
  snprintf(page, sizeof(page), "%d/2", currentScreen + 1);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.drawString(page, SCREEN_WIDTH - 8, 9);
}


// ============================================================================
//  5. THE SCREENS
//  Each one reads the car data and draws a full picture on the canvas.
// ============================================================================

// Screen 1: two gauges, the gear in between, coolant at the bottom.
void drawGaugeScreen()
{
  float rpm     = getRPM();
  float mph     = getSpeedMPH();
  int   gear    = getGear();
  float coolant = getCoolantTempF();

  canvas.fillScreen(BLACK);
  drawTitleBar("FOCUS SE  -  GAUGES");

  // RPM gauge turns red past the redline
  uint32_t rpmColor = (rpm > REDLINE_RPM) ? RED : CYAN;
  drawGauge(90, 130, 78, rpm, MAX_RPM, "RPM", rpmColor);
  drawGauge(270, 130, 78, mph, MAX_SPEED, "MPH", CYAN);

  // gear in a circle between the gauges ("P" when stopped)
  canvas.drawCircle(180, 50, 20, CYAN);
  canvas.setFont(&fonts::Orbitron_Light_24);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setTextColor(gear == 0 ? AMBER : WHITE);
  if (gear == 0) canvas.drawString("P", 180, 51);
  else           canvas.drawNumber(gear, 180, 51);

  // coolant along the bottom
  char text[24];
  snprintf(text, sizeof(text), "COOLANT  %.0f F", coolant);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(coolant > 225 ? RED : LABEL);   // red if overheating
  canvas.drawString(text, 180, 222);
}

// Screen 2: every value as a labeled bar.
void drawInfoScreen()
{
  canvas.fillScreen(BLACK);
  drawTitleBar("FOCUS SE  -  INFO");

  //           y    label       value                 max   decimals unit   color
  drawBarRow(50,  "RPM",      getRPM(),             MAX_RPM,   0, "",    CYAN);
  drawBarRow(85,  "SPEED",    getSpeedMPH(),        MAX_SPEED, 0, "mph", CYAN);
  drawBarRow(120, "COOLANT",  getCoolantTempF(),    240,       0, "F",   AMBER);
  drawBarRow(155, "THROTTLE", getThrottlePercent(), 100,       0, "%",   CYAN);
  drawBarRow(190, "BATTERY",  getBatteryVolts(),    16,        1, "V",   CYAN);
}


// ============================================================================
//  6. BUTTON
//  Press BOOT (or the GPIO27 button) to go to the next screen.
// ============================================================================

bool buttonWasDown = false;
unsigned long lastPressTime = 0;

void checkButton()
{
  // The pins read LOW when the button is pressed (it connects them to GND).
  bool buttonIsDown = digitalRead(BUTTON_PIN) == LOW ||
                      digitalRead(BOOT_BUTTON_PIN) == LOW;

  // Only react at the moment it goes from "up" to "down", and ignore
  // presses closer than 200 ms apart (buttons "bounce" when pressed).
  bool justPressed = buttonIsDown && !buttonWasDown;
  if (justPressed && millis() - lastPressTime > 200) {
    currentScreen = (currentScreen + 1) % 2;   // 0 -> 1 -> 0 -> ...
    lastPressTime = millis();
  }
  buttonWasDown = buttonIsDown;
}


// ============================================================================
//  7. SETUP AND LOOP
// ============================================================================

// Runs once when the board powers on.
void setup()
{
  Serial.begin(115200);                    // for messages to the Serial Monitor

  // Buttons: INPUT_PULLUP means "HIGH normally, LOW when pressed to GND".
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  // Make the invisible canvas: 360 x 240 pixels, 1 byte (256 colors) each.
  // Done before starting the video so this big 86 KB piece of memory fits.
  canvas.setColorDepth(8);
  canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);

  // Start the TV signal.
  tv.setColorDepth(8);
  tv.init();

  Serial.println("Simple dashboard running. Press BOOT to switch screens.");
}

// Runs over and over, about 30 times a second.
void loop()
{
  checkButton();

  // Draw the current screen onto the invisible canvas...
  if (currentScreen == 0) drawGaugeScreen();
  else                    drawInfoScreen();

  // ...then copy the finished picture to the TV in one go.
  canvas.pushSprite(0, 0);

  delay(33);   // wait ~33 ms  ->  about 30 frames per second
}
