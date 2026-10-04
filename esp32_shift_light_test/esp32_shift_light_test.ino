// Shift-light test: fake RPM, no car needed.
//
// Drives the LM3914 10-LED bar and the RGB shift LED from a simulated drive:
// idle, then pulls through gears 1-4 (rev up, shift, RPM drops, repeat),
// cruise, and back to idle. Prints RPM and how many LEDs are lit.
//
// Wiring (see the wiring diagram, middle board):
//   GPIO26 -> LM3914 pin 5 (SIG), a plain wire
//   GPIO32 / GPIO33 / GPIO13 -> RGB red / green / blue
//   RGB long leg -> 270 ohm -> GND (or 3V3 if it's common anode, see below)
//   GPIO34 <- pot middle leg (optional; outer legs to 3V3 and GND)
//   GPIO35 <- HW-201 IR sensor OUT (VCC to 3V3, not 5 V; GND to GND)
//   ESP32 VIN (5 V) -> + rails, GND -> - rails
//
// How the bar works: the LM3914 lights LED k when its input is above
// k x 0.125 V (10 steps up to its 1.25 V reference). GPIO26 is one of the
// ESP32's two real analog outputs (DAC, 0-3.3 V in 256 steps of ~13 mV), so
// it can hand the LM3914 a steady voltage directly - no filter needed.
// We pick the voltage in the MIDDLE of each LED step, so small errors
// (supply, chip tolerance) never make an LED flicker.
//
// Serial monitor (115200): s = start/stop the simulation,
// 0-9 = hold a fixed RPM (0 = idle ... 9 = 6000), p = sweep slowly up and down,
// r = RGB test (red, green, blue, yellow for 1 s each),
// h = hand throttle on/off (off at start): hand over the HW-201 = gas pedal.
//
// RGB: one 270 ohm resistor on its shared (long) leg is fine; see setRgb.

#include "esp_timer.h"

// ---- settings ------------------------------------------------------------
#define BAR_PIN        26      // DAC2 output (GPIO25 and GPIO26 are the only DAC pins)
#define RGB_R_PIN      32
#define RGB_G_PIN      33
#define RGB_B_PIN      13
#define POT_PIN        34
#define HAND_PIN       35      // HW-201 IR sensor OUT (LOW = hand detected). Power it from 3V3!
#define USE_POT        false   // true once the pot is wired (else GPIO34 floats)
#define RGB_COMMON_ANODE false // true if the RGB long leg goes to 3V3 instead of GND

#define BAR_START_RPM  900     // bar is empty at or below this
#define SHIFT_RPM      3800    // all 10 LEDs lit here; RGB flashes at and above
#define WARN_PCT       90      // RGB turns solid blue at this % of SHIFT_RPM
#define IDLE_RPM       780
// ---------------------------------------------------------------------------

const float DAC_FULL = 3.3f;   // DAC output at value 255 (roughly; it's not precise)
const float STEP_V = 0.125f;   // LM3914 volts per LED (1.25 V reference / 10)

int shiftRpm = SHIFT_RPM;

// Light `n` LEDs (0..10): aim for the middle of LED n's step.
void setBar(int n)
{
  n = constrain(n, 0, 10);
  float volts = (n == 0) ? 0.0f : (n + 0.5f) * STEP_V;     // e.g. 3 LEDs -> 0.4375 V
  int value = (int)(volts / DAC_FULL * 255 + 0.5f);       // 0..255
  dacWrite(BAR_PIN, constrain(value, 0, 255));
}

// RPM -> number of LEDs: empty at BAR_START_RPM, full at shiftRpm.
int ledsFor(float rpm)
{
  if (rpm <= BAR_START_RPM) return 0;
  float frac = (rpm - BAR_START_RPM) / (float)(shiftRpm - BAR_START_RPM);
  return constrain((int)ceilf(frac * 10.0f), 0, 10);
}

// ---- RGB LED ------------------------------------------------------------------
// The LED has ONE resistor on its shared leg, so only one colour may be on at a
// time (two at once share that resistor and red takes nearly all the current).
// Mixed colours are made by switching between colours 1000 times a second,
// faster than the eye can see: YELLOW = red, green, red, green, ...
enum Colour { OFF, RED, GREEN, BLUE, YELLOW };
volatile Colour rgbColour = OFF;
void setRgb(Colour c);   // declared here so Arduino doesn't put it above Colour

void writeRgb(bool r, bool g, bool b)
{
  // common cathode: HIGH = on. Common anode: LOW = on.
  digitalWrite(RGB_R_PIN, r != RGB_COMMON_ANODE);
  digitalWrite(RGB_G_PIN, g != RGB_COMMON_ANODE);
  digitalWrite(RGB_B_PIN, b != RGB_COMMON_ANODE);
}

// runs every 1 ms on a timer (esp_timer), independent of loop()
void rgbTick(void*)
{
  static bool phase = false;
  phase = !phase;
  switch (rgbColour) {
    case RED:    writeRgb(true, false, false); break;
    case GREEN:  writeRgb(false, true, false); break;
    case BLUE:   writeRgb(false, false, true); break;
    case YELLOW: writeRgb(phase, !phase, false); break;   // alternate red / green
    default:     writeRgb(false, false, false); break;
  }
}

void setRgb(Colour c) { rgbColour = c; }

// off below WARN_PCT, solid blue approaching the shift point,
// strobing red / yellow at and above it
void updateRgb(float rpm, uint32_t ms)
{
  if (rpm >= shiftRpm) {
    setRgb(((ms / 70) % 2) ? RED : YELLOW);   // swap ~7 times a second
  } else if (rpm >= shiftRpm * WARN_PCT / 100) {
    setRgb(BLUE);
  } else {
    setRgb(OFF);
  }
}

// ---- fake RPM: a little drive cycle ------------------------------------------
// Like a Python generator: each call advances the "car" by dt seconds.
struct FakeCar {
  float rpm = IDLE_RPM;
  int gear = 0;            // 0 = idling
  float t = 0;             // seconds in the current phase
  int phase = 0;           // 0 idle, 1 accelerating, 2 shift dip, 3 cruise, 4 slow down

  void step(float dt)
  {
    t += dt;
    switch (phase) {
      case 0:   // idle with a little wobble
        rpm = IDLE_RPM + 25 * sinf(t * 7);
        if (t > 2.5f) { phase = 1; gear = 1; t = 0; }
        break;
      case 1: { // rev up; lower gears climb faster
        float rate = 2600.0f / gear;                 // rpm per second
        rpm += rate * dt;
        if (rpm >= shiftRpm + 150) {                 // overshoot the shift point a little
          if (gear < 4) { phase = 2; t = 0; }
          else { phase = 3; t = 0; }
        }
        break;
      }
      case 2:   // clutch in, shift: RPM falls to the next gear's level
        rpm -= 9000 * dt;
        if (rpm <= shiftRpm * 0.62f) { gear++; phase = 1; t = 0; }
        break;
      case 3:   // ease off to cruise
        rpm += (2400 - rpm) * 2.0f * dt;
        if (t > 3.0f) { phase = 4; t = 0; }
        break;
      case 4:   // coast down to idle
        rpm -= 900 * dt;
        if (rpm <= IDLE_RPM) { rpm = IDLE_RPM; gear = 0; phase = 0; t = 0; }
        break;
    }
  }
};

FakeCar car;

// ---- hand throttle: HW-201 IR sensor as a gas pedal -----------------------------
// The sensor only says "something is close" (yes/no), so it works like an on/off
// throttle in neutral: revs climb while your hand is there and fall when it isn't,
// with a rev limiter that bounces the RPM just past the shift point.
struct HandThrottle {
  float rpm = IDLE_RPM;
  int seen = 0;              // debounce: consecutive readings that agree
  bool hand = false;
  uint32_t cutUntil = 0;     // rev limiter: fuel cut until this time (ms)

  void step(float dt, bool rawHand)
  {
    // the sensor's output can chatter at the edge of its range: require 3 equal
    // readings in a row (30 ms) before believing a change
    if (rawHand == hand) seen = 0;
    else if (++seen >= 3) { hand = rawHand; seen = 0; }

    // rev limiter, like a real one: at the limit, cut fuel for 60 ms (revs sag a
    // little), then fuel again. Holds the RPM in a tight band ABOVE the shift
    // point, so the shift light stays on steadily while your hand is there.
    uint32_t now = millis();
    float limit = shiftRpm + 300;
    if (rpm >= limit) cutUntil = now + 60;
    bool cut = now < cutUntil;

    if (hand && !cut) rpm += (3200 - rpm * 0.25f) * dt;          // climbs, a bit slower near the top
    else if (hand)    rpm -= 2500 * dt;                           // limiter cut: small sag
    else              rpm -= (rpm - IDLE_RPM) * 2.2f * dt + 300 * dt;   // off the gas: back to idle
    if (rpm < IDLE_RPM) rpm = IDLE_RPM + 25 * sinf(now / 140.0f);  // idle wobble
  }
};
HandThrottle throttle;
bool handMode = false;     // h toggles; only turn on with the HW-201 connected (GPIO35 floats without it)
bool running = true;
int holdRpm = -1;          // >= 0: hold this RPM instead of simulating
bool sweep = false;

void setup()
{
  Serial.begin(115200);
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  pinMode(HAND_PIN, INPUT);   // the HW-201 has its own pull-up resistor
  writeRgb(false, false, false);
  setBar(0);
  // 1 ms timer for the RGB colour switching
  esp_timer_create_args_t targs = {};
  targs.callback = rgbTick;
  targs.name = "rgb";
  esp_timer_handle_t timer;
  esp_timer_create(&targs, &timer);
  esp_timer_start_periodic(timer, 1000);

  // power-on self test: fill the bar one LED at a time, then R, G, B
  Serial.println("\n=== shift light test === (s = start/stop, 0-9 = fixed RPM, p = slow sweep, r = RGB test, h = hand throttle)");
  for (int n = 0; n <= 10; n++) { setBar(n); delay(120); }
  setRgb(RED);    delay(400);
  setRgb(GREEN);  delay(400);
  setRgb(BLUE);   delay(400);
  setRgb(YELLOW); delay(400);
  setRgb(OFF);
  setBar(0);
  delay(300);
}

void loop()
{
  static uint32_t last = millis(), printAt = 0;
  uint32_t now = millis();
  float dt = (now - last) / 1000.0f;
  last = now;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') { running = !running; holdRpm = -1; sweep = false; Serial.println(running ? "simulation on" : "paused"); }
    else if (c >= '0' && c <= '9') { holdRpm = (c == '0') ? IDLE_RPM : (c - '0') * 6000 / 9; sweep = false; Serial.printf("holding %d rpm\n", holdRpm); }
    else if (c == 'p') { sweep = !sweep; holdRpm = -1; Serial.println(sweep ? "slow sweep" : "sweep off"); }
    else if (c == 'h') { handMode = !handMode; holdRpm = -1; sweep = false; Serial.println(handMode ? "hand throttle ON" : "hand throttle off (simulation)"); }
    else if (c == 'r') {   // RGB test: each colour for 1 s
      const char* names[] = { "red", "green", "blue", "yellow" };
      Colour cs[] = { RED, GREEN, BLUE, YELLOW };
      for (int i = 0; i < 4; i++) { Serial.printf("RGB %s\n", names[i]); setRgb(cs[i]); delay(1000); }
      setRgb(OFF);
    }
  }

#if USE_POT
  // pot sets the shift point between 3000 and 6000 rpm
  shiftRpm = map(analogRead(POT_PIN), 0, 4095, 3000, 6000);
#endif

  float rpm;
  if (holdRpm >= 0) {
    rpm = holdRpm;
  } else if (handMode) {
    throttle.step(dt, digitalRead(HAND_PIN) == LOW);
    rpm = throttle.rpm;
  } else if (sweep) {
    rpm = IDLE_RPM + (shiftRpm + 400 - IDLE_RPM) * (0.5f - 0.5f * cosf(now / 4000.0f * PI));
  } else {
    if (running) car.step(dt);
    rpm = car.rpm;
  }

  int n = ledsFor(rpm);
  setBar(n);
  updateRgb(rpm, now);

  if (now - printAt > 200) {
    printAt = now;
    if (handMode) Serial.printf("rpm %4.0f  hand %s  bar %2d/10  shift @ %d  [", rpm, throttle.hand ? "YES" : "no ", n, shiftRpm);
    else Serial.printf("rpm %4.0f  gear %d  bar %2d/10  shift @ %d  [", rpm, car.gear, n, shiftRpm);
    for (int i = 1; i <= 10; i++) Serial.print(i <= n ? (i > 8 ? 'R' : i > 6 ? 'Y' : 'G') : '.');
    Serial.println(rpm >= shiftRpm ? "]  SHIFT!" : "]");
  }
  delay(10);
}
