// ============================================================================
//  OBD TEST DASHBOARD  (ESP32 + Bluetooth ELM327 -> composite video)
//
//  Connects to the Veepeak over Bluetooth, reads real values from the car,
//  and shows every value WITH ITS STATUS so you can see what works:
//    OK       - the car answered
//    NO DATA  - the car doesn't support that value (normal for some)
//    TIMEOUT / NO RESP / ERROR - something went wrong
//
//  Everything is also logged to the Serial Monitor (115200 baud).
//
//  Wiring:  GPIO25 -> RCA center, GND -> RCA shell
//  Board:   ESP32-WROOM-DA Module
//  Fits the default partition (~85% of flash). If you add more and it
//  says "too big": Tools -> Partition Scheme -> "Huge APP (3MB No OTA)"
//  Car:     key at ON (dash lights on). Unpair the adapter from phones/Macs.
// ============================================================================

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "BluetoothSerial.h"
#include "ELMduino.h"

// ============================================================================
//  1. SETTINGS
// ============================================================================

const char* ADAPTER_NAME = "OBDII";                                    // Bluetooth name
uint8_t     ADAPTER_MAC[6] = { 0x00, 0x1D, 0xA5, 0x10, 0x29, 0x40 };   // fallback: 00:1D:A5:10:29:40
const char* ADAPTER_PIN  = "1234";

const char OBD_PROTOCOL  = SAE_J1850_PWM_41_KBAUD;   // '1' = J1850 PWM (2000 Focus)
const int  ELM_TIMEOUT_MS = 2000;                    // J1850 is slow, give it time

// true = also print the raw text going to/from the adapter (very chatty)
const bool ELM_RAW_DEBUG = false;

const int VIDEO_PIN = 25;

const uint32_t BLACK = 0x000000;
const uint32_t WHITE = 0xFFFFFF;
const uint32_t CYAN  = 0x00DBFF;
const uint32_t LABEL = 0x6D92FF;
const uint32_t GREEN = 0x24FF6D;
const uint32_t AMBER = 0xFFB600;
const uint32_t RED   = 0xFF2424;
const uint32_t DIM   = 0x496DAA;


// ============================================================================
//  2. VIDEO SETUP  (same as the other sketches)
// ============================================================================

class TVScreen : public lgfx::LGFX_Device
{
  lgfx::Panel_CVBS panel;

public:
  TVScreen()
  {
    auto size = panel.config();
    size.memory_width = size.panel_width = 360;
    size.memory_height = size.panel_height = 240;
    panel.config(size);

    auto signal = panel.config_detail();
    signal.signal_type  = signal.signal_type_t::NTSC;
    signal.pin_dac      = VIDEO_PIN;
    signal.output_level = 128;
    panel.config_detail(signal);

    setPanel(&panel);
  }
};

TVScreen tv;
BluetoothSerial SerialBT;
ELM327 elm;


// ============================================================================
//  3. THE VALUES WE ASK FOR
//  Each item remembers its last value, last status, and when it last worked.
//  intervalMs = how often to ask (0 = as often as possible).
// ============================================================================

enum ItemId { RPM, SPEED, THROTTLE, LOAD, TIMING, COOLANT, INTAKE, BATTERY, CHECK_ENGINE, CODES, ITEM_COUNT };

struct Item {
  const char* name;
  const char* unit;
  int decimals;
  uint32_t intervalMs;
  float value = 0;
  int8_t status = -99;          // -99 = not asked yet
  uint32_t lastOkMs = 0;
  uint32_t lastTryMs = 0;
  uint32_t okCount = 0, failCount = 0;
};

Item items[ITEM_COUNT] = {
  // name         unit    dec  interval
  { "RPM",        "",      0,  0     },
  { "SPEED",      "mph",   0,  0     },
  { "THROTTLE",   "%",     0,  0     },
  { "LOAD",       "%",     0,  500   },
  { "TIMING",     "deg",   1,  1000  },
  { "COOLANT",    "F",     0,  3000  },
  { "INTAKE AIR", "F",     0,  3000  },
  { "BATTERY",    "V",     1,  5000  },
  { "CHECK ENG",  "",      0,  10000 },
  { "CODES",      "",      0,  30000 },
};

bool checkEngineLightOn = false;   // filled in by CHECK_ENGINE
int  storedCodeCount = 0;

// Start (or continue) the request for one item. Like the example sketch,
// call this repeatedly until elm.nb_rx_state is no longer ELM_GETTING_MSG.
float requestItem(int id)
{
  switch (id) {
    case RPM:      return elm.rpm();
    case SPEED:    return elm.mph();
    case THROTTLE: return elm.throttle();
    case LOAD:     return elm.engineLoad();
    case TIMING:   return elm.timingAdvance();
    case COOLANT:  return elm.engineCoolantTemp() * 9.0 / 5.0 + 32;   // car answers in C
    case INTAKE:   return elm.intakeAirTemp() * 9.0 / 5.0 + 32;
    case BATTERY:  return elm.batteryVoltage();   // measured by the adapter itself

    case CHECK_ENGINE: {
      // 4 status bytes; in the first byte, bit 7 = light on, bits 0-6 = number of codes
      uint32_t status = elm.monitorStatus();
      if (elm.nb_rx_state == ELM_SUCCESS) {
        uint8_t a = status >> 24;
        checkEngineLightOn = a & 0x80;
        storedCodeCount = a & 0x7F;
      }
      return storedCodeCount;
    }

    case CODES:
      elm.currentDTCCodes(false);   // false = non-blocking, results in elm.DTC_Response
      return elm.DTC_Response.codesFound;
  }
  return 0;
}

const char* statusText(int8_t s)
{
  switch (s) {
    case ELM_SUCCESS:           return "OK";
    case ELM_NO_DATA:           return "NO DATA";
    case ELM_TIMEOUT:           return "TIMEOUT";
    case ELM_NO_RESPONSE:       return "NO RESP";
    case ELM_UNABLE_TO_CONNECT: return "UNABLE";
    case ELM_GARBAGE:           return "GARBAGE";
    case ELM_BUFFER_OVERFLOW:   return "OVERFLOW";
    case ELM_STOPPED:           return "STOPPED";
    case -99:                   return "--";
    default:                    return "ERROR";
  }
}


// ============================================================================
//  4. CONNECTION STATE
// ============================================================================

enum ConnState { BT_CONNECTING, ELM_INIT, POLLING };
ConnState conn = BT_CONNECTING;

char btLine[48]  = "BT : starting";
char elmLine[48] = "ELM: waiting for Bluetooth";
uint32_t btColor = AMBER, elmColor = DIM;

int currentItem = 0;
uint32_t requestStartMs = 0;
uint32_t repliesThisWindow = 0, windowStartMs = 0;
float repliesPerSecond = 0;


// ============================================================================
//  5. DRAWING (straight to the TV, no big canvas, so Bluetooth has memory)
//  setTextColor(fg, bg) + setTextPadding() overwrite the old text in place.
// ============================================================================

const int ROW_Y0 = 74, ROW_H = 16;

void drawText(const char* s, int x, int y, uint32_t color, textdatum_t datum, int padWidth)
{
  tv.setTextDatum(datum);
  tv.setTextColor(color, BLACK);
  tv.setTextPadding(padWidth);
  tv.drawString(s, x, y);
}

void drawStaticParts()
{
  tv.fillScreen(BLACK);
  tv.setFont(&fonts::Font2);
  drawText("OBD TEST   2000 FOCUS   J1850 PWM", 8, 2, CYAN, textdatum_t::top_left, 0);
  drawText("VALUE", 8, 56, DIM, textdatum_t::top_left, 0);
  drawText("READING", 200, 56, DIM, textdatum_t::top_right, 0);
  drawText("STATUS", 212, 56, DIM, textdatum_t::top_left, 0);
  drawText("AGE", 352, 56, DIM, textdatum_t::top_right, 0);
  tv.drawFastHLine(8, 71, 344, DIM);
  for (int i = 0; i < CODES; i++) {
    drawText(items[i].name, 8, ROW_Y0 + i * ROW_H, LABEL, textdatum_t::top_left, 0);
  }
}

void drawStatusLines()
{
  tv.setFont(&fonts::Font2);
  drawText(btLine, 8, 20, btColor, textdatum_t::top_left, 344);

  char line[64];
  if (conn == POLLING) {
    snprintf(line, sizeof(line), "%s   %.1f replies/s", elmLine, repliesPerSecond);
    drawText(line, 8, 36, elmColor, textdatum_t::top_left, 344);
  } else {
    drawText(elmLine, 8, 36, elmColor, textdatum_t::top_left, 344);
  }
}

void drawItemRow(int i)
{
  Item& it = items[i];
  int y = ROW_Y0 + i * ROW_H;
  char text[24];
  tv.setFont(&fonts::Font2);

  // reading
  if (it.okCount == 0) {
    snprintf(text, sizeof(text), "--");
  } else if (i == CHECK_ENGINE) {
    snprintf(text, sizeof(text), "%s  %d code%s", checkEngineLightOn ? "ON" : "off",
             storedCodeCount, storedCodeCount == 1 ? "" : "s");
  } else {
    snprintf(text, sizeof(text), "%.*f %s", it.decimals, it.value, it.unit);
  }
  uint32_t readingColor = WHITE;
  if (i == CHECK_ENGINE && checkEngineLightOn) readingColor = AMBER;
  drawText(text, 200, y, readingColor, textdatum_t::top_right, 110);

  // status
  uint32_t sc = it.status == ELM_SUCCESS ? GREEN : it.status == ELM_NO_DATA ? AMBER : it.status == -99 ? DIM : RED;
  drawText(statusText(it.status), 212, y, sc, textdatum_t::top_left, 90);

  // age = seconds since the last good reading
  if (it.okCount == 0) snprintf(text, sizeof(text), "--");
  else snprintf(text, sizeof(text), "%.1fs", (millis() - it.lastOkMs) / 1000.0);
  drawText(text, 352, y, DIM, textdatum_t::top_right, 50);
}

void drawCodesLine()
{
  Item& it = items[CODES];
  char line[96];
  if (it.okCount == 0) {
    snprintf(line, sizeof(line), "CODES: %s", it.status == -99 ? "not read yet" : statusText(it.status));
  } else if (elm.DTC_Response.codesFound == 0) {
    snprintf(line, sizeof(line), "CODES: none stored");
  } else {
    int n = snprintf(line, sizeof(line), "CODES:");
    for (int c = 0; c < elm.DTC_Response.codesFound && n < (int)sizeof(line) - 8; c++) {
      n += snprintf(line + n, sizeof(line) - n, " %s", elm.DTC_Response.codes[c]);
    }
  }
  tv.setFont(&fonts::Font2);
  drawText(line, 8, ROW_Y0 + CODES * ROW_H + 4, it.okCount && elm.DTC_Response.codesFound ? AMBER : LABEL,
           textdatum_t::top_left, 344);
}


// ============================================================================
//  6. CONNECTING
// ============================================================================

void setBt(const char* text, uint32_t color)  { strncpy(btLine, text, sizeof(btLine) - 1);  btColor = color;  drawStatusLines(); }
void setElm(const char* text, uint32_t color) { strncpy(elmLine, text, sizeof(elmLine) - 1); elmColor = color; drawStatusLines(); }

// List nearby Bluetooth devices, to help when the adapter isn't found.
void scanAndPrint()
{
  Serial.println("[BT ] scanning 5 s for nearby devices...");
  BTScanResults* found = SerialBT.discover(5000);
  if (!found || found->getCount() == 0) {
    Serial.println("[BT ] no devices found (is the adapter powered? LED on?)");
    return;
  }
  for (int i = 0; i < found->getCount(); i++) {
    BTAdvertisedDevice* d = found->getDevice(i);
    Serial.printf("[BT ]   found \"%s\"  %s  rssi %d\n", d->getName().c_str(),
                  d->getAddress().toString().c_str(), d->getRSSI());
  }
}

void tryBluetooth()
{
  static int attempt = 0;
  attempt++;
  char msg[48];

  snprintf(msg, sizeof(msg), "BT : connecting to \"%s\" (try %d)...", ADAPTER_NAME, attempt);
  setBt(msg, AMBER);
  Serial.printf("[BT ] try %d: connecting by name \"%s\"...\n", attempt, ADAPTER_NAME);
  uint32_t t0 = millis();
  bool ok = SerialBT.connect(ADAPTER_NAME);

  if (!ok) {
    Serial.printf("[BT ] by name failed after %.1f s, trying MAC 00:1D:A5:10:29:40...\n", (millis() - t0) / 1000.0);
    setBt("BT : name failed, trying MAC address...", AMBER);
    t0 = millis();
    ok = SerialBT.connect(ADAPTER_MAC);
  }

  if (ok && SerialBT.connected(1000)) {
    Serial.printf("[BT ] CONNECTED in %.1f s\n", (millis() - t0) / 1000.0);
    setBt("BT : connected to adapter", GREEN);
    conn = ELM_INIT;
  } else {
    Serial.println("[BT ] FAILED. Check: adapter LED on, not connected to a phone/Mac, in range.");
    setBt("BT : FAILED - retrying (see Serial Monitor)", RED);
    scanAndPrint();
    delay(2000);
  }
}

void tryElmInit()
{
  static int attempt = 0;
  attempt++;
  char msg[48];
  snprintf(msg, sizeof(msg), "ELM: starting, protocol J1850 PWM (try %d)...", attempt);
  setElm(msg, AMBER);
  Serial.printf("[ELM] try %d: init with protocol '%c', timeout %d ms...\n", attempt, OBD_PROTOCOL, ELM_TIMEOUT_MS);

  uint32_t t0 = millis();
  if (elm.begin(SerialBT, ELM_RAW_DEBUG, ELM_TIMEOUT_MS, OBD_PROTOCOL)) {
    Serial.printf("[ELM] init OK in %.1f s - car is answering\n", (millis() - t0) / 1000.0);
    setElm("ELM: connected to car", GREEN);
    conn = POLLING;
    windowStartMs = millis();
  } else {
    Serial.println("[ELM] init FAILED. Is the key at ON (not just ACC)? Set ELM_RAW_DEBUG = true for details.");
    setElm("ELM: FAILED - key ON? retrying...", RED);
    if (!SerialBT.connected()) conn = BT_CONNECTING;
    delay(3000);
  }
}


// ============================================================================
//  7. POLLING: one request at a time, round-robin
// ============================================================================

// Next item that is due, starting after the current one.
int pickNextItem(int after)
{
  uint32_t now = millis();
  for (int step = 1; step <= ITEM_COUNT; step++) {
    int i = (after + step) % ITEM_COUNT;
    if (items[i].lastTryMs == 0 || now - items[i].lastTryMs >= items[i].intervalMs) return i;
  }
  return (after + 1) % ITEM_COUNT;
}

void poll()
{
  if (requestStartMs == 0) requestStartMs = millis();

  float v = requestItem(currentItem);
  if (elm.nb_rx_state == ELM_GETTING_MSG) return;     // still waiting for the answer

  // the request finished (good or bad)
  Item& it = items[currentItem];
  uint32_t took = millis() - requestStartMs;
  it.status = elm.nb_rx_state;
  it.lastTryMs = millis();
  repliesThisWindow++;

  if (it.status == ELM_SUCCESS) {
    it.value = v;
    it.lastOkMs = millis();
    it.okCount++;
    Serial.printf("[%-10s] OK       %8.1f %-4s %5lu ms\n", it.name, v, it.unit, (unsigned long)took);
  } else {
    it.failCount++;
    Serial.printf("[%-10s] %-8s (code %d)      %5lu ms%s\n", it.name, statusText(it.status), it.status,
                  (unsigned long)took, it.status == ELM_NO_DATA ? "  <- car may not support this" : "");
  }

  if (currentItem == CODES) drawCodesLine();
  else drawItemRow(currentItem);

  currentItem = pickNextItem(currentItem);
  requestStartMs = 0;
}


// ============================================================================
//  8. SETUP AND LOOP
// ============================================================================

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== OBD TEST DASHBOARD ===");
  Serial.printf("[MEM] start: free %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  tv.setColorDepth(8);
  tv.init();
  drawStaticParts();
  drawStatusLines();
  for (int i = 0; i < CODES; i++) drawItemRow(i);
  drawCodesLine();
  Serial.printf("[MEM] after video: free %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // true = we are the "master" that starts the connection
  if (!SerialBT.begin("ESP32-OBD", true)) {
    Serial.println("[BT ] Bluetooth failed to start (out of memory?)");
    setBt("BT : could not start Bluetooth", RED);
    while (true) delay(1000);
  }
  SerialBT.setPin(ADAPTER_PIN, strlen(ADAPTER_PIN));
  Serial.printf("[MEM] after Bluetooth: free %u, largest block %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
}

void loop()
{
  switch (conn) {
    case BT_CONNECTING:
      tryBluetooth();
      break;

    case ELM_INIT:
      tryElmInit();
      break;

    case POLLING:
      if (!SerialBT.connected()) {
        Serial.println("[BT ] connection LOST");
        setBt("BT : connection lost - reconnecting", RED);
        setElm("ELM: waiting for Bluetooth", DIM);
        conn = BT_CONNECTING;
        requestStartMs = 0;
        break;
      }
      poll();
      break;
  }

  // once a second: refresh the replies/s figure and the "age" column
  static uint32_t lastRefresh = 0;
  if (millis() - lastRefresh >= 1000) {
    lastRefresh = millis();
    if (conn == POLLING) {
      repliesPerSecond = repliesThisWindow * 1000.0 / (millis() - windowStartMs);
      repliesThisWindow = 0;
      windowStartMs = millis();
      for (int i = 0; i < CODES; i++) drawItemRow(i);
    }
    drawStatusLines();
  }
}
