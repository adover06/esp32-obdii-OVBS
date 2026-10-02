// Live car data: Bluetooth -> ELM327 (Veepeak) -> OBD-II, on a background task.
//
// Same logic as the esp32_obd_test sketch, but it runs on its own FreeRTOS task
// (think: a Python thread) so connecting and waiting for the car never freezes
// the screen. The task writes into `shared` while holding `lock`; once per
// frame the screen loop calls copyInto() to take a consistent snapshot.
//
// Include after dash.h. ESP32 only (not used by the desktop preview).
#pragma once

#include "BluetoothSerial.h"
#include "ELMduino.h"

namespace obd {

// ---- settings ------------------------------------------------------------------
const char* ADAPTER_NAME = "OBDII";
uint8_t     ADAPTER_MAC[6] = { 0x00, 0x1D, 0xA5, 0x10, 0x29, 0x40 };   // 00:1D:A5:10:29:40
const char* ADAPTER_PIN  = "1234";
const char  PROTOCOL     = SAE_J1850_PWM_41_KBAUD;   // 2000 Focus
const int   TIMEOUT_MS   = 2000;
const bool  ELM_RAW_DEBUG    = false;                    // true = print raw adapter traffic
const bool  LOG_EVERY_REPLY = false;                 // true = print every value as it arrives

// 2000 Focus automatic (4F27E): estimated mph per 1000 rpm in each gear,
// from the gear ratios, final drive and stock tire size. Used to guess the gear.
const float AUTO_MPH_PER_K[5] = { 0, 6.5f, 12.3f, 18.4f, 25.3f };

// ---- what we ask the car for ------------------------------------------------------
enum ItemId { RPM, SPEED, THROTTLE, LOAD, MAF, TIMING, COOLANT, INTAKE, FUEL_TRIM, BATTERY, CHECK_ENGINE, CODES, ITEM_COUNT };

struct ItemDef { const char* name; const char* unit; uint8_t decimals; uint32_t intervalMs; };
const ItemDef DEFS[ITEM_COUNT] = {
  // name          unit    dec  how often (ms, 0 = as often as possible)
  { "RPM",         "",      0,  0     },   // also asked every other request
  { "SPEED",       "mph",   0,  0     },
  { "THROTTLE",    "%",     0,  0     },
  { "LOAD",        "%",     0,  300   },
  { "MAF",         "g/s",   1,  500   },
  { "TIMING",      "deg",   1,  1000  },
  { "COOLANT",     "F",     0,  3000  },
  { "INTAKE AIR",  "F",     0,  5000  },
  { "FUEL TRIM",   "%",     1,  2000  },
  { "BATTERY",     "V",     1,  5000  },
  { "CHECK ENG",   "",      0,  10000 },
  { "CODES",       "",      0,  30000 },
};

// ---- shared between the OBD task (writes) and the screen loop (reads) ------------
struct Shared {
  float value[ITEM_COUNT] = {};
  int8_t status[ITEM_COUNT];          // ELMduino result codes, -99 = not asked yet
  uint32_t lastOkMs[ITEM_COUNT] = {};
  bool milOn = false;
  int codeCount = 0;
  char codes[8][6] = {};
  int codesListed = 0;
  bool codesRead = false;
  char linkLine[48] = "BT : starting";
  uint8_t linkColor = dash::YELLOW;
  char elmLine[48] = "ELM: waiting for Bluetooth";
  uint8_t elmColor = dash::DIM;
  bool polling = false;               // true = connected and reading the car
  float repliesPerSec = 0;
  uint32_t okCount[ITEM_COUNT] = {}, errCount[ITEM_COUNT] = {};
  char adapterInfo[24] = "";
  char protocolInfo[28] = "";
  uint32_t linkUpMs = 0, connects = 0, drops = 0;
  Shared() { for (int i = 0; i < ITEM_COUNT; i++) status[i] = -99; }
};

Shared shared;
SemaphoreHandle_t lock = nullptr;
TaskHandle_t taskHandle = nullptr;
BluetoothSerial bt;
ELM327 elm;

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

uint8_t statusColor(int8_t s)
{
  if (s == ELM_SUCCESS) return dash::GREEN;
  if (s == ELM_NO_DATA) return dash::YELLOW;
  if (s == -99) return dash::DIM;
  return dash::RED;
}

void setLink(const char* text, uint8_t color)
{
  xSemaphoreTake(lock, portMAX_DELAY);
  strncpy(shared.linkLine, text, sizeof(shared.linkLine) - 1);
  shared.linkColor = color;
  xSemaphoreGive(lock);
}

void setElm(const char* text, uint8_t color)
{
  xSemaphoreTake(lock, portMAX_DELAY);
  strncpy(shared.elmLine, text, sizeof(shared.elmLine) - 1);
  shared.elmColor = color;
  xSemaphoreGive(lock);
}

// ---- asking for one value ----------------------------------------------------------
// Starts or continues a request; call until elm.nb_rx_state != ELM_GETTING_MSG.
// mil/codes are filled in for CHECK_ENGINE.
float request(int id, bool& mil, int& codes)
{
  switch (id) {
    case RPM:       return elm.rpm();
    case SPEED:     return elm.mph();
    case THROTTLE:  return elm.throttle();
    case LOAD:      return elm.engineLoad();
    case MAF:       return elm.mafRate();
    case TIMING:    return elm.timingAdvance();
    case COOLANT:   return elm.engineCoolantTemp() * 9.0f / 5.0f + 32;   // car answers in C
    case INTAKE:    return elm.intakeAirTemp() * 9.0f / 5.0f + 32;
    case FUEL_TRIM: return elm.shortTermFuelTrimBank_1();
    case BATTERY:   return elm.batteryVoltage();
    case CHECK_ENGINE: {
      // first status byte: bit 7 = light on, bits 0-6 = number of stored codes
      uint32_t st = elm.monitorStatus();
      if (elm.nb_rx_state == ELM_SUCCESS) {
        uint8_t a = st >> 24;
        mil = a & 0x80;
        codes = a & 0x7F;
      }
      return codes;
    }
    case CODES:
      elm.currentDTCCodes(false);   // non-blocking; results in elm.DTC_Response
      return elm.DTC_Response.codesFound;
  }
  return 0;
}

// Mode 03 reply -> codes like "P0133". ELMduino's own parser assumes the CAN
// layout (a count byte after "43"); J1850 PWM (this car) sends one line per
// 3 codes: "43" + three 4-digit codes, padded with 0000. Handles both.
static int hexDigit(char c) { return c <= '9' ? c - '0' : (c & ~0x20) - 'A' + 10; }

int parseTroubleCodes(const char* p, char out[][6], int maxCodes)
{
  int n = 0;
  if (!p) return 0;
  while (*p && n < maxCodes) {
    char line[64];
    int len = 0;
    while (*p && *p != '\r') {                       // one reply line, hex digits only
      char c = *p++;
      if (isxdigit((unsigned char)c) && len < 63) line[len++] = c;
    }
    while (*p == '\r') p++;
    line[len] = 0;
    if (len < 6 || line[0] != '4' || line[1] != '3') continue;
    int start = (len - 2) % 4 == 2 ? 4 : 2;          // CAN: skip the count byte
    for (int i = start; i + 4 <= len && n < maxCodes; i += 4) {
      if (!strncmp(line + i, "0000", 4)) continue;   // padding
      int first = hexDigit(line[i]);
      out[n][0] = "PCBU"[first >> 2];
      out[n][1] = '0' + (first & 3);
      memcpy(out[n] + 2, line + i + 1, 3);
      out[n][5] = 0;
      n++;
    }
  }
  return n;
}

// RPM every other request (it drives the tach and shift light), the rest
// take turns as they come due.
int pickNext(int justDone, int& rr, const uint32_t* lastTry)
{
  if (justDone != RPM) return RPM;
  uint32_t now = millis();
  for (int step = 0; step < ITEM_COUNT; step++) {
    rr = (rr + 1) % ITEM_COUNT;
    if (rr == RPM) continue;
    if (lastTry[rr] == 0 || now - lastTry[rr] >= DEFS[rr].intervalMs) return rr;
  }
  return RPM;
}

// ---- the background task ------------------------------------------------------------
bool connectBluetooth(int attempt)
{
  char msg[48];
  snprintf(msg, sizeof(msg), "BT : connecting to \"%s\" (try %d)", ADAPTER_NAME, attempt);
  setLink(msg, dash::YELLOW);
  Serial.printf("[BT ] try %d: connecting by name \"%s\"...  (free %u, largest block %u)\n", attempt,
                ADAPTER_NAME, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  uint32_t t0 = millis();
  bool ok = bt.connect(ADAPTER_NAME);
  if (!ok) {
    Serial.printf("[BT ] by name failed after %.1f s, trying MAC 00:1D:A5:10:29:40...\n", (millis() - t0) / 1000.0);
    setLink("BT : name failed, trying MAC address", dash::YELLOW);
    t0 = millis();
    ok = bt.connect(ADAPTER_MAC);
  }
  if (ok && bt.connected(1000)) {
    Serial.printf("[BT ] CONNECTED in %.1f s  (free %u, largest block %u)\n", (millis() - t0) / 1000.0,
                  ESP.getFreeHeap(), ESP.getMaxAllocHeap());
    setLink("BT : connected to adapter", dash::GREEN);
    return true;
  }
  Serial.println("[BT ] FAILED. Check: adapter LED on, not connected to a phone/Mac, in range.");
  setLink("BT : FAILED - retrying", dash::RED);
  return false;
}

bool initElm(int attempt)
{
  char msg[48];
  snprintf(msg, sizeof(msg), "ELM: starting J1850 PWM (try %d)", attempt);
  setElm(msg, dash::YELLOW);
  Serial.printf("[ELM] try %d: init, protocol '%c', timeout %d ms...\n", attempt, PROTOCOL, TIMEOUT_MS);
  uint32_t t0 = millis();
  // Start from a fresh ELMduino object every time: begin() mallocs a new reply
  // buffer without freeing the old one (a leak on every reconnect), and a
  // request cut off by a dropout would leave its private state machine waiting.
  if (elm.payload) free(elm.payload);
  elm = ELM327();
  if (!elm.begin(bt, ELM_RAW_DEBUG, TIMEOUT_MS, PROTOCOL)) {
    Serial.println("[ELM] adapter did not answer. Retrying...");
    setElm("ELM: adapter not answering - retrying", dash::RED);
    return false;
  }
  // With a fixed protocol, begin() only proves the ADAPTER answered.
  // Ask the car for something real before calling it connected.
  Serial.printf("[ELM] adapter OK in %.1f s, asking the car...\n", (millis() - t0) / 1000.0);
  uint32_t t1 = millis();
  do {
    elm.supportedPIDs_1_20();
    vTaskDelay(1);
  } while (elm.nb_rx_state == ELM_GETTING_MSG && millis() - t1 < 6000);
  if (elm.nb_rx_state != ELM_SUCCESS) {
    Serial.printf("[ELM] car did not answer (%s). Is the key at ON (not just ACC)?\n", statusText(elm.nb_rx_state));
    setElm("ELM: car not answering - key ON?", dash::RED);
    return false;
  }
  Serial.printf("[ELM] car is answering (%.1f s total)\n", (millis() - t0) / 1000.0);

  // adapter version and the protocol it's using, for the SYS screen
  char info[24] = "", proto[28] = "";
  if (elm.sendCommand_Blocking("ATI") == ELM_SUCCESS) strncpy(info, elm.payload, sizeof(info) - 1);
  if (elm.sendCommand_Blocking("ATDP") == ELM_SUCCESS) strncpy(proto, elm.payload, sizeof(proto) - 1);
  for (char* c = info; *c; c++) if (*c == '\r' || *c == '\n') *c = ' ';
  for (char* c = proto; *c; c++) if (*c == '\r' || *c == '\n') *c = ' ';
  Serial.printf("[ELM] adapter \"%s\", protocol \"%s\"\n", info, proto);

  xSemaphoreTake(lock, portMAX_DELAY);
  strncpy(shared.adapterInfo, info, sizeof(shared.adapterInfo) - 1);
  strncpy(shared.protocolInfo, proto, sizeof(shared.protocolInfo) - 1);
  shared.linkUpMs = millis();
  shared.connects++;
  xSemaphoreGive(lock);
  setElm("ELM: connected to car", dash::GREEN);
  return true;
}

void pollUntilDisconnected()
{
  uint32_t lastTry[ITEM_COUNT] = {};
  int current = RPM, rr = 0;
  uint32_t started = millis(), windowStart = millis(), replies = 0;
  bool mil = false;
  int codes = 0;

  xSemaphoreTake(lock, portMAX_DELAY);
  shared.polling = true;
  xSemaphoreGive(lock);

  while (bt.connected()) {
    float v = request(current, mil, codes);
    if (elm.nb_rx_state == ELM_GETTING_MSG) { vTaskDelay(1); continue; }   // still waiting

    // this request finished, good or bad
    int8_t st = elm.nb_rx_state;
    uint32_t now = millis();
    lastTry[current] = now;
    replies++;

    xSemaphoreTake(lock, portMAX_DELAY);
    shared.status[current] = st;
    if (st == ELM_SUCCESS) shared.okCount[current]++;
    else shared.errCount[current]++;
    if (st == ELM_SUCCESS) {
      shared.value[current] = v;
      shared.lastOkMs[current] = now;
      if (current == CHECK_ENGINE) { shared.milOn = mil; shared.codeCount = codes; }
      if (current == CODES) {
        shared.codesRead = true;
        shared.codesListed = parseTroubleCodes(elm.payload, shared.codes, 8);
      }
    }
    if (current == CODES && st == ELM_NO_DATA) {   // some ECUs answer NO DATA when nothing is stored
      shared.codesRead = true;
      shared.codesListed = 0;
    }
    if (now - windowStart >= 1000) {
      shared.repliesPerSec = replies * 1000.0f / (now - windowStart);
      replies = 0;
      windowStart = now;
    }
    xSemaphoreGive(lock);

    if (st != ELM_SUCCESS) {
      Serial.printf("[%-10s] %-8s (code %d)%s\n", DEFS[current].name, statusText(st), st,
                    st == ELM_NO_DATA ? "  <- car may not support this" : "");
    } else if (LOG_EVERY_REPLY) {
      Serial.printf("[%-10s] OK  %.1f %s  (%lu ms)\n", DEFS[current].name, v, DEFS[current].unit,
                    (unsigned long)(now - started));
    }

    current = pickNext(current, rr, lastTry);
    started = millis();
    vTaskDelay(1);   // let the Bluetooth stack breathe
  }

  xSemaphoreTake(lock, portMAX_DELAY);
  shared.polling = false;
  shared.linkUpMs = 0;
  shared.drops++;
  xSemaphoreGive(lock);
  Serial.println("[BT ] connection LOST");
  setLink("BT : connection lost - reconnecting", dash::RED);
  setElm("ELM: waiting for Bluetooth", dash::DIM);
}

void task(void*)
{
  int btTry = 0, elmTry = 0;
  for (;;) {
    if (!bt.connected()) {
      if (!connectBluetooth(++btTry)) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
      elmTry = 0;
    }
    if (!initElm(++elmTry)) { vTaskDelay(pdMS_TO_TICKS(3000)); continue; }
    pollUntilDisconnected();
  }
}

// Step 1, from setup(): start the Bluetooth radio. Returns false if there
// isn't enough memory, so the sketch can free some and try again.
bool beginBluetooth()
{
  if (!lock) lock = xSemaphoreCreateMutex();
  // true = we start the connection (master), true = release BLE memory (we only use Classic)
  if (!bt.begin("ESP32-OBD", true, true)) {
    Serial.println("[BT ] Bluetooth failed to start (out of memory?)");
    setLink("BT : could not start Bluetooth (memory?)", dash::RED);
    return false;
  }
  bt.setPin(ADAPTER_PIN, strlen(ADAPTER_PIN));
  return true;
}

// Step 2: connect + read the car on a background task (core 0).
// The screen loop keeps running on core 1.
void start()
{
  xTaskCreatePinnedToCore(task, "obd", 8192, nullptr, 1, &taskHandle, 0);
}

// Smallest amount of stack the OBD task has had left (bytes); for tuning.
uint32_t stackFreeBytes()
{
  return taskHandle ? uxTaskGetStackHighWaterMark(taskHandle) : 0;
}

// ---- turning raw values into the dashboard's Telemetry -------------------------------
int estimateGear(float rpm, float mph)
{
  if (mph < 2 || rpm < 300) return 0;
  float ratio = mph / (rpm / 1000.0f);   // mph per 1000 rpm right now
  int best = 1;
  float bestErr = 1e9;
  for (int g = 1; g <= 4; g++) {
    float err = fabsf(ratio - AUTO_MPH_PER_K[g]) / AUTO_MPH_PER_K[g];
    if (err < bestErr) { bestErr = err; best = g; }
  }
  return best;
}

// Call once per frame from the screen loop.
void copyInto(dash::Telemetry& t, float dt)
{
  static bool wasPolling = false, wasMil = false, wasOver = false;
  static int loggedGear = 0, candidateGear = 0;
  static float candidateSince = 0;

  xSemaphoreTake(lock, portMAX_DELAY);
  Shared s = shared;   // copy the whole thing while it's locked
  xSemaphoreGive(lock);

  t.clock += dt;
  auto& d = t.d;
  // each value is the last GOOD reading (0 until the car has answered once);
  // anything that isn't a real number becomes 0 so it can't upset the drawing
  auto num = [&](int i) { float v = s.value[i]; return isfinite(v) ? v : 0.0f; };
  d.rpm      = num(RPM);
  d.mph      = num(SPEED);
  d.throttle = num(THROTTLE);
  d.load     = num(LOAD);
  d.maf      = num(MAF);
  d.timing   = num(TIMING);
  d.coolantF = num(COOLANT);
  d.iatF     = num(INTAKE);
  d.stft     = num(FUEL_TRIM);
  d.volts    = num(BATTERY);
  d.mpg = (d.mph > 1 && d.maf > 0.5f) ? min(710.7f * d.mph / d.maf, 99.9f) : 0;
  d.gear = estimateGear(d.rpm, d.mph);

  // DIAG screen info
  strncpy(t.linkLine, s.linkLine, sizeof(t.linkLine));
  strncpy(t.elmLine, s.elmLine, sizeof(t.elmLine));
  t.linkColor = s.linkColor;
  t.elmColor = s.elmColor;
  t.repliesPerSec = s.repliesPerSec;
  t.readingCount = CODES;   // every item except CODES gets a row
  for (int i = 0; i < CODES; i++) {
    dash::Reading& r = t.readings[i];
    r.name = DEFS[i].name;
    r.unit = DEFS[i].unit;
    r.decimals = DEFS[i].decimals;
    r.value = s.value[i];
    strncpy(r.status, statusText(s.status[i]), sizeof(r.status) - 1);
    r.statusColor = statusColor(s.status[i]);
    r.lastOkMs = s.lastOkMs[i];
    r.okCount = s.okCount[i];
    r.errCount = s.errCount[i];
  }
  t.okTotal = t.errTotal = 0;
  for (int i = 0; i < ITEM_COUNT; i++) { t.okTotal += s.okCount[i]; t.errTotal += s.errCount[i]; }
  strncpy(t.adapterInfo, s.adapterInfo, sizeof(t.adapterInfo) - 1);
  strncpy(t.protocolInfo, s.protocolInfo, sizeof(t.protocolInfo) - 1);
  t.linkUpMs = s.linkUpMs;
  t.connects = s.connects;
  t.drops = s.drops;
  t.milOn = s.milOn;
  t.codeCount = s.codeCount;
  t.codesRead = s.codesRead;
  t.codesListed = s.codesListed;
  memcpy(t.codes, s.codes, sizeof(t.codes));

  // memory, refreshed once a second, shown on the DIAG screen
  static uint32_t lastMem = 0;
  if (millis() - lastMem >= 1000) {
    lastMem = millis();
    snprintf(t.memLine, sizeof(t.memLine), "MEM %uK free %uK blk",
             (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024));
  }

  t.sourceTag = s.polling ? "LIVE" : "NO LINK";
  t.sourceColor = s.polling ? dash::GREEN : dash::RED;

  // events for the TERM screen's log
  if (s.polling != wasPolling) {
    t.log.add(t.clock, s.polling ? "LINK UP  J1850 PWM" : "LINK LOST");
    wasPolling = s.polling;
  }
  if (s.milOn != wasMil) {
    t.log.add(t.clock, s.milOn ? "CHECK ENGINE ON  %d CODES" : "CHECK ENGINE OFF", s.codeCount);
    wasMil = s.milOn;
  }
  bool over = d.rpm >= dash::SHIFT_RPM;
  if (over && !wasOver) t.log.add(t.clock, "REDLINE  %d RPM", (int)d.rpm);
  wasOver = over;
  // log a gear change once the new gear has held for half a second
  if (d.gear != candidateGear) { candidateGear = d.gear; candidateSince = t.clock; }
  if (candidateGear != loggedGear && t.clock - candidateSince > 0.5f) {
    if (candidateGear > 0 && loggedGear > 0) t.log.add(t.clock, "SHIFT %d>%d  @%d", loggedGear, candidateGear, (int)d.rpm);
    loggedGear = candidateGear;
  }

  t.sample(dt);
}

} // namespace obd
