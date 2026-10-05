// Fake ELM327 OBD adapter + fake car, for testing the dashboard at a desk.
//
// Flash this on a SECOND ESP32. It appears over Bluetooth Classic exactly
// like the Veepeak in the car: name "OBDII", PIN 1234, and (IMPERSONATE)
// even the same Bluetooth address 00:1D:A5:10:29:40, so the dashboard
// connects to it without any code changes.
//
// It answers the AT commands ELMduino sends and the OBD-II requests the
// dashboard makes (RPM, speed, throttle, load, MAF, coolant, intake air,
// fuel trim, battery, MIL status, trouble codes), with values from a little
// simulated drive. TIMING answers "NO DATA", like the real 2000 Focus.
//
// Serial monitor (115200): shows every request; type a number + Enter to
// hold that RPM (e.g. 3800), or "a" for automatic driving again.
//
// Unplug the real adapter / keep the car off while this runs, so the
// dashboard can't connect to the wrong one.

#include "BluetoothSerial.h"
#include "esp_mac.h"

#define IMPERSONATE true   // use the real adapter's Bluetooth address
#define REPLY_MS 60        // delay per OBD reply; 60 ~ the real car, 0 = as fast as possible (stress test)
static const uint8_t ADAPTER_MAC[6] = { 0x00, 0x1D, 0xA5, 0x10, 0x29, 0x40 };

BluetoothSerial bt;

// ---- ELM327 settings the host can change --------------------------------------
bool echo = true, spaces = true, linefeeds = false, headers = false;

// ---- the fake car ------------------------------------------------------------------
struct Car {
  float rpm = 780, mph = 0, throttle = 0, load = 20, maf = 2.5, coolantF = 189, iatF = 88, stft = -0.9;
  int gear = 0, phase = 0;
  float t = 0;
  int holdRpm = -1;

  void step(float dt)
  {
    t += dt;
    if (holdRpm >= 0) {
      rpm += (holdRpm - rpm) * min(1.0f, dt * 4);
      throttle = holdRpm > 900 ? 30 : 0;
    } else {
      switch (phase) {
        case 0: rpm = 780 + 20 * sinf(t * 6); throttle = 0; mph = max(0.0f, mph - 8 * dt);
                if (t > 3) { phase = 1; gear = 1; t = 0; } break;
        case 1: throttle = 85; rpm += 2600.0f / gear * dt;            // pull through the gear
                mph = rpm / 1000 * (gear == 1 ? 6.5f : gear == 2 ? 12.3f : gear == 3 ? 18.4f : 25.3f);
                if (rpm > 4300) { if (gear < 4) { phase = 2; t = 0; } else { phase = 3; t = 0; } } break;
        case 2: throttle = 20; rpm -= 9000 * dt;                        // shift
                if (rpm < 2700) { gear++; phase = 1; } break;
        case 3: throttle = 25; rpm += (2300 - rpm) * dt;                // cruise
                if (t > 6) { phase = 4; t = 0; } break;
        case 4: throttle = 0; rpm = max(780.0f, rpm - 400 * dt);         // coast down
                mph = max(0.0f, mph - 6 * dt); if (rpm <= 780 && mph < 1) { phase = 0; gear = 0; t = 0; } break;
      }
    }
    load = constrain(15 + throttle * 0.7f + rpm / 6000 * 10, 0, 100);
    maf = rpm / 1000 * (2 + throttle / 10);
    coolantF += (195 - coolantF) * dt * 0.01f;
  }
};
Car car;

// ---- replies -----------------------------------------------------------------------
String hexBytes(const uint8_t* b, int n)
{
  String s;
  char h[4];
  for (int i = 0; i < n; i++) {
    snprintf(h, sizeof(h), spaces ? "%02X " : "%02X", b[i]);
    s += h;
  }
  s.trim();
  return s;
}

String obdReply(const String& req)
{
  if (req.length() < 2) return "?";
  int mode = strtol(req.substring(0, 2).c_str(), nullptr, 16);
  if (mode == 0x03) {                                  // stored trouble codes: P0171, P0420
    const uint8_t r[] = { 0x43, 0x01, 0x71, 0x04, 0x20, 0x00, 0x00 };
    return hexBytes(r, sizeof(r));
  }
  if (mode != 0x01 || req.length() < 4) return "NO DATA";
  int pid = strtol(req.substring(2, 4).c_str(), nullptr, 16);
  uint8_t r[6] = { 0x41, (uint8_t)pid };
  int n = 2;
  auto put = [&](int v) { r[n++] = (uint8_t)v; };
  switch (pid) {
    case 0x00: put(0xBE); put(0x1F); put(0xB8); put(0x10); break;   // supported PIDs 01-20
    case 0x01: put(0x82); put(0x07); put(0x65); put(0x00); break;   // MIL on, 2 codes
    case 0x04: put((int)(car.load * 255 / 100)); break;
    case 0x05: put((int)((car.coolantF - 32) / 1.8f + 40)); break;
    case 0x06: put((int)(car.stft * 128 / 100 + 128)); break;
    case 0x0C: { int v = (int)(car.rpm * 4); put(v >> 8); put(v & 0xFF); } break;
    case 0x0D: put((int)(car.mph * 1.609f)); break;
    case 0x0E: return "NO DATA";                                      // like the real Focus
    case 0x0F: put((int)((car.iatF - 32) / 1.8f + 40)); break;
    case 0x10: { int v = (int)(car.maf * 100); put(v >> 8); put(v & 0xFF); } break;
    case 0x11: put((int)(car.throttle * 255 / 100)); break;
    default: return "NO DATA";
  }
  return hexBytes(r, n);
}

String atReply(String cmd)   // cmd: upper case, no spaces, without "AT"
{
  if (cmd == "Z" || cmd == "WS") { echo = true; spaces = true; linefeeds = false; headers = false; return "\r\rELM327 v1.5"; }
  if (cmd == "D") { echo = true; spaces = true; linefeeds = false; headers = false; return "OK"; }
  if (cmd == "I") return "ELM327 v1.5";
  if (cmd == "DP") return "SAE J1850 PWM";
  if (cmd == "DPN") return "1";
  if (cmd == "RV") { char b[8]; snprintf(b, sizeof(b), "%.1fV", 13.9f + 0.1f * sinf(millis() / 3000.0f)); return b; }
  if (cmd.startsWith("E")) { echo = cmd[1] == '1'; return "OK"; }
  if (cmd.startsWith("S") && cmd.length() == 2 && isdigit(cmd[1])) { spaces = cmd[1] == '1'; return "OK"; }
  if (cmd.startsWith("L") && cmd.length() == 2) { linefeeds = cmd[1] == '1'; return "OK"; }
  if (cmd.startsWith("H") && cmd.length() == 2) { headers = cmd[1] == '1'; return "OK"; }
  return "OK";   // AL, ST xx, SP x, TP x, AT x, M0, ...
}

void handle(String line)
{
  String req = line;
  req.toUpperCase();
  req.replace(" ", "");
  if (req.length() == 0) return;
  if (echo) bt.print(line + "\r");
  String reply = req.startsWith("AT") ? atReply(req.substring(2)) : obdReply(req);
  if (!req.startsWith("AT")) delay(REPLY_MS);        // J1850 replies take tens of ms (the real Focus: ~6 replies/s)
  const char* eol = linefeeds ? "\r\n" : "\r";
  bt.print(reply);
  bt.print(eol);
  bt.print(eol);
  bt.print(">");
  Serial.printf("%8lu  %-8s -> %s\n", millis(), req.c_str(), reply.c_str());
}

void setup()
{
  Serial.begin(115200);
  delay(200);
#if IMPERSONATE
  // Bluetooth address = base MAC + 2 on the ESP32
  uint8_t base[6];
  memcpy(base, ADAPTER_MAC, 6);
  base[5] -= 2;
  esp_base_mac_addr_set(base);
#endif
  bt.setPin("1234", 4);                 // legacy PIN pairing, like the Veepeak
  if (!bt.begin("OBDII", false, true)) Serial.println("Bluetooth failed to start");
  uint8_t mac[6];
  bt.getBtAddress(mac);
  Serial.printf("\n=== fake ELM327 'OBDII' at %02X:%02X:%02X:%02X:%02X:%02X, PIN 1234 ===\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("Type an RPM + Enter to hold it, or 'a' for automatic driving.");
}

void loop()
{
  static uint32_t last = millis();
  static String line, typed;
  uint32_t now = millis();
  car.step((now - last) / 1000.0f);
  last = now;

  while (bt.available()) {
    char c = bt.read();
    if (c == '\r' || c == '\n') { if (line.length()) handle(line); line = ""; }
    else if (line.length() < 40) line += c;
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      typed.trim();
      if (typed == "a") { car.holdRpm = -1; Serial.println("automatic driving"); }
      else if (typed.length()) { car.holdRpm = typed.toInt(); Serial.printf("holding %d rpm\n", car.holdRpm); }
      typed = "";
    } else typed += c;
  }

  static bool wasConnected = false;
  if (bt.hasClient() != wasConnected) {
    wasConnected = !wasConnected;
    Serial.println(wasConnected ? "*** dashboard connected" : "*** dashboard disconnected");
  }
  delay(2);
}
