// ESP32 OBD-II RPM reader over a Bluetooth ELM327 adapter
// Step 2 of the car display project (input side).
//
// Board:   original ESP32 (WROOM-32 / WROOM-DA). Needs Bluetooth Classic,
//          so ESP32-S3/C3 won't work with this sketch.
// Car:     2000 Ford Focus -> SAE J1850 PWM (41.6 kbaud).
// Adapter: must support J1850 PWM and Bluetooth Classic (SPP).
//
// Open Serial Monitor at 115200 to see RPM.

#include "BluetoothSerial.h"
#include "ELMduino.h"

// ---- settings to tweak -------------------------------------------------
// Bluetooth name of the adapter as it shows up on your phone.
// Common names: "OBDII", "OBDLink LX", "V-LINK", "Android-Vlink"
#define ADAPTER_NAME  "OBDII"
#define ADAPTER_PIN   "1234"   // most adapters use 1234 or 0000

// '1' = SAE J1850 PWM (2000 Focus). Use '0' to let the adapter auto-detect.
#define OBD_PROTOCOL  SAE_J1850_PWM_41_KBAUD
// ------------------------------------------------------------------------

BluetoothSerial SerialBT;
ELM327 elm;

uint32_t rpm = 0;

bool connectAdapter()
{
  Serial.printf("Connecting to Bluetooth adapter \"%s\"...\n", ADAPTER_NAME);
  if (!SerialBT.connect(ADAPTER_NAME)) {
    Serial.println("  Bluetooth connect failed (is the adapter plugged in and powered?)");
    return false;
  }
  Serial.println("  Bluetooth connected. Initializing ELM327...");

  // debug=true prints the raw ELM327 traffic, handy while learning
  if (!elm.begin(SerialBT, true, 2000, OBD_PROTOCOL)) {
    Serial.println("  ELM327 init failed (is the ignition ON?)");
    SerialBT.disconnect();
    return false;
  }
  Serial.println("  Connected to car.");
  return true;
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32-RPM", true);  // true = we are the master
  SerialBT.setPin(ADAPTER_PIN, strlen(ADAPTER_PIN));

  while (!connectAdapter()) {
    delay(3000);
  }
}

void loop()
{
  if (!SerialBT.connected()) {
    Serial.println("Lost connection, retrying...");
    while (!connectAdapter()) {
      delay(3000);
    }
  }

  // Non-blocking: call repeatedly until the reply arrives
  float reading = elm.rpm();

  if (elm.nb_rx_state == ELM_SUCCESS) {
    rpm = (uint32_t)reading;
    Serial.printf("RPM: %lu\n", rpm);
  } else if (elm.nb_rx_state != ELM_GETTING_MSG) {
    elm.printError();
  }
}
