// DAC / wiring check for the composite video pin.
// GPIO25 steps through steady voltages, 3 s each, so a multimeter (DC volts,
// black probe on GND) can confirm the pin, the solder joint and the RCA wire.
//   0.0 V -> 1.0 V -> 2.0 V -> 3.1 V -> repeat
// Measure at the ESP32 pin first, then at the RCA plug's centre tip.
// (With the TV plugged in, its 75 ohm input pulls the voltage down a lot;
//  unplug the RCA from the TV for this test.)
void setup()
{
  Serial.begin(115200);
}

void loop()
{
  const uint8_t levels[] = { 0, 77, 155, 240 };
  const float volts[] = { 0.0, 1.0, 2.0, 3.1 };
  for (int i = 0; i < 4; i++) {
    dacWrite(25, levels[i]);
    Serial.printf("GPIO25 should read about %.1f V now\n", volts[i]);
    delay(3000);
  }
}
