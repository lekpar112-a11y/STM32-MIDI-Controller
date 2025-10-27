#include <Arduino.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"

Adafruit_BluefruitLE_UART ble(Serial2);

void setup() {
  Serial.begin(115200);
  ble.begin(true);
  ble.setMode(BLUEFRUIT_MODE_DATA);
  ble.setDeviceName("STM32-BLE-MIDI");
}

void loop() {
  // Kirim pesan Note On untuk C4
  uint8_t noteOn[] = {0x90, 60, 127};
  if (ble.isConnected()) {
    ble.write(noteOn, sizeof(noteOn));
  }
  delay(1000);

  // Kirim pesan Note Off untuk C4
  uint8_t noteOff[] = {0x80, 60, 0};
  if (ble.isConnected()) {
    ble.write(noteOff, sizeof(noteOff));
  }
  delay(1000);
}
