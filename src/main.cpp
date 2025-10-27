#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <bluefruit.h>

// Ganti Serial2 dengan port yang Anda gunakan untuk modul BLE
Adafruit_BluefruitLE_UART ble(Serial2);

// (Fungsi-fungsi pembantu Anda seperti scanMatrix, dll. perlu diadaptasi)

void sendMidiBle(uint8_t *msg, uint8_t len) {
    if (ble.isConnected()) {
        ble.write(msg, len);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Adafruit BLE MIDI Controller");

    ble.begin(true);
    ble.setMode(BLUEFRUIT_MODE_DATA);
    ble.factoryReset(); // Opsional, untuk memastikan bersih
    ble.setDeviceName("STM32-BLE-MIDI");
}

void loop() {
    // Contoh: Kirim not C4 setiap detik
    uint8_t noteOn[] = {0x90, 60, 127};
    sendMidiBle(noteOn, sizeof(noteOn));
    delay(500);
    uint8_t noteOff[] = {0x80, 60, 0};
    sendMidiBle(noteOff, sizeof(noteOff));
    delay(500);
}
