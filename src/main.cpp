#include <Arduino.h>
#include <USBComposite.h>
#include <USBMIDI.h>
#include <MIDI.h>

// =========================
// USB MIDI
// =========================
USBMIDI usbMIDI;
MIDI_CREATE_INSTANCE(USBMIDI, usbMIDI, MIDI_USB);

// =========================
// UART MIDI (STM32 <-> ESP32)
// =========================
// Catatan penting: Pada STM32 Arduino core,
// USART1 = Serial2  (INI YANG DIPAKAI)
HardwareSerial &SerialMIDI_HW = Serial2;

// =========================
// CD4067 MUX
// =========================
// Pin Selector S0-S3
const uint8_t MUX_S0 = PA0;
const uint8_t MUX_S1 = PA1;
const uint8_t MUX_S2 = PA2;
const uint8_t MUX_S3 = PA3;

// Analog input MUX
const uint8_t MUX_SIGNAL = PA4;

// =========================
// Velocity Control
// =========================
const uint8_t VELOCITY_POT = PA5;          // analog pot
const uint8_t VELOCITY_TOGGLE_PIN = PA6;   // ON/OFF button

bool velocityEnabled = true;
uint8_t globalVelocity = 100;

// =========================
// MATRIX KEYS (contoh)
// =========================
const uint8_t ROWS = 8;
const uint8_t COLS = 8;

uint8_t rowPins[ROWS] = {PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15};
uint8_t colPins[COLS] = {PA7, PA8, PA9, PA10, PA11, PA12, PA15, PB3};

// =========================
// Setup
// =========================
void setup() {

    // UART MIDI (STM32 <-> ESP32)
    SerialMIDI_HW.begin(115200);

    // USB MIDI
    USBComposite.clear();
    MIDI_USB.begin();
    USBComposite.begin();

    // MUX pins
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);
    pinMode(MUX_SIGNAL, INPUT_ANALOG);

    // Velocity
    pinMode(VELOCITY_TOGGLE_PIN, INPUT_PULLUP);

    // Matrix
    for (int i = 0; i < ROWS; i++) pinMode(rowPins[i], OUTPUT);
    for (int i = 0; i < COLS; i++) pinMode(colPins[i], INPUT_PULLUP);
}

// =========================
// BACA MUX CD4067
// =========================
int readMux(uint8_t ch) {
    digitalWrite(MUX_S0, (ch & 1) ? HIGH : LOW);
    digitalWrite(MUX_S1, (ch & 2) ? HIGH : LOW);
    digitalWrite(MUX_S2, (ch & 4) ? HIGH : LOW);
    digitalWrite(MUX_S3, (ch & 8) ? HIGH : LOW);
    delayMicroseconds(5);
    return analogRead(MUX_SIGNAL);
}

// =========================
// LOOP
// =========================
void loop() {

    // ==========================
    // Velocity Toggle Button
    // ==========================
    static bool lastBtn = HIGH;
    bool btn = digitalRead(VELOCITY_TOGGLE_PIN);

    if (btn == LOW && lastBtn == HIGH) {
        velocityEnabled = !velocityEnabled;
        delay(200);
    }
    lastBtn = btn;

    // ==========================
    // Baca Velocity Pot
    // ==========================
    if (velocityEnabled) {
        int v = analogRead(VELOCITY_POT);
        globalVelocity = map(v, 0, 4095, 1, 127);
    } else {
        globalVelocity = 100; // default
    }

    // ==========================
    // SEND UART TO USB MIDI
    // ==========================
    while (SerialMIDI_HW.available()) {
        uint8_t b = SerialMIDI_HW.read();
        MIDI_USB.sendRealTime(b);
    }

    // ==========================
    // SEND USB MIDI TO UART
    // (Receive from DAW/Phone)
    // ==========================
    if (MIDI_USB.read()) {
        uint8_t type = MIDI_USB.getType();
        uint8_t d1 = MIDI_USB.getData1();
        uint8_t d2 = MIDI_USB.getData2();

        SerialMIDI_HW.write(type);
        SerialMIDI_HW.write(d1);
        SerialMIDI_HW.write(d2);
    }
}
