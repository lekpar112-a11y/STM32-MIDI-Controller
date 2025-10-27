#include <Arduino.h>
#include <BLEMIDI_Transport.h>

// Definisikan port Serial
#define HW_MIDI_SERIAL  Serial1
#define BLE_MIDI_SERIAL Serial2
#define USB_MIDI_SERIAL Serial

// Konfigurasi Pin
const uint8_t ROW_PINS[8] = {PA4, PA5, PA6, PA7, PB12, PB13, PB14, PB15};
const uint8_t COL_PINS[8] = {PC13, PC14, PC15, PA8, PA11, PA12, PB3, PB4};
const uint8_t PIN_PITCH_X     = PA0; 
const uint8_t PIN_PITCH_Y     = PA1; 
const uint8_t PIN_PITCH_WHEEL = PB0; 
const uint8_t PIN_POT_BALANCE = PB1; 
const uint8_t PIN_POT_MASTER  = PB10;
const uint8_t PIN_EQ[3] = {PA5, PA6, PA7};
const uint8_t PIN_ENCODER_A  = PB6;
const uint8_t PIN_ENCODER_B  = PB7;
const uint8_t PIN_ENCODER_SW = PB8;

// Variabel
const uint16_t MIDI_CHANNEL = 1; 
const uint8_t BASE_NOTE = 48; 
uint8_t padNoteMap[64];
int currentOctaveOffset = 0;
int transpose = 0;
uint8_t matrixState[8][8] = {0};
unsigned long lastDebounce[8][8] = {0};
volatile bool encoderMoved = false;
unsigned long lastScanTime = 0;

// Fungsi Pengiriman MIDI
// ... (semua fungsi midiSendNoteOn, dll.)

void setup() {
    // ...
}

void loop() {
    // ...
}
