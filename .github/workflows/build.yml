/* 
 * SynthController.cpp - Versi 1.3.1 (Lengkap)
 * Firmware Final dengan Multi-Interface MIDI Controller
 * Fitur: USB MIDI, Hardware MIDI (DIN-5), dan BLE MIDI (via HM-10)
 * Penulis: Assistant & lekpar112-a11y
 * Target: STM32F103C8 (Blue Pill)
*/

#include <Arduino.h>
#include <BLEMidi.h> // Library untuk Bluetooth Low Energy MIDI

// Definisikan port Serial untuk setiap output
#define HW_MIDI_SERIAL  Serial1 // Untuk MIDI DIN-5
#define BLE_MIDI_SERIAL Serial2 // Untuk modul HM-10
#define USB_MIDI_SERIAL Serial  // Untuk USB MIDI

/* ---------- KONFIGURASI PIN ---------- */
// Matrix pins
const uint8_t ROW_PINS[8] = {PA4, PA5, PA6, PA7, PB12, PB13, PB14, PB15};
const uint8_t COL_PINS[8] = {PC13, PC14, PC15, PA8, PA11, PA12, PB3, PB4};

// Analog pins
const uint8_t PIN_PITCH_X     = PA0; // A0
const uint8_t PIN_PITCH_Y     = PA1; // A1

// Pin yang dipindahkan untuk memberi ruang bagi BLE_MIDI_SERIAL (PA2, PA3)
const uint8_t PIN_PITCH_WHEEL = PB0; // Dipindahkan dari A2
const uint8_t PIN_POT_BALANCE = PB1; // Dipindahkan dari A3
const uint8_t PIN_POT_MASTER  = PB10;

// EQ sliders
const uint8_t PIN_EQ[3] = {PA5, PA6, PA7}; // Menggunakan pin A5, A6, A7

// Switches
const uint8_t PIN_VELOCITY_ENABLE_SW = PB5;
const uint8_t PIN_CHORD_SCAN_SW    = PB10; // Shared with a pot, might need adjustment
const uint8_t PIN_TEMPO_ENABLE_SW  = PC13; // Shared with matrix, physical button might be better
const uint8_t PIN_TEMPO_MODE_SW    = PC14; // Shared with matrix

// Rotary encoder (di pin yang aman dari konflik)
const uint8_t PIN_ENCODER_A  = PB6;
const uint8_t PIN_ENCODER_B  = PB7;
const uint8_t PIN_ENCODER_SW = PB8;

/* ---------- KONFIGURASI MIDI & STATE ---------- */
const unsigned long DEBOUNCE_MS = 8;
const uint16_t MIDI_CHANNEL = 1; 
const uint8_t BASE_NOTE = 48; // C3

uint8_t padNoteMap[64];
bool chordScanEnabled = false;
bool tempoEnabled = false;
uint8_t tempoBeats = 4;
int currentOctaveOffset = 0;
int transpose = 0;

uint8_t matrixState[8][8];
unsigned long lastDebounce[8][8];

volatile int encoderPos = 0;
volatile bool encoderMoved = false;

unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 5; 

void sendMidiMessage(uint8_t* message, uint8_t size) {
    HW_MIDI_SERIAL.write(message, size);
    #if defined(USBCON)
        USB_MIDI_SERIAL.write(message, size);
    #endif
    if (BLE.isConnected()) {
        BLE.write(message, size);
    }
}

void midiSendNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
    uint8_t msg[] = {0x90 | ((ch-1) & 0x0F), note, vel};
    sendMidiMessage(msg, sizeof(msg));
}

void midiSendNoteOff(uint8_t ch, uint8_t note, uint8_t vel) {
    uint8_t msg[] = {0x80 | ((ch-1) & 0x0F), note, vel};
    sendMidiMessage(msg, sizeof(msg));
}

void midiSendCC(uint8_t ch, uint8_t cc, uint8_t val) {
    uint8_t msg[] = {0xB0 | ((ch-1) & 0x0F), cc, val};
    sendMidiMessage(msg, sizeof(msg));
}

void midiSendPitchbend(uint8_t ch, int value14) {
    if (value14 < 0) value14 = 0;
    if (value14 > 16383) value14 = 16383;
    uint8_t lsb = value14 & 0x7F;
    uint8_t msb = (value14 >> 7) & 0x7F;
    uint8_t msg[] = {0xE0 | ((ch-1) & 0x0F), lsb, msb};
    sendMidiMessage(msg, sizeof(msg));
}

void setupPadMap() {
  for (int i=0; i<64; ++i) {
    padNoteMap[i] = BASE_NOTE + i;
  }
}

void scanMatrix() {
  for (int c=0; c<8; ++c) {
    pinMode(COL_PINS[c], OUTPUT);
    digitalWrite(COL_PINS[c], LOW);
    for (int r=0; r<8; ++r) {
      pinMode(ROW_PINS[r], INPUT_PULLUP);
      bool pressed = (digitalRead(ROW_PINS[r]) == LOW);
      if (pressed != matrixState[r][c]) {
        if (millis() - lastDebounce[r][c] > DEBOUNCE_MS) {
          lastDebounce[r][c] = millis();
          matrixState[r][c] = pressed;
          uint8_t note = padNoteMap[r*8 + c] + (currentOctaveOffset*12) + transpose;
          if (pressed) {
            midiSendNoteOn(MIDI_CHANNEL, note, 100);
          } else {
            midiSendNoteOff(MIDI_CHANNEL, note, 0);
          }
        }
      }
    }
    pinMode(COL_PINS[c], INPUT); // High-impedance
  }
}

void readAnalogs() {
  int pbx = map(analogRead(PIN_PITCH_X), 0, 1023, 0, 16383);
  midiSendPitchbend(MIDI_CHANNEL, pbx);

  int vol = map(analogRead(PIN_POT_MASTER), 0, 1023, 0, 127);
  midiSendCC(MIDI_CHANNEL, 7, vol);

  int pan = map(analogRead(PIN_POT_BALANCE), 0, 1023, 0, 127);
  midiSendCC(MIDI_CHANNEL, 10, pan);

  for(int i=0; i<3; ++i) {
    int eq_val = map(analogRead(PIN_EQ[i]), 0, 1023, 0, 127);
    midiSendCC(MIDI_CHANNEL, 20+i, eq_val);
  }
}

void encoderISR() {
  static uint8_t last = 0;
  uint8_t s = (digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B);
  if (s == last) return;
  if ((last == 0b00 && s == 0b01) || (last == 0b01 && s == 0b11) || (last == 0b11 && s == 0b10) || (last == 0b10 && s == 0b00)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
  last = s;
  encoderMoved = true;
}

void setupPins() {
  for(int i=0; i<8; ++i) {
    pinMode(ROW_PINS[i], INPUT_PULLUP);
    pinMode(COL_PINS[i], INPUT);
  }
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Multi-Interface MIDI Controller v1.3.1...");

    HW_MIDI_SERIAL.begin(31250);
    BLE_MIDI_SERIAL.begin(9600);
    
    BLE.begin("STM32 BLE MIDI", BLE_MIDI_SERIAL);
    Serial.println("BLE MIDI Server Started.");

    setupPins();
    setupPadMap();

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
}

void loop() {
    if (BLE.isConnected()) {
      BLE.poll();
    }
    
    unsigned long now = millis();
    if (now - lastScanTime > SCAN_INTERVAL) {
        lastScanTime = now;
        scanMatrix();
        readAnalogs();
    }

    if (encoderMoved) {
        noInterrupts();
        int pos = encoderPos;
        encoderPos = 0;
        encoderMoved = false;
        interrupts();

        if (digitalRead(PIN_ENCODER_SW) == LOW) { // Change octave
          currentOctaveOffset += (pos > 0) ? 1 : -1;
          if (currentOctaveOffset > 2) currentOctaveOffset = 2;
          if (currentOctaveOffset < -2) currentOctaveOffset = -2;
        } else { // Change transpose
          transpose += (pos > 0) ? 1 : -1;
          if (transpose > 12) transpose = 12;
          if (transpose < -12) transpose = -12;
        }
    }
}
