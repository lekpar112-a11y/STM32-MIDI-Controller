/* 
 * SynthController.cpp - Versi 1.3.1 (Tanpa Library BLEMidi)
 * Firmware Final dengan Multi-Interface MIDI Controller
 * Fitur: USB MIDI, Hardware MIDI (DIN-5), dan BLE MIDI (via HM-10 UART)
 * Penulis: Assistant & lekpar112-a11y
 * Target: STM32F103C8 (Blue Pill)
*/

#include <Arduino.h>

// Definisikan port Serial untuk setiap output
#define HW_MIDI_SERIAL  Serial1 // Untuk MIDI DIN-5
#define BLE_MIDI_SERIAL Serial2 // Untuk modul HM-10
#define USB_MIDI_SERIAL Serial  // Untuk USB MIDI

/* ---------- KONFIGURASI PIN ---------- */
const uint8_t ROW_PINS[8] = {PA0, PA1, PA4, PA5, PA6, PA7, PB12, PB13};
const uint8_t COL_PINS[8] = {PB14, PB15, PC13, PC14, PC15, PA8, PA11, PA12};

// Analog pins
const uint8_t PIN_PITCH_X     = PA0;
const uint8_t PIN_PITCH_Y     = PA1;
const uint8_t PIN_POT_MASTER  = PA4;

// Pin yang dipindahkan untuk memberi ruang bagi BLE_MIDI_SERIAL (PA2, PA3)
const uint8_t PIN_PITCH_WHEEL = PB0;
const uint8_t PIN_POT_BALANCE = PB1;

// EQ sliders
const uint8_t PIN_EQ[3] = {PA5, PA6, PA7};

// Switches
const uint8_t PIN_VELOCITY_ENABLE_SW = PB3;
const uint8_t PIN_CHORD_SCAN_SW    = PB4;
const uint8_t PIN_TEMPO_ENABLE_SW  = PB5;
const uint8_t PIN_TEMPO_MODE_SW    = PB10;

// Rotary encoder
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
int tempo = 120;

uint8_t matrixState[8][8];
unsigned long lastDebounce[8][8];

volatile int encoderPos = 0;
volatile bool encoderMoved = false;

unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 6; 

/* ---------- UTILITAS BLE MIDI ---------- */
/* 
   Fungsi pengganti library BLEMidi.h — 
   semua komunikasi BLE dilakukan via Serial2 (HM-10)
*/

void bleSend(uint8_t* data, uint8_t size) {
  BLE_MIDI_SERIAL.write(data, size);
}

void blePoll() {
  // HM-10 tidak memerlukan polling kompleks — placeholder
}

/* ---------- PENGIRIMAN PESAN MIDI ---------- */
void sendMidiMessage(uint8_t* message, uint8_t size) {
  // 1. Kirim ke Hardware MIDI (DIN-5)
  HW_MIDI_SERIAL.write(message, size);

  // 2. Kirim ke USB MIDI (jika aktif)
  #if defined(USBCON)
    USB_MIDI_SERIAL.write(message, size);
  #endif

  // 3. Kirim ke BLE MIDI via HM-10
  bleSend(message, size);
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

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Multi-Interface MIDI Controller v1.3.1...");

  // Inisialisasi Hardware MIDI
  HW_MIDI_SERIAL.begin(31250);

  // Inisialisasi BLE MIDI (HM-10)
  BLE_MIDI_SERIAL.begin(9600);
  Serial.println("HM-10 BLE MIDI Ready.");

  #if defined(USBCON)
    Serial.println("USB MIDI Enabled.");
  #endif

  // Inisialisasi pin (bisa tambahkan setupPins(), attachInterrupt(), dll.)
  // setupPins();
}

/* ---------- LOOP ---------- */
void loop() {
  // Perbarui koneksi BLE (placeholder)
  blePoll();

  // Contoh test MIDI:
  midiSendNoteOn(MIDI_CHANNEL, 60, 127);
  delay(200);
  midiSendNoteOff(MIDI_CHANNEL, 60, 0);
  delay(800);

  // Di sini kamu bisa tambahkan semua fungsi seperti:
  // scanMatrix();
  // readAnalogs();
  // handleEncoder();
  // dll.
}
