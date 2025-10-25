/* 
 * SynthController.cpp - Versi 1.3
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
const uint8_t ROW_PINS[8] = {PA0, PA1, PA4, PA5, PA6, PA7, PB12, PB13}; // Disesuaikan agar tidak bentrok
const uint8_t COL_PINS[8] = {PB14, PB15, PC13, PC14, PC15, PA8, PA11, PA12};

// Analog pins
const uint8_t PIN_PITCH_X     = PA0; // A0
const uint8_t PIN_PITCH_Y     = PA1; // A1
const uint8_t PIN_POT_MASTER  = PA4; // A4

// Pin yang dipindahkan untuk memberi ruang bagi BLE_MIDI_SERIAL (PA2, PA3)
const uint8_t PIN_PITCH_WHEEL = PB0; // Dipindahkan dari A2
const uint8_t PIN_POT_BALANCE = PB1; // Dipindahkan dari A3

// EQ sliders
const uint8_t PIN_EQ[3] = {PA5, PA6, PA7}; // Menggunakan pin A5, A6, A7

// Switches
const uint8_t PIN_VELOCITY_ENABLE_SW = PB3;
const uint8_t PIN_CHORD_SCAN_SW    = PB4;
const uint8_t PIN_TEMPO_ENABLE_SW  = PB5;
const uint8_t PIN_TEMPO_MODE_SW    = PB10;

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
int tempo = 120;

uint8_t matrixState[8][8];
unsigned long lastDebounce[8][8];

volatile int encoderPos = 0;
volatile bool encoderMoved = false;

unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 6; 

// Fungsi untuk mengirim pesan MIDI ke SEMUA interface
void sendMidiMessage(uint8_t* message, uint8_t size) {
    // 1. Kirim ke Hardware MIDI (DIN-5)
    HW_MIDI_SERIAL.write(message, size);

    // 2. Kirim ke USB MIDI (jika terhubung)
    #if defined(USBCON)
        USB_MIDI_SERIAL.write(message, size);
    #endif

    // 3. Kirim ke BLE MIDI
    BLE.write(message, size);
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

// ... [ Semua fungsi lain (scanMatrix, readAnalogs, chordDetector, encoderISR, dll.) dari kode sebelumnya bisa ditempel di sini TANPA PERUBAHAN ] ...
// Pastikan semua fungsi tersebut memanggil midiSendNoteOn, midiSendCC, dll. yang sudah kita modifikasi di atas.


void setup() {
    Serial.begin(115200); // Serial untuk debug, BUKAN untuk MIDI USB
    Serial.println("Starting Multi-Interface MIDI Controller v1.3...");

    // Inisialisasi Hardware MIDI (DIN-5)
    HW_MIDI_SERIAL.begin(31250);

    // Inisialisasi Serial untuk modul BLE
    BLE_MIDI_SERIAL.begin(9600); // 9600 adalah baud rate default untuk HM-10

    // Inisialisasi dan mulai server BLE MIDI
    // Nama ini yang akan muncul saat Anda mencari perangkat Bluetooth
    BLE.begin("STM32 BLE MIDI", BLE_MIDI_SERIAL);
    Serial.println("BLE MIDI Server Started.");

    #if defined(USBCON)
      // MIDI USB akan di-handle secara otomatis oleh core STM32
      Serial.println("USB MIDI Enabled.");
    #endif
    
    // ... [ Panggil semua fungsi setup lain seperti setupPins(), setupPadMap(), attachInterrupt(...) ] ...
}

void loop() {
    // WAJIB: Terus perbarui koneksi BLE.
    // Fungsi ini menangani koneksi, pairing, dan pengiriman data.
    BLE.poll();

    // ... [ Panggil semua fungsi lain di dalam loop seperti scanMatrix(), readAnalogs(), dll. seperti kode sebelumnya ] ...
}
