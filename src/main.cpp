/* 
 * SynthController.cpp - Versi Joystick Pitchbend
 * Firmware dengan fungsi Pitchbend (Y-axis) dan Modulation (X-axis).
 * Fitur: USB MIDI, Hardware MIDI, Matrix, Analog, Encoder.
*/

#include <Arduino.h>

/* ---------- KONFIGURASI PIN ---------- */
// Matrix pins (8x8)
const uint8_t ROW_PINS[8] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t COL_PINS[8] = {PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14};

// Analog controls (PIN_PITCH_WHEEL dihapus)
const uint8_t PIN_PITCH_X = A0; // Akan digunakan untuk MODULATION (CC #1)
const uint8_t PIN_PITCH_Y = A1; // Akan digunakan untuk PITCH BEND
const uint8_t PIN_POT_BALANCE = A3;
const uint8_t PIN_POT_MASTER = A4;

// EQ sliders
const uint8_t PIN_EQ[3] = {A5, A6, A7}; 

// Switches
const uint8_t PIN_VELOCITY_ENABLE_SW = PC13;
const uint8_t PIN_CHORD_SCAN_SW = PC14;
const uint8_t PIN_TEMPO_ENABLE_SW = PC15;
const uint8_t PIN_TEMPO_MODE_SW = PB15;

// Rotary encoder
const uint8_t PIN_ENCODER_A = PB6;
const uint8_t PIN_ENCODER_B = PB7;
const uint8_t PIN_ENCODER_SW = PB8;

// MIDI output selection
#define MIDI_HARDWARE_SERIAL Serial1
#define MIDI_USB_SERIAL Serial

/* ---------- VARIABEL & PENGATURAN ---------- */
const unsigned long DEBOUNCE_MS = 8;
const uint16_t MIDI_CHANNEL = 1;
const uint8_t BASE_NOTE = 48; // C3

// Variabel untuk kalibrasi joystick
const int JOYSTICK_CENTER = 512; // Nilai tengah ADC (0-1023), sesuaikan jika perlu
const int JOYSTICK_DEADZONE = 20; // Zona mati di tengah agar tidak goyang

uint8_t padNoteMap[64];
bool chordScanEnabled = false;
bool tempoEnabled = false;
uint8_t tempoBeats = 4;
int currentOctaveOffset = 0;
int transpose = 0;

uint8_t matrixState[8][8] = {0};
unsigned long lastDebounce[8][8] = {0};

volatile int encoderPos = 0;
volatile bool encoderMoved = false;

unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 6;

// Variabel untuk menyimpan nilai analog terakhir, agar tidak mengirim data MIDI terus-menerus
int lastPitchY = -1;
int lastModX = -1;
int lastPan = -1;
int lastVol = -1;
int lastEQ[3] = {-1, -1, -1};

/* ---------- FUNGSI PENGIRIMAN MIDI (Tidak Berubah) ---------- */
void midiSendNoteOn(uint8_t ch, uint8_t note, uint8_t vel) { /* ... (sama seperti sebelumnya) ... */ }
void midiSendNoteOff(uint8_t ch, uint8_t note, uint8_t vel) { /* ... (sama seperti sebelumnya) ... */ }
void midiSendCC(uint8_t ch, uint8_t cc, uint8_t val) { /* ... (sama seperti sebelumnya) ... */ }
void midiSendPitchbend(uint8_t ch, int value14) { /* ... (sama seperti sebelumnya) ... */ }

// (Salin-tempel implementasi lengkap fungsi-fungsi di atas dari skrip sebelumnya)


/* ---------- FUNGSI PEMBANTU (readAnalogs Diubah) ---------- */
void setupPadMap() { /* ... (sama seperti sebelumnya) ... */ }
void scanMatrix() { /* ... (sama seperti sebelumnya) ... */ }
void encoderISR() { /* ... (sama seperti sebelumnya) ... */ }
void setupPins() { /* ... (sama seperti sebelumnya) ... */ }

// (Salin-tempel implementasi lengkap fungsi-fungsi di atas dari skrip sebelumnya)


// ===================================================================
// INI BAGIAN UTAMA YANG DIUBAH
// ===================================================================
void readAnalogs() {
  // --- Pitch Bend (Sumbu Y) ---
  int pitchY_raw = analogRead(PIN_PITCH_Y);
  
  // Periksa apakah ada perubahan signifikan
  if (abs(pitchY_raw - lastPitchY) > 2) { // Cek perubahan > 2 untuk stabilitas
    lastPitchY = pitchY_raw;
    
    // Map nilai 0-1023 ke rentang pitch bend MIDI (-8192 hingga +8191)
    // Nilai tengah (8192) adalah "tidak ada bend"
    int pitchBendValue = map(pitchY_raw, 0, 1023, 0, 16383);
    
    // Terapkan zona mati (deadzone) di tengah
    if (abs(pitchY_raw - JOYSTICK_CENTER) < JOYSTICK_DEADZONE) {
      pitchBendValue = 8192; // Paksa ke nilai tengah jika di dalam deadzone
    }
    
    midiSendPitchbend(MIDI_CHANNEL, pitchBendValue);
  }

  // --- Modulation Wheel (Sumbu X) ---
  int modX_raw = analogRead(PIN_PITCH_X);
  int modX_value = map(modX_raw, 0, 1023, 0, 127);
  
  // Kirim hanya jika nilainya berubah
  if (modX_value != lastModX) {
    lastModX = modX_value;
    midiSendCC(MIDI_CHANNEL, 1, modX_value); // CC #1 adalah Modulation Wheel
  }
  
  // --- Potensiometer Lain (Pan & Volume) ---
  int pan_value = map(analogRead(PIN_POT_BALANCE), 0, 1023, 0, 127);
  if (pan_value != lastPan) {
    lastPan = pan_value;
    midiSendCC(MIDI_CHANNEL, 10, pan_value); // CC 10 for Pan
  }
  
  int vol_value = map(analogRead(PIN_POT_MASTER), 0, 1023, 0, 127);
  if (vol_value != lastVol) {
    lastVol = vol_value;
    midiSendCC(MIDI_CHANNEL, 7, vol_value); // CC 7 for Master Volume
  }

  // --- EQ Sliders ---
  for (int i = 0; i < 3; i++) {
    int eq_value = map(analogRead(PIN_EQ[i]), 0, 1023, 0, 127);
    if (eq_value != lastEQ[i]) {
      lastEQ[i] = eq_value;
      midiSendCC(MIDI_CHANNEL, 70 + i, eq_value); // CC 70, 71, 72 for EQ
    }
  }
}


/* ---------- FUNGSI UTAMA (Tidak Berubah) ---------- */
void setup() {
  // ... (sama seperti skrip sebelumnya)
}

void loop() {
  // ... (sama seperti skrip sebelumnya)
}

// (Salin-tempel implementasi lengkap fungsi setup() dan loop() dari skrip sebelumnya)
