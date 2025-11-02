/* 
 * SynthController.cpp - Versi Final Lengkap
 * Fitur: USB/Hardware MIDI, Matriks, Analog, Encoder dengan 4 Mode Siklus
 * (Tempo, Oktaf, Transpose, Set Pitch Bend).
*/

#include <Arduino.h>

/* ---------- KONFIGURASI PIN ---------- */
// Matrix pins (8x8)
const uint8_t ROW_PINS[8] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t COL_PINS[8] = {PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14};

// Analog controls
const uint8_t PIN_PITCH_X = A0; // Modulation (CC #1)
const uint8_t PIN_PITCH_Y = A1; // Pitch Bend
const uint8_t PIN_POT_BALANCE = A3;
const uint8_t PIN_POT_MASTER = A4;

// EQ sliders
const uint8_t PIN_EQ[3] = {A5, A6, A7}; 

// Switches
const uint8_t PIN_CHORD_SCAN_SW = PC14;
const uint8_t PIN_TEMPO_ENABLE_SW = PC15;
const uint8_t PIN_TEMPO_MODE_SW = PB15;

// Rotary encoder
const uint8_t PIN_ENCODER_A = PB6;
const uint8_t PIN_ENCODER_B = PB7;
const uint8_t PIN_ENCODER_SW = PB8;

// LED Indikator (menggunakan LED onboard)
const int LED_PIN = PC13;

// MIDI output selection
#define MIDI_HARDWARE_SERIAL Serial1
#define MIDI_USB_SERIAL Serial

/* ---------- VARIABEL & PENGATURAN ---------- */
const uint16_t MIDI_CHANNEL = 1;
const uint8_t BASE_NOTE = 48; // C3

const int JOYSTICK_CENTER = 512;
const int JOYSTICK_DEADZONE = 20;

uint8_t padNoteMap[64];
uint8_t matrixState[8][8] = {0};
unsigned long lastDebounce[8][8] = {0};
const unsigned long DEBOUNCE_MS = 8;

unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 5;

// Variabel untuk siklus mode encoder
enum EncoderMode { MODE_TEMPO, MODE_OKTAF, MODE_TRANSPOSE, MODE_PITCHBEND_SET };
EncoderMode currentEncoderMode = MODE_TEMPO;
bool lastEncoderButtonState = HIGH;
unsigned long lastEncoderButtonDebounce = 0;
const long ENCODER_DEBOUNCE_DELAY = 50;

// Variabel nilai yang bisa diubah
int tempo = 120;
int currentOctaveOffset = 0;
int transpose = 0;
byte pitchBendRange = 2; // Default +/- 2 semitones

volatile int encoderPos = 0;
volatile bool encoderMoved = false;

// Variabel untuk optimalisasi pengiriman data analog
int lastPitchY = -1;
int lastModX = -1;
int lastPan = -1;
int lastVol = -1;
int lastEQ[3] = {-1, -1, -1};

/* ---------- FUNGSI PENGIRIMAN MIDI ---------- */
void midiSend(uint8_t status, uint8_t data1, uint8_t data2) {
  MIDI_HARDWARE_SERIAL.write(status);
  MIDI_HARDWARE_SERIAL.write(data1);
  MIDI_HARDWARE_SERIAL.write(data2);
  #if defined(USBCON)
    MIDI_USB_SERIAL.write(status);
    MIDI_USB_SERIAL.write(data1);
    MIDI_USB_SERIAL.write(data2);
  #endif
}

void midiSendNoteOn(uint8_t ch, uint8_t note, uint8_t vel) { midiSend(0x90 | ((ch - 1) & 0x0F), note, vel); }
void midiSendNoteOff(uint8_t ch, uint8_t note, uint8_t vel) { midiSend(0x80 | ((ch - 1) & 0x0F), note, vel); }
void midiSendCC(uint8_t ch, uint8_t cc, uint8_t val) { midiSend(0xB0 | ((ch - 1) & 0x0F), cc, val); }
void midiSendPitchbend(uint8_t ch, int value14) {
  if (value14 < 0) value14 = 0;
  if (value14 > 16383) value14 = 16383;
  uint8_t lsb = value14 & 0x7F;
  uint8_t msb = (value14 >> 7) & 0x7F;
  midiSend(0xE0 | ((ch - 1) & 0x0F), lsb, msb);
}

// Fungsi untuk mengirim RPN untuk mengatur Pitch Bend Range
void setPitchBendRange(uint8_t ch, byte range) {
  midiSendCC(ch, 101, 0); // RPN MSB
  midiSendCC(ch, 100, 0); // RPN LSB
  midiSendCC(ch, 6, range);   // Data Entry MSB
  midiSendCC(ch, 38, 0);      // Data Entry LSB
  midiSendCC(ch, 101, 127); // Batalkan RPN
  midiSendCC(ch, 100, 127);
}

/* ---------- FUNGSI-FUNGSI PEMBANTU ---------- */
void setupPadMap() {
  for (int i = 0; i < 64; i++) { padNoteMap[i] = BASE_NOTE + i; }
}

void scanMatrix() {
  for (int c = 0; c < 8; c++) {
    pinMode(COL_PINS[c], OUTPUT);
    digitalWrite(COL_PINS[c], LOW);
    delayMicroseconds(10);
    for (int r = 0; r < 8; r++) {
      pinMode(ROW_PINS[r], INPUT_PULLUP);
      bool pressed = (digitalRead(ROW_PINS[r]) == LOW);
      if (pressed != matrixState[r][c] && (millis() - lastDebounce[r][c] > DEBOUNCE_MS)) {
        lastDebounce[r][c] = millis();
        matrixState[r][c] = pressed;
        uint8_t note = padNoteMap[r * 8 + c] + (currentOctaveOffset * 12) + transpose;
        if (pressed) { midiSendNoteOn(MIDI_CHANNEL, note, 100); } 
        else { midiSendNoteOff(MIDI_CHANNEL, note, 0); }
      }
    }
    pinMode(COL_PINS[c], INPUT);
  }
}

void readAnalogs() {
  // Pitch Bend (Sumbu Y)
  int pitchY_raw = analogRead(PIN_PITCH_Y);
  if (abs(pitchY_raw - lastPitchY) > 2) {
    lastPitchY = pitchY_raw;
    int pitchBendValue = map(pitchY_raw, 0, 1023, 0, 16383);
    if (abs(pitchY_raw - JOYSTICK_CENTER) < JOYSTICK_DEADZONE) { pitchBendValue = 8192; }
    midiSendPitchbend(MIDI_CHANNEL, pitchBendValue);
  }

  // Modulation (Sumbu X)
  int modX_value = map(analogRead(PIN_PITCH_X), 0, 1023, 0, 127);
  if (modX_value != lastModX) { lastModX = modX_value; midiSendCC(MIDI_CHANNEL, 1, modX_value); }
  
  // Pots
  int pan_value = map(analogRead(PIN_POT_BALANCE), 0, 1023, 0, 127);
  if (pan_value != lastPan) { lastPan = pan_value; midiSendCC(MIDI_CHANNEL, 10, pan_value); }
  int vol_value = map(analogRead(PIN_POT_MASTER), 0, 1023, 0, 127);
  if (vol_value != lastVol) { lastVol = vol_value; midiSendCC(MIDI_CHANNEL, 7, vol_value); }

  // EQ Sliders
  for (int i = 0; i < 3; i++) {
    int eq_value = map(analogRead(PIN_EQ[i]), 0, 1023, 0, 127);
    if (eq_value != lastEQ[i]) { lastEQ[i] = eq_value; midiSendCC(MIDI_CHANNEL, 70 + i, eq_value); }
  }
}

void encoderISR() {
  static uint8_t last = 0;
  uint8_t s = (digitalRead(PIN_ENCODER_A) << 1) | digitalRead(PIN_ENCODER_B);
  if (s == last) return;
  if ((last==0b00&&s==0b01)||(last==0b01&&s==0b11)||(last==0b11&&s==0b10)||(last==0b10&&s==0b00)) { encoderPos++; } else { encoderPos--; }
  encoderMoved = true;
  last = s;
}

void setupPins() {
  for (int i = 0; i < 8; i++) { pinMode(ROW_PINS[i], INPUT_PULLUP); pinMode(COL_PINS[i], INPUT); }
  pinMode(PIN_CHORD_SCAN_SW, INPUT_PULLUP);
  pinMode(PIN_TEMPO_ENABLE_SW, INPUT_PULLUP);
  pinMode(PIN_TEMPO_MODE_SW, INPUT_PULLUP);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

/* ---------- FUNGSI UTAMA ---------- */
void setup() {
  MIDI_USB_SERIAL.begin(115200);
  MIDI_HARDWARE_SERIAL.begin(31250);
  setupPadMap();
  setupPins();
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
  Serial.println("SynthController Final Version Started");
  setPitchBendRange(MIDI_CHANNEL, pitchBendRange); // Set default pitch bend range at startup
}

void loop() {
  if (millis() - lastScanTime > SCAN_INTERVAL) {
    lastScanTime = millis();
    scanMatrix();
    readAnalogs();
  }

  // Mendeteksi Penekanan Tombol Encoder
  bool currentEncoderButtonState = (digitalRead(PIN_ENCODER_SW) == LOW);
  if (currentEncoderButtonState != lastEncoderButtonState && (millis() - lastEncoderButtonDebounce > ENCODER_DEBOUNCE_DELAY)) {
    if (currentEncoderButtonState == LOW) { // Tombol baru saja ditekan
      if (currentEncoderMode == MODE_TEMPO) { currentEncoderMode = MODE_OKTAF; }
      else if (currentEncoderMode == MODE_OKTAF) { currentEncoderMode = MODE_TRANSPOSE; }
      else if (currentEncoderMode == MODE_TRANSPOSE) { currentEncoderMode = MODE_PITCHBEND_SET; }
      else if (currentEncoderMode == MODE_PITCHBEND_SET) { currentEncoderMode = MODE_TEMPO; }
    }
    lastEncoderButtonDebounce = millis();
  }
  lastEncoderButtonState = currentEncoderButtonState;

  // Menangani Perputaran Encoder
  if (encoderMoved) {
    noInterrupts();
    int delta = encoderPos;
    encoderPos = 0;
    encoderMoved = false;
    interrupts();

    switch (currentEncoderMode) {
      case MODE_TEMPO:
        tempo += (delta > 0) ? 5 : -5; // Ubah 5 BPM per-klik
        if (tempo > 250) tempo = 250;
        if (tempo < 40) tempo = 40;
        break;
      case MODE_OKTAF:
        currentOctaveOffset += (delta > 0) ? 1 : -1;
        if (currentOctaveOffset > 3) currentOctaveOffset = 3;
        if (currentOctaveOffset < -3) currentOctaveOffset = -3;
        break;
      case MODE_TRANSPOSE:
        transpose += (delta > 0) ? 1 : -1;
        if (transpose > 12) transpose = 12;
        if (transpose < -12) transpose = -12;
        break;
      case MODE_PITCHBEND_SET:
        pitchBendRange += (delta > 0) ? 1 : -1;
        if (pitchBendRange > 12) pitchBendRange = 12;
        if (pitchBendRange < 1) pitchBendRange = 1;
        setPitchBendRange(MIDI_CHANNEL, pitchBendRange);
        delay(10);
        break;
    }
  }

  // Umpan Balik Visual untuk Mode Aktif
  if (currentEncoderMode == MODE_PITCHBEND_SET) {
    digitalWrite(LED_PIN, (millis() / 150) % 2); // Berkedip cepat
  } else {
    digitalWrite(LED_PIN, LOW); // Mati
  }
}
