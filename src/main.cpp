#include <Arduino.h>
#include <USBMIDI.h>

USBMIDI MIDI;

// ==========================
// MATRIX 8×8 (64 keys)
// ==========================
const uint8_t ROWS[8] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t COLS[8] = {PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15};

// State key
uint8_t keyState[8][8];

// ==========================
// MULTIPLEXER CD74HC4067
// ==========================
const uint8_t MUX_S0 = PB8;
const uint8_t MUX_S1 = PB9;
const uint8_t MUX_S2 = PA8;
const uint8_t MUX_S3 = PA9;
const uint8_t MUX_EN = PA10;   // EN LOW = aktif
const uint8_t MUX_SIG = PA11;  // Output ADC

// Input channel multiplexer
enum {
  CH_FADER_1, CH_FADER_2, CH_FADER_3, CH_FADER_4,
  CH_FADER_5, CH_FADER_6, CH_FADER_7, CH_FADER_8,
  CH_VELOCITY_POT,
  CH_JOYSTICK_X,
  CH_JOYSTICK_Y,
  CH_UNUSED_11,
  CH_UNUSED_12,
  CH_UNUSED_13,
  CH_UNUSED_14,
  CH_UNUSED_15
};

// ==========================
// DIRECT ADC (NO MUX)
// ==========================
const uint8_t POT_VOLUME  = PA15;
const uint8_t POT_BALANCE = PB4;

// ==========================
// BUTTONS
// ==========================
const uint8_t PIN_OCTAVE_UP   = PB5;
const uint8_t PIN_OCTAVE_DOWN = PB6;
const uint8_t PIN_TRANSPOSE_UP   = PB7;
const uint8_t PIN_TRANSPOSE_DOWN = PB3;

const uint8_t PIN_WHEEL_MODE = PA12;      // Toggle joystick → pitchbend / modwheel
const uint8_t PIN_VELOCITY_ONOFF = PA13;  // Velocity ON/OFF

const uint8_t PIN_TEMPO_MODE = PA14;      // Change tempo parameter mode

// ==========================
// ENCODER (TEMPO / MENU)
// ==========================
const uint8_t ENC_A = PB2;
const uint8_t ENC_B = PB10;
const uint8_t ENC_SW = PB1;

int lastEnc = 0;

// ==========================
// GLOBAL PARAMETERS
// ==========================
bool velocityEnabled = true;
uint8_t velocityValue = 100;

int octaveShift = 0;
int transpose = 0;

bool wheelPitchMode = true; // true = pitchbend, false = modulation

uint16_t tempoBPM = 120;

// ==========================
// READ MUX
// ==========================
int readMux(uint8_t ch) {
  digitalWrite(MUX_S0, (ch & 1) ? HIGH : LOW);
  digitalWrite(MUX_S1, (ch & 2) ? HIGH : LOW);
  digitalWrite(MUX_S2, (ch & 4) ? HIGH : LOW);
  digitalWrite(MUX_S3, (ch & 8) ? HIGH : LOW);
  digitalWrite(MUX_EN, LOW);
  delayMicroseconds(4);
  return analogRead(MUX_SIG);
}

// ==========================
// SETUP
// ==========================
void setup() {
  // MATRIX
  for (int i = 0; i < 8; i++) {
    pinMode(ROWS[i], OUTPUT);
    pinMode(COLS[i], INPUT_PULLUP);
  }

  // MUX
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_EN, OUTPUT);
  pinMode(MUX_SIG, INPUT_ANALOG);

  // DIRECT ADC
  pinMode(POT_VOLUME, INPUT_ANALOG);
  pinMode(POT_BALANCE, INPUT_ANALOG);

  // BUTTONS
  pinMode(PIN_OCTAVE_UP, INPUT_PULLUP);
  pinMode(PIN_OCTAVE_DOWN, INPUT_PULLUP);
  pinMode(PIN_TRANSPOSE_UP, INPUT_PULLUP);
  pinMode(PIN_TRANSPOSE_DOWN, INPUT_PULLUP);
  pinMode(PIN_WHEEL_MODE, INPUT_PULLUP);
  pinMode(PIN_VELOCITY_ONOFF, INPUT_PULLUP);
  pinMode(PIN_TEMPO_MODE, INPUT_PULLUP);

  // ENCODER
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  MIDI.begin();
}

// ==========================
// MATRIX SCAN
// ==========================
void scanKeys() {
  for (int r = 0; r < 8; r++) {
    digitalWrite(ROWS[r], LOW);
    for (int c = 0; c < 8; c++) {
      bool pressed = (digitalRead(COLS[c]) == LOW);
      if (pressed && keyState[r][c] == 0) {
        int note = r * 8 + c + 36 + transpose + (octaveShift * 12);
        MIDI.sendNoteOn(note, velocityEnabled ? velocityValue : 100, 1);
        keyState[r][c] = 1;
      } else if (!pressed && keyState[r][c] == 1) {
        int note = r * 8 + c + 36 + transpose + (octaveShift * 12);
        MIDI.sendNoteOff(note, 0, 1);
        keyState[r][c] = 0;
      }
    }
    digitalWrite(ROWS[r], HIGH);
  }
}

// ==========================
// LOOP
// ==========================
void loop() {

  // ===== MATRIX =====
  scanKeys();

  // ===== BUTTON EVENTS =====
  if (!digitalRead(PIN_OCTAVE_UP))     octaveShift++;
  if (!digitalRead(PIN_OCTAVE_DOWN))   octaveShift--;
  if (!digitalRead(PIN_TRANSPOSE_UP))  transpose++;
  if (!digitalRead(PIN_TRANSPOSE_DOWN)) transpose--;

  if (!digitalRead(PIN_WHEEL_MODE))
    wheelPitchMode = !wheelPitchMode;

  if (!digitalRead(PIN_VELOCITY_ONOFF))
    velocityEnabled = !velocityEnabled;

  // ===== VELOCITY POT =====
  velocityValue = map(readMux(CH_VELOCITY_POT), 0, 4095, 1, 127);

  // ===== JOYSTICK =====
  int joyX = readMux(CH_JOYSTICK_X);
  int joyY = readMux(CH_JOYSTICK_Y);

  if (wheelPitchMode) {
    int bend = map(joyX, 0, 4095, -8192, 8191);
    MIDI.sendPitchBend(bend, 1);
  } else {
    int modwheel = map(joyY, 0, 4095, 0, 127);
    MIDI.sendControlChange(1, modwheel, 1);
  }

  // ===== FADER 8 =====
  for (int i = 0; i < 8; i++) {
    int val = readMux(i);
    val = map(val, 0, 4095, 0, 127);
    MIDI.sendControlChange(20 + i, val, 1);
  }

  // ===== MASTER & BALANCE =====
  int vol = map(analogRead(POT_VOLUME), 0, 4095, 0, 127);
  int bal = map(analogRead(POT_BALANCE), 0, 4095, 0, 127);

  MIDI.sendControlChange(7, vol, 1);
  MIDI.sendControlChange(10, bal, 1);

  delay(3);
}
