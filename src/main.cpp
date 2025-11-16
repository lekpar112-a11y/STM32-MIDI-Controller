#include <Arduino.h>
#include <MIDI.h>

// =======================
//   USB MIDI INSTANCE
// =======================
MIDI_CREATE_DEFAULT_INSTANCE();

// =======================
//   CONFIG
// =======================

// Matrix 8x8 (61 key)
const uint8_t rowPins[8] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t colPins[8] = {PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15};

bool keyState[8][8];
bool lastKeyState[8][8];

// =======================
//  Velocity Toggle
// =======================
bool velocityOn = true;
uint8_t fixedVelocity = 100;
const uint8_t velocityTogglePin = PB8;

// =======================
//  Joystick (Pitch / Mod)
// =======================
const uint8_t joyX = PA8;
const uint8_t joyY = PA9;

// =======================
//  Two Pots
// =======================
const uint8_t potMasterVol = PA10;
const uint8_t potBalance = PA11;

// =======================
//  8 Fader
// =======================
const uint8_t faderPins[8] = {
  PA12, PA15, PB3, PB4, PB5, PB6, PB7, PA13
};

int lastFader[8];

// =======================
//  CD74HC4067 Analog Multiplexer
// =======================
const uint8_t cdS0 = PB9;
const uint8_t cdS1 = PA14;
const uint8_t cdS2 = PA0;
const uint8_t cdS3 = PA1;
const uint8_t cdSIG = PA12; // analog input

int lastCD[16];

// =======================
//  Rotary Encoder
// =======================
const uint8_t encA = PB2;
const uint8_t encB = PB1;
const uint8_t encSW = PB13;

int encoderMode = 0;   // 0 tempo, 1 octave, 2 transpose, 3 wheel-select
int tempo = 120;
int octave = 0;
int transpose = 0;

int lastA = 0;

// =======================
//  Helper
// =======================
int readCD74HC4067(uint8_t ch) {
  digitalWrite(cdS0, ch & 1);
  digitalWrite(cdS1, (ch >> 1) & 1);
  digitalWrite(cdS2, (ch >> 2) & 1);
  digitalWrite(cdS3, (ch >> 3) & 1);
  delayMicroseconds(5);
  return analogRead(cdSIG);
}

void scanMatrix() {
  for (int r = 0; r < 8; r++) {
    digitalWrite(rowPins[r], LOW);
    for (int c = 0; c < 8; c++) {
      bool pressed = !digitalRead(colPins[c]);
      keyState[r][c] = pressed;

      if (pressed != lastKeyState[r][c]) {
        int note = (r * 8 + c);
        if (note <= 60) {
          if (pressed) {
            uint8_t vel = velocityOn ? map(analogRead(joyY), 0, 4095, 30, 127) : fixedVelocity;
            MIDI.sendNoteOn(note + transpose + (octave * 12), vel, 1);
          } else {
            MIDI.sendNoteOff(note + transpose + (octave * 12), 0, 1);
          }
        }
        lastKeyState[r][c] = pressed;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
}

void readFaders() {
  for (int i = 0; i < 8; i++) {
    int v = analogRead(faderPins[i]);
    if (abs(v - lastFader[i]) > 8) {
      uint8_t value = map(v, 0, 4095, 0, 127);
      MIDI.sendControlChange(20 + i, value, 1);
      lastFader[i] = v;
    }
  }
}

void readCDMultiplexer() {
  for (int ch = 0; ch < 16; ch++) {
    int v = readCD74HC4067(ch);
    if (abs(v - lastCD[ch]) > 10) {
      uint8_t value = map(v, 0, 4095, 0, 127);
      MIDI.sendControlChange(40 + ch, value, 1);
      lastCD[ch] = v;
    }
  }
}

void readJoystick() {
  int x = analogRead(joyX);
  int y = analogRead(joyY);

  uint16_t pitch = map(x, 0, 4095, 0, 16383);
  MIDI.sendPitchBend(pitch, 1);

  uint8_t mod = map(y, 0, 4095, 0, 127);
  MIDI.sendControlChange(1, mod, 1);
}

void readEncoder() {
  int a = digitalRead(encA);
  if (a != lastA) {
    if (digitalRead(encB) != a) {
      if (encoderMode == 0) tempo++;
      if (encoderMode == 1) octave++;
      if (encoderMode == 2) transpose++;
    } else {
      if (encoderMode == 0) tempo--;
      if (encoderMode == 1) octave--;
      if (encoderMode == 2) transpose--;
    }
  }
  lastA = a;

  if (!digitalRead(encSW)) {
    encoderMode++;
    if (encoderMode > 3) encoderMode = 0;
    delay(250);
  }
}

void setup() {
  MIDI.begin(MIDI_CHANNEL_OMNI);

  pinMode(velocityTogglePin, INPUT_PULLUP);

  for (int r = 0; r < 8; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }
  for (int c = 0; c < 8; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  pinMode(cdS0, OUTPUT);
  pinMode(cdS1, OUTPUT);
  pinMode(cdS2, OUTPUT);
  pinMode(cdS3, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(encSW, INPUT_PULLUP);
}

void loop() {
  velocityOn = digitalRead(velocityTogglePin);

  scanMatrix();
  readFaders();
  readCDMultiplexer();
  readJoystick();
  readEncoder();

  delay(2);
}
