/* SynthController.ino
   Versi: 1.1 (adapted for broader Arduino/STM32 toolchains incl. mobile)
   Penulis: assistant (template & implementation)
   Target: STM32F1 (Arduino core), board: BluePill (STM32F103C8/STM32F103C6)
   Perubahan: kompatibilitas compile (no range-for, safer Serial checks),
             tetap mempertahankan semua fitur.
*/

#include <Arduino.h>

/* ---------- CONFIGURABLE ---------- */
// Matrix pins (example wiring; sesuaikan dengan wiring kamu)
const uint8_t ROW_PINS[8] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7}; // input pullups or outputs depending scan
const uint8_t COL_PINS[8] = {PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14}; // outputs

// Buttons not in matrix (if any)
const uint8_t EXTRA_BTN_PINS[] = {}; // if none leave empty

// Analog pitchbend / wheel / pots pins
const uint8_t PIN_PITCH_X = A0;  // analog
const uint8_t PIN_PITCH_Y = A1;  // analog
const uint8_t PIN_PITCH_WHEEL = A2; // analog

const uint8_t PIN_POT_BALANCE = A3;    // pan
const uint8_t PIN_POT_MASTER = A4;     // master volume

// EQ sliders (8)
const uint8_t PIN_EQ[3] = {A5, A6, A7}; // if not enough analog pins use CD4051 mux

// Velocity sensors using analog inputs (example 32 sensors via matrix + multiplexers)
const bool USE_VELOCITY = true;
const uint8_t PIN_VELOCITY_ENABLE_SW = PC13; // enable/disable velocity sensors

// Chord scan and tempo switches
const uint8_t PIN_CHORD_SCAN_SW = PC14;
const uint8_t PIN_TEMPO_ENABLE_SW = PC15;
const uint8_t PIN_TEMPO_MODE_SW = PB15; // cycles 2/3/4 beats

// Rotary encoder pins + encoder button
const uint8_t PIN_ENCODER_A = PA6;
const uint8_t PIN_ENCODER_B = PA7;
const uint8_t PIN_ENCODER_SW = PA8;

// MIDI output selection
// We'll send MIDI on Serial1 at 31250 (DIN) and also on Serial (USB CDC) if available.
// Make sure Serial1 pins match your board (TX/RX).
#define MIDI_SERIAL Serial1
#define MIDI_USB Serial // fallback to USB-CDC if available (guarded by USBCON)

// CD4051 configuration (example):
// For each CD4051: S0,S1,S2 select lines and common analog pin to read
struct CD4051 {
  uint8_t pin_common; // analog input connected to Z
  uint8_t s0, s1, s2; // select pins (digital)
};
// Jika tidak pakai mux tinggalkan array kosong
CD4051 muxes[] = {
  // contoh: {A13, PB4, PB5, PB6}
  // Isi sesuai wiring kamu; jika tidak pakai mux, kosongkan array
};

/* ---------- END CONFIG ---------- */

const unsigned long DEBOUNCE_MS = 8;
const uint16_t MIDI_CHANNEL = 1; // kanal MIDI (1..16)

// mapping default: first 32 pads -> MIDI notes (C3..)
// kamu bisa ubah sesuai kebutuhan
const uint8_t BASE_NOTE = 48; // C3
uint8_t padNoteMap[64];

bool chordScanEnabled = false;
bool tempoEnabled = false;
uint8_t tempoBeats = 4;
int currentOctaveOffset = 0;
int transpose = 0;
int tempo = 120;

// state
uint8_t matrixState[8][8];
unsigned long lastDebounce[8][8];

// simple encoder state
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
bool encoderButtonPressed = false;

// helper timing
unsigned long lastScanTime = 0;
const unsigned long SCAN_INTERVAL = 6; // ms

// helper functions to send MIDI messages
void midiSendNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  uint8_t status = 0x90 | ((ch-1) & 0x0F);
  // Hardware MIDI (DIN) via Serial1
  MIDI_SERIAL.write(status); MIDI_SERIAL.write(note); MIDI_SERIAL.write(vel);
  // USB CDC if available
  #if defined(USBCON)
    MIDI_USB.write(status); MIDI_USB.write(note); MIDI_USB.write(vel);
  #endif
}
void midiSendNoteOff(uint8_t ch, uint8_t note, uint8_t vel) {
  uint8_t status = 0x80 | ((ch-1) & 0x0F);
  MIDI_SERIAL.write(status); MIDI_SERIAL.write(note); MIDI_SERIAL.write(vel);
  #if defined(USBCON)
    MIDI_USB.write(status); MIDI_USB.write(note); MIDI_USB.write(vel);
  #endif
}
void midiSendCC(uint8_t ch, uint8_t cc, uint8_t val) {
  uint8_t status = 0xB0 | ((ch-1) & 0x0F);
  MIDI_SERIAL.write(status); MIDI_SERIAL.write(cc); MIDI_SERIAL.write(val);
  #if defined(USBCON)
    MIDI_USB.write(status); MIDI_USB.write(cc); MIDI_USB.write(val);
  #endif
}
void midiSendPitchbend(uint8_t ch, int value14) {
  if (value14 < 0) value14 = 0;
  if (value14 > 16383) value14 = 16383;
  uint8_t lsb = value14 & 0x7F;
  uint8_t msb = (value14 >> 7) & 0x7F;
  uint8_t status = 0xE0 | ((ch-1) & 0x0F);
  MIDI_SERIAL.write(status); MIDI_SERIAL.write(lsb); MIDI_SERIAL.write(msb);
  #if defined(USBCON)
    MIDI_USB.write(status); MIDI_USB.write(lsb); MIDI_USB.write(msb);
  #endif
}

// read CD4051 channel
int readMuxAnalog(CD4051 &m, uint8_t channel) {
  // channel 0..7
  digitalWrite(m.s0, channel & 0x01);
  digitalWrite(m.s1, (channel>>1) & 0x01);
  digitalWrite(m.s2, (channel>>2) & 0x01);
  delayMicroseconds(50);
  int v = analogRead(m.pin_common);
  return v;
}

// setup map default
void setupPadMap() {
  for (int r=0; r<8; ++r) {
    for (int c=0; c<8; ++c) {
      int idx = r*8 + c;
      padNoteMap[idx] = BASE_NOTE + idx; // linear mapping
    }
  }
}

// matrix read functions
void setAllColsHigh() {
  for (int c=0;c<8;c++){
    pinMode(COL_PINS[c], OUTPUT);
    digitalWrite(COL_PINS[c], HIGH);
  }
}
void setRowAsInputPullup(int r) {
  pinMode(ROW_PINS[r], INPUT_PULLUP);
}
void setRowAsOutputLow(int r) {
  pinMode(ROW_PINS[r], OUTPUT);
  digitalWrite(ROW_PINS[r], LOW);
}

void scanMatrix() {
  unsigned long now = millis();
  if (now - lastScanTime < SCAN_INTERVAL) return;
  lastScanTime = now;

  for (int c=0;c<8;c++){
    // drive one column low (active)
    for (int cc=0; cc<8; ++cc) {
      pinMode(COL_PINS[cc], OUTPUT);
      digitalWrite(COL_PINS[cc], HIGH);
    }
    digitalWrite(COL_PINS[c], LOW);
    delayMicroseconds(50);
    for (int r=0;r<8;r++){
      // read row
      pinMode(ROW_PINS[r], INPUT_PULLUP);
      int val = digitalRead(ROW_PINS[r]) == LOW ? 1 : 0; // pressed if LOW
      if (val != matrixState[r][c]) {
        if (millis() - lastDebounce[r][c] > DEBOUNCE_MS) {
          // state changed
          matrixState[r][c] = val;
          lastDebounce[r][c] = millis();
          int padIndex = r*8 + c;
          uint8_t note = padNoteMap[padIndex] + currentOctaveOffset*12 + transpose;
          if (val) {
            // pressed -> velocity read
            uint8_t velocity = 100;
            if (USE_VELOCITY && digitalRead(PIN_VELOCITY_ENABLE_SW) == LOW) {
              int v = 512;
              if ((sizeof(muxes)/sizeof(muxes[0])) > 0) {
                // map padIndex -> mux channel (simple)
                CD4051 &m = muxes[0];
                v = readMuxAnalog(m, padIndex % 8);
              } else {
                v = analogRead(PIN_PITCH_X); // fallback
              }
              velocity = map(constrain(v, 0, 1023), 0, 1023, 10, 127);
            }
            midiSendNoteOn(MIDI_CHANNEL, note, velocity);
          } else {
            midiSendNoteOff(MIDI_CHANNEL, note, 64);
          }
        }
      }
    }
    // release column
    digitalWrite(COL_PINS[c], HIGH);
  }
}

// read analog controls
void readAnalogs() {
  // pitch X, Y -> pitchbend mapping
  int px = analogRead(PIN_PITCH_X); // 0..1023
  int py = analogRead(PIN_PITCH_Y);
  // Map px to 0..16383
  int pbx = map(px, 0, 1023, 0, 16383);
  int pby = map(py, 0, 1023, 0, 127); // fallback as CC
  midiSendPitchbend(MIDI_CHANNEL, pbx);
  midiSendCC(MIDI_CHANNEL, 16, pby); // CC16 for Y axis mapped

  // wheel
  int pw = analogRead(PIN_PITCH_WHEEL);
  int pbw = map(pw, 0, 1023, 0, 16383);
  midiSendPitchbend(MIDI_CHANNEL, pbw);

  // pots
  int pan = analogRead(PIN_POT_BALANCE);
  int vol = analogRead(PIN_POT_MASTER);
  midiSendCC(MIDI_CHANNEL, 10, map(pan, 0, 1023, 0, 127)); // CC10 pan
  midiSendCC(MIDI_CHANNEL, 7, map(vol, 0, 1023, 0, 127)); // CC7 volume

  // EQ sliders
  for (int i=0;i<3;i++){
    int v = analogRead(PIN_EQ[i]);
    midiSendCC(MIDI_CHANNEL, 70 + i, map(v, 0, 1023, 0, 127));
    // keep very short gap to avoid saturating serial
    delayMicroseconds(500);
  }
}

// simple chord detection: tries to detect triad (root + major/minor third + fifth)
void chordDetectorAndActions() {
  // collect pressed notes
  uint8_t pressedNotes[64];
  int pn = 0;
  for (int r=0;r<8;r++) for (int c=0;c<8;c++) {
    if (matrixState[r][c]) {
      pressedNotes[pn++] = padNoteMap[r*8 + c] + currentOctaveOffset*12 + transpose;
    }
  }
  if (pn >= 3) {
    // try every combination of three notes (naive)
    for (int a=0;a<pn;a++) for (int b=a+1;b<pn;b++) for (int c=b+1;c<pn;c++) {
      int x = pressedNotes[a], y = pressedNotes[b], z = pressedNotes[c];
      int arr[3] = {x,y,z};
      // sort
      for (int i=0;i<2;i++) for (int j=i+1;j<3;j++) if (arr[j]<arr[i]) { int t=arr[i]; arr[i]=arr[j]; arr[j]=t; }
      int r1 = arr[0], r2 = arr[1], r3 = arr[2];
      int i1 = (r2 - r1) % 12; if (i1<0) i1+=12;
      int i2 = (r3 - r2) % 12; if (i2<0) i2+=12;
      // major triad intervals: 4,3 or minor: 3,4
      if ((i1==4 && i2==3) || (i1==3 && i2==4)) {
        bool isMajor = (i1==4 && i2==3);
        midiSendCC(MIDI_CHANNEL, 20, isMajor ? 1 : 0); // CC20 = chord major flag
        return;
      }
    }
  }
}

// encoder ISR/read
void encoderISR() {
  static uint8_t last = 0;
  uint8_t s = (digitalRead(PIN_ENCODER_A) << 1) | (digitalRead(PIN_ENCODER_B) & 0x1);
  if (s != last) {
    if ((last == 0b00 && s == 0b01) || (last == 0b01 && s == 0b11) || (last == 0b11 && s == 0b10) || (last == 0b10 && s == 0b00)) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    encoderMoved = true;
    last = s;
  }
}

void setupPins() {
  for (int i=0;i<8;i++){
    pinMode(ROW_PINS[i], INPUT_PULLUP);
    pinMode(COL_PINS[i], OUTPUT);
    digitalWrite(COL_PINS[i], HIGH);
  }
  pinMode(PIN_VELOCITY_ENABLE_SW, INPUT_PULLUP);
  pinMode(PIN_CHORD_SCAN_SW, INPUT_PULLUP);
  pinMode(PIN_TEMPO_ENABLE_SW, INPUT_PULLUP);
  pinMode(PIN_TEMPO_MODE_SW, INPUT_PULLUP);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);

  // mux pins
  size_t nmux = sizeof(muxes)/sizeof(muxes[0]);
  for (size_t mi=0; mi<nmux; ++mi) {
    pinMode(muxes[mi].s0, OUTPUT);
    pinMode(muxes[mi].s1, OUTPUT);
    pinMode(muxes[mi].s2, OUTPUT);
  }
}

void setup() {
  // Serial for debug
  Serial.begin(115200);
  // MIDI serial (hardware UART) at 31250
  MIDI_SERIAL.begin(31250);
  // USB Serial if available (Serial)
  #if defined(USBCON)
    MIDI_USB.begin(115200);
  #endif

  setupPadMap();
  setupPins();

  // attach interrupts - use digitalPinToInterrupt to be portable
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);

  Serial.println("SynthController started (v1.1)");
}

void loop() {
  // read switches
  chordScanEnabled = (digitalRead(PIN_CHORD_SCAN_SW) == LOW);
  tempoEnabled = (digitalRead(PIN_TEMPO_ENABLE_SW) == LOW);
  // tempo mode switch (cycle 2/3/4)
  if (digitalRead(PIN_TEMPO_MODE_SW) == LOW) {
    tempoBeats = (tempoBeats % 4) + 1; // cycles 1..4 (we'll interpret 1->2,2->3,3->4,4->2 etc)
    delay(200);
  }

  scanMatrix();
  readAnalogs();
  if (chordScanEnabled) chordDetectorAndActions();

  // handle encoder movement
  if (encoderMoved) {
    int delta = encoderPos;
    encoderPos = 0;
    encoderMoved = false;
    // mode: octave/tempo/transpose based on encoder button (simple toggle)
    if (digitalRead(PIN_ENCODER_SW) == LOW) {
      // octave mode
      currentOctaveOffset += (delta>0) ? 1 : -1;
      if (currentOctaveOffset > 3) currentOctaveOffset = 3;
      if (currentOctaveOffset < -3) currentOctaveOffset = -3;
      midiSendCC(MIDI_CHANNEL, 21, currentOctaveOffset + 64); // CC21 arbitrary
    } else {
      // transpose mode
      transpose += (delta>0) ? 1 : -1;
      if (transpose > 12) transpose = 12;
      if (transpose < -12) transpose = -12;
      midiSendCC(MIDI_CHANNEL, 22, transpose + 64);
    }
  }

  // small delay to yield
  delay(4);
}
