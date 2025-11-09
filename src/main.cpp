/*
  STM32 61-key (8x8 matrix) -> MIDI controller (Arduino-style)
  - 8x8 matrix scanning
  - 61 keys mapped starting from MIDI note C2 (36)
  - MIDI over Serial1 (TX=PA9) at 31250 baud
  - Control serial (USB) for mapping commands at 115200 baud
  - Simple commands:
      DUMP                  -> print mapping list
      SET <i> <n>           -> set mapping index i (0..63) to MIDI note n (0..127)
      SETBASE <n>           -> set BASE NOTE and reset linear mapping
      RESETMAP              -> reset to linear mapping
      HELP                  -> list commands
  - Indexing: index = column * NUM_ROWS + row  (column-major)
*/

#include <Arduino.h>

// ----- configuration -----
#define NUM_ROWS 8
#define NUM_COLS 8
#define NUM_KEYS (NUM_ROWS * NUM_COLS)
#define USED_KEYS 61
#define DEFAULT_BASE_NOTE 36  // C2

// Rows input pins (INPUT_PULLUP)
const uint8_t rowPins[NUM_ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
// Columns output pins (drive LOW when active)
const uint8_t colPins[NUM_COLS] = {PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13};

// MIDI serial (UART1 TX=PA9)
#define MIDI_SERIAL Serial1
#define MIDI_BAUD 31250

// Control serial (USB/USB-serial)
#define CTRL_SERIAL Serial
#define CTRL_BAUD 115200

// debounce / scan
const uint16_t SCAN_INTERVAL_MS = 2;
const uint8_t DEBOUNCE_READS = 3;

// mapping table
uint8_t keyMap[NUM_KEYS];
uint8_t baseNote = DEFAULT_BASE_NOTE;

// state buffers
bool keyState[NUM_KEYS];
uint8_t keyBounce[NUM_KEYS];

// timing
unsigned long lastScan = 0;

// ---------------- MIDI helpers ----------------
void midiSend3(uint8_t a, uint8_t b, uint8_t c) {
  MIDI_SERIAL.write(a);
  MIDI_SERIAL.write(b);
  MIDI_SERIAL.write(c);
}
void midiNoteOn(uint8_t note, uint8_t vel) {
  midiSend3(0x90, note, vel);
}
void midiNoteOff(uint8_t note, uint8_t vel) {
  midiSend3(0x80, note, vel);
}

// index helper (column-major)
inline int keyIndexFromCR(int col, int row) {
  return col * NUM_ROWS + row;
}

// set linear default mapping
void resetMappingToLinear() {
  for (int i = 0; i < NUM_KEYS; ++i) {
    if (i < USED_KEYS) keyMap[i] = baseNote + i;
    else keyMap[i] = 0xFF; // unused
  }
  memset(keyState, 0, sizeof(keyState));
  memset(keyBounce, 0, sizeof(keyBounce));
}

// pin setup
void setupPins() {
  // rows input pullup
  for (int r = 0; r < NUM_ROWS; ++r) {
    pinMode(rowPins[r], INPUT_PULLUP);
  }
  // cols output high (inactive)
  for (int c = 0; c < NUM_COLS; ++c) {
    pinMode(colPins[c], OUTPUT);
    digitalWrite(colPins[c], HIGH);
  }
}

// read all row pins and return bitmask (1 = pressed)
uint8_t readRowsBitmask() {
  uint8_t mask = 0;
  for (int r = 0; r < NUM_ROWS; ++r) {
    if (digitalRead(rowPins[r]) == LOW) mask |= (1 << r); // active low
  }
  return mask;
}

// matrix scanning & MIDI event generation
void scanMatrixOnce() {
  for (int c = 0; c < NUM_COLS; ++c) {
    // drive column active (LOW)
    digitalWrite(colPins[c], LOW);
    delayMicroseconds(40); // settle
    uint8_t mask = readRowsBitmask();
    for (int r = 0; r < NUM_ROWS; ++r) {
      int idx = keyIndexFromCR(c, r);
      bool pressed = (mask >> r) & 1;
      // debounce
      if (pressed) {
        if (keyBounce[idx] < DEBOUNCE_READS) keyBounce[idx]++;
      } else {
        if (keyBounce[idx] > 0) keyBounce[idx]--;
      }
      bool stablePressed = (keyBounce[idx] >= DEBOUNCE_READS);
      if (stablePressed != keyState[idx]) {
        keyState[idx] = stablePressed;
        uint8_t mapped = keyMap[idx];
        if (mapped != 0xFF) {
          if (stablePressed) {
            midiNoteOn(mapped, 100); // fixed velocity
          } else {
            midiNoteOff(mapped, 64);
          }
        }
      }
    }
    digitalWrite(colPins[c], HIGH); // release column
    delayMicroseconds(20);
  }
}

// ---------------- control serial parser ----------------
String ctrlBuffer = "";

void dumpMapping() {
  CTRL_SERIAL.println(F("Index -> MIDI note"));
  for (int i = 0; i < NUM_KEYS; ++i) {
    if (keyMap[i] == 0xFF) CTRL_SERIAL.printf("%02d: --\r\n", i);
    else CTRL_SERIAL.printf("%02d: %d\r\n", i, keyMap[i]);
  }
}

void processCtrlLine(const String &line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;
  String up = s;
  up.toUpperCase();
  if (up == "DUMP") {
    dumpMapping();
    return;
  }
  if (up == "RESETMAP") {
    resetMappingToLinear();
    CTRL_SERIAL.println(F("Mapping reset to linear."));
    return;
  }
  if (up.startsWith("SETBASE")) {
    int sp = s.indexOf(' ');
    if (sp > 0) {
      int n = s.substring(sp + 1).toInt();
      if (n >= 0 && n <= 127) {
        baseNote = (uint8_t)n;
        resetMappingToLinear();
        CTRL_SERIAL.printf("Base note set to %d and mapping reset\r\n", baseNote);
      } else {
        CTRL_SERIAL.println(F("Invalid base note (0..127)"));
      }
    } else {
      CTRL_SERIAL.println(F("Usage: SETBASE <n>"));
    }
    return;
  }
  if (up.startsWith("SET ")) {
    // SET i n
    int p1 = s.indexOf(' ');
    int p2 = s.indexOf(' ', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      int idx = s.substring(p1 + 1, p2).toInt();
      int val = s.substring(p2 + 1).toInt();
      if (idx >= 0 && idx < NUM_KEYS && val >= 0 && val <= 127) {
        keyMap[idx] = (uint8_t)val;
        CTRL_SERIAL.printf("MAP[%d] = %d\r\n", idx, val);
      } else {
        CTRL_SERIAL.println(F("Invalid args (idx 0..63, note 0..127)"));
      }
    } else {
      CTRL_SERIAL.println(F("Usage: SET <index> <note>"));
    }
    return;
  }
  if (up == "HELP") {
    CTRL_SERIAL.println(F("Commands: DUMP, SET <idx> <note>, SETBASE <n>, RESETMAP, HELP"));
    return;
  }
  CTRL_SERIAL.println(F("Unknown command. Type HELP."));
}

// ---------------- setup / loop ----------------
void setup() {
  CTRL_SERIAL.begin(CTRL_BAUD);
  delay(50);
  CTRL_SERIAL.println(F("STM32 61-key (8x8) MIDI controller booting..."));
  MIDI_SERIAL.begin(MIDI_BAUD);

  setupPins();
  resetMappingToLinear();
  CTRL_SERIAL.println(F("Ready. Type HELP."));
  lastScan = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastScan >= SCAN_INTERVAL_MS) {
    scanMatrixOnce();
    lastScan = now;
  }
  // control serial
  while (CTRL_SERIAL.available()) {
    char c = (char)CTRL_SERIAL.read();
    if (c == '\r') continue;
    if (c == '\n') {
      processCtrlLine(ctrlBuffer);
      ctrlBuffer = "";
    } else {
      ctrlBuffer += c;
      if (ctrlBuffer.length() > 200) ctrlBuffer = ctrlBuffer.substring(ctrlBuffer.length() - 200);
    }
  }
}


