/**
 * SynthController.ino
 * STM32F103C6T6 - USB MIDI class device + 8x8 matrix + joystick X/Y + encoder range control
 *
 * Features:
 * - USB-MIDI class-compliant (uses USBComposite library)
 * - 8x8 pad matrix (note on/off)
 * - Joystick Y = pitchbend (14-bit)
 * - Joystick X = normal -> CC#1 (mod), or Trem Mode -> pitchbend within encoder-set range
 * - Encoder rotate adjusts Trem Range (continuous 0..max)
 * - Encoder push toggles Trem Mode (Normal <-> Trem)
 * - Master switch (ON/OFF) disables all pitch functions when OFF
 * - LED status: indicates master + mode + encoder-adjust activity
 * - Non-blocking: no delay(), uses millis() timing
 *
 * Pin mapping (check carefully):
 * ANALOG:
 *  A0 (PA0) JOY_X
 *  A1 (PA1) JOY_Y
 *  A2 (PA2) POT_BALANCE (CC10)
 *  A3 (PA3) POT_MASTER  (CC7)
 *  A4 (PA4) EQ1 (CC70)
 *  A5 (PA5) EQ2 (CC71)
 *  A6 (PA6) EQ3 (CC72)
 *
 * MATRIX:
 *  ROWS: PB0..PB7
 *  COLS: PB8..PB15
 *
 * ENCODER:
 *  ENC_A PA7 (interrupt)
 *  ENC_B PA8
 *  ENC_SW PA9 (push)
 *
 * MASTER SWITCH: PA10 (active LOW = pressed)
 * LED: PC13 (onboard LED, active HIGH -> light on)
 *
 * Ensure STM32 core + USBComposite library available in PlatformIO.
 */

#include <Arduino.h>
#include <USBComposite.h> // provides USB MIDI device (must be installed in PlatformIO)

USBMIDI_CREATE_INSTANCE(0); // create global USB MIDI instance

// --- PIN DEFINITIONS (adjust only if your board pinout differs) ---
constexpr uint8_t PIN_JOY_X = A0; // PA0
constexpr uint8_t PIN_JOY_Y = A1; // PA1
constexpr uint8_t PIN_POT_BALANCE = A2; // A2
constexpr uint8_t PIN_POT_MASTER = A3;  // A3
constexpr uint8_t PIN_EQ0 = A4; // A4
constexpr uint8_t PIN_EQ1 = A5; // A5
constexpr uint8_t PIN_EQ2 = A6; // A6

// Matrix pins - rows PB0..PB7, cols PB8..PB15
constexpr uint8_t MATRIX_ROWS = 8;
constexpr uint8_t MATRIX_COLS = 8;
const uint8_t ROW_PINS[MATRIX_ROWS] = {PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7};
const uint8_t COL_PINS[MATRIX_COLS] = {PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15};

// Encoder pins
constexpr uint8_t PIN_ENC_A = PA7;
constexpr uint8_t PIN_ENC_B = PA8;
constexpr uint8_t PIN_ENC_SW = PA9;

// Master switch & LED
constexpr uint8_t PIN_MASTER_SW = PA10; // active LOW = pressed
constexpr uint8_t PIN_LED = PC13;       // onboard LED

// MIDI settings
const uint8_t MIDI_CHANNEL = 1;
const uint8_t BASE_NOTE = 48; // C3

// Matrix state and debounce
volatile uint8_t matrixState[MATRIX_ROWS][MATRIX_COLS] = {0};
unsigned long lastDebounce[MATRIX_ROWS][MATRIX_COLS] = {0};
const unsigned long DEBOUNCE_MS = 12; // ms

// pad -> note map
uint8_t padNoteMap[64];

// scan timing
const unsigned long SCAN_INTERVAL = 4; // ms (~250Hz)
unsigned long lastScan = 0;

// analog smoothing/change thresholds
const int ANALOG_THRESH = 2; // raw ADC change threshold to send

// last analog values to avoid flooding
int lastPitchY = -1;
int lastModX = -1;
int lastPan = -1;
int lastVol = -1;
int lastEQ[3] = {-1, -1, -1};

// Encoder state (non-blocking)
volatile int32_t encoderCounter = 0; // always increment/decrement from ISR
volatile uint8_t lastEncState = 0;
bool encButtonPressed = false;
unsigned long lastEncBtnDebounce = 0;
const unsigned long ENC_BTN_DEBOUNCE = 40;

// Tremolo / mode / master
volatile bool tremMode = false;           // toggled by ENC push
volatile bool masterEnabled = true;       // read from switch (PA10)
volatile int tremRange = 0;               // 0..8192 (14-bit half-range)
const int TREM_RANGE_MAX = 8192;

// LED timing
unsigned long ledBlinkUntil = 0;
unsigned long ledLastToggle = 0;

// Encoder read smoothing (main loop)
int32_t lastEncoderCounterRead = 0;

// Matrix scanning: set rows outputs low one by one, read cols (input_pullup)
void setupPadMap() {
  for (uint8_t i = 0; i < 64; ++i) padNoteMap[i] = BASE_NOTE + i;
}

void midiSendRaw(uint8_t status, uint8_t d1, uint8_t d2) {
  // send over USB MIDI (class) and optionally Serial1 if used
  // Using USBMIDI instance API
  // Convert to USBMIDI calls:
  // Note On / Off / CC / Pitchbend
  // But USBMIDI library has send* functions; we'll call appropriate ones below instead of raw
  (void)status; (void)d1; (void)d2;
}

// Helper send functions using USBMIDI API
inline void sendNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  USBMIDI.sendNoteOn(note, vel, ch);
}
inline void sendNoteOff(uint8_t ch, uint8_t note, uint8_t vel) {
  USBMIDI.sendNoteOff(note, vel, ch);
}
inline void sendCC(uint8_t ch, uint8_t cc, uint8_t val) {
  USBMIDI.sendControlChange(cc, val, ch);
}
inline void sendPitchBend14(uint8_t ch, int value14) {
  // clamp
  if (value14 < 0) value14 = 0;
  if (value14 > 16383) value14 = 16383;
  // USBMIDI pitchbend expects signed? Many libs accept value -8192..8191 or raw 0..16383.
  // USBMIDI.sendPitchBend accepts (int value, uint8_t channel) with value in -8192..8191 typically.
  int16_t signedV = value14 - 8192;
  USBMIDI.sendPitchBend(signedV, ch);
}

// Matrix scanning (non-blocking; called from loop at interval)
void scanMatrixOnce() {
  unsigned long now = millis();
  if (now - lastScan < SCAN_INTERVAL) return;
  lastScan = now;

  for (uint8_t c = 0; c < MATRIX_COLS; ++c) {
    // activate column (set as OUTPUT LOW)
    pinMode(COL_PINS[c], OUTPUT);
    digitalWrite(COL_PINS[c], LOW);
    // small settle (no delay) — allow one ADC cycle worth (we'll not block but read)
    for (uint8_t r = 0; r < MATRIX_ROWS; ++r) {
      pinMode(ROW_PINS[r], INPUT_PULLUP);
      bool pressed = (digitalRead(ROW_PINS[r]) == LOW);
      if (pressed != (matrixState[r][c] != 0)) {
        if (millis() - lastDebounce[r][c] > DEBOUNCE_MS) {
          lastDebounce[r][c] = millis();
          matrixState[r][c] = pressed ? 1 : 0;
          uint8_t note = padNoteMap[r * MATRIX_COLS + c];
          if (pressed) sendNoteOn(MIDI_CHANNEL, note, 100);
          else sendNoteOff(MIDI_CHANNEL, note, 0);
        }
      }
    }
    // deactivate column
    pinMode(COL_PINS[c], INPUT);
  }
}

// Read analogs and send CC / Pitchbend accordingly (non-blocking)
void handleAnalogs() {
  // Read Y -> Pitch Bend (only if masterEnabled)
  int rawY = analogRead(PIN_JOY_Y); // 0..4095 or 0..1023 depending on ADC resolution; Arduino core typically 12-bit -> 0..4095
  // PlatformIO/STM32duino default ADC resolution is 12-bit (0..4095). Map accordingly.
  // We'll map using analogReadResolution if available. For portability, detect 12-bit.
  int adcMax = 4095;
#if defined(ADC_RESOLUTION)
  (void)0;
#endif

  // Map rawY to 0..16383
  int pbY = map(rawY, 0, adcMax, 0, 16383);
  // Apply master
  if (!masterEnabled) {
    // send center pitch (only when changed)
    if (lastPitchY != 8192) {
      sendPitchBend14(MIDI_CHANNEL, 8192);
      lastPitchY = 8192;
    }
  } else {
    if (abs(pbY - lastPitchY) > ANALOG_THRESH) {
      lastPitchY = pbY;
      sendPitchBend14(MIDI_CHANNEL, pbY);
    }
  }

  // Read X
  int rawX = analogRead(PIN_JOY_X);
  int modX = map(rawX, 0, adcMax, 0, 127);
  if (!tremMode || !masterEnabled) {
    // normal mode: send CC#1 (mod)
    if (modX != lastModX) {
      lastModX = modX;
      sendCC(MIDI_CHANNEL, 1, (uint8_t)modX);
    }
  } else {
    // Trem mode: X controls pitchbend limited by tremRange
    // tremRange in 0..TREM_RANGE_MAX (14-bit half range)
    long bend = map(rawX, 0, adcMax, -tremRange, tremRange);
    long out = 8192 + bend;
    if (out < 0) out = 0;
    if (out > 16383) out = 16383;
    // compare with lastPitchY? we track lastPitchY for Y, so we'll send even if same to ensure real-time
    // We'll keep a separate last value to avoid flooding
    static int lastTremSent = -1;
    if ((int)out != lastTremSent) {
      lastTremSent = out;
      sendPitchBend14(MIDI_CHANNEL, (int)out);
    }
  }

  // Pots: PAN (A2) -> CC10, MASTER -> CC7
  int panRaw = analogRead(PIN_POT_BALANCE);
  int pan = map(panRaw, 0, adcMax, 0, 127);
  if (abs(pan - lastPan) > 1) { lastPan = pan; sendCC(MIDI_CHANNEL, 10, pan); }

  int volRaw = analogRead(PIN_POT_MASTER);
  int vol = map(volRaw, 0, adcMax, 0, 127);
  if (abs(vol - lastVol) > 1) { lastVol = vol; sendCC(MIDI_CHANNEL, 7, vol); }

  int eqRaw0 = analogRead(PIN_EQ0); int eq0 = map(eqRaw0,0,adcMax,0,127);
  int eqRaw1 = analogRead(PIN_EQ1); int eq1 = map(eqRaw1,0,adcMax,0,127);
  int eqRaw2 = analogRead(PIN_EQ2); int eq2 = map(eqRaw2,0,adcMax,0,127);
  if (eq0 != lastEQ[0]) { lastEQ[0] = eq0; sendCC(MIDI_CHANNEL, 70, eq0); }
  if (eq1 != lastEQ[1]) { lastEQ[1] = eq1; sendCC(MIDI_CHANNEL, 71, eq1); }
  if (eq2 != lastEQ[2]) { lastEQ[2] = eq2; sendCC(MIDI_CHANNEL, 72, eq2); }
}

// Encoder ISR — handles quadrature (attach to ENC_A pin change)
void IRAM_ATTR encoderISR() {
  // Read both pins (fast)
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  // Gray code state machine
  static const int8_t tbl[4][4] = {
    {0, -1, 1, 0},
    {1, 0, 0, -1},
    {-1, 0, 0, 1},
    {0, 1, -1, 0}
  };
  static uint8_t last = 0;
  if (s != last) {
    int8_t delta = tbl[last & 3][s & 3];
    if (delta) encoderCounter += delta;
    last = s;
  }
}

// Read encoder button (debounced) — called in loop (non-blocking)
void handleEncoderButton() {
  bool pressed = (digitalRead(PIN_ENC_SW) == LOW);
  unsigned long now = millis();
  if (pressed && (now - lastEncBtnDebounce) > ENC_BTN_DEBOUNCE) {
    // toggle tremMode on press edge
    if (!encButtonPressed) {
      tremMode = !tremMode;
      // set LED quick blink while adjusting
      ledBlinkUntil = now + 400;
    }
    encButtonPressed = true;
    lastEncBtnDebounce = now;
  } else if (!pressed) {
    encButtonPressed = false;
  }
}

// Handle encoder rotation in main loop (non-blocking)
void handleEncoderRotation() {
  // read current counter, compare to last read
  int32_t cur;
  noInterrupts();
  cur = encoderCounter;
  interrupts();
  if (cur != lastEncoderCounterRead) {
    int32_t delta = cur - lastEncoderCounterRead;
    lastEncoderCounterRead = cur;
    // adjust tremRange based on delta
    // scale: each encoder tick changes by 16 (fine) => can be adjusted
    tremRange += (int)(delta * 16);
    if (tremRange < 0) tremRange = 0;
    if (tremRange > TREM_RANGE_MAX) tremRange = TREM_RANGE_MAX;
    // set LED blink while editing
    ledBlinkUntil = millis() + 800;
  }
}

// Read master switch (non-blocking)
void handleMasterSwitch() {
  // switch active LOW
  bool m = (digitalRead(PIN_MASTER_SW) == LOW);
  masterEnabled = m;
}

// LED status updates
void handleLED() {
  unsigned long now = millis();
  if (!masterEnabled) {
    digitalWrite(PIN_LED, LOW); // off
    return;
  }
  if (millis() < ledBlinkUntil) {
    // Fast blink while editing
    if (now - ledLastToggle > 120) {
      ledLastToggle = now;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
    return;
  }
  // steady or slow blink depending on mode
  if (tremMode) {
    if (now - ledLastToggle > 400) {
      ledLastToggle = now;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
  } else {
    // steady ON
    digitalWrite(PIN_LED, HIGH);
  }
}

// Setup pins carefully, no overlapping usage
void setupPins() {
  // Analog pins implicitly input for analogRead
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_MASTER_SW, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Matrix pins: initialize columns as INPUT (HiZ), rows as INPUT_PULLUP
  for (uint8_t r = 0; r < MATRIX_ROWS; ++r) {
    pinMode(ROW_PINS[r], INPUT_PULLUP);
  }
  for (uint8_t c = 0; c < MATRIX_COLS; ++c) {
    pinMode(COL_PINS[c], INPUT);
  }
}

// Setup routine
void setup() {
  // Serial for debug (optional)
  Serial.begin(115200);
  delay(10);

  setupPadMap();
  setupPins();

  // attach encoder ISR to ENC_A pin change
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  // Initialize USB MIDI
  USBMIDI.begin();
  // Set default pitchbend range via RPN (so host sees correct range)
  // Use Control Change sequence (RPN 0)
  sendCC(MIDI_CHANNEL, 101, 0);
  sendCC(MIDI_CHANNEL, 100, 0);
  sendCC(MIDI_CHANNEL, 6, 2);  // default +/-2 semitone
  sendCC(MIDI_CHANNEL, 38, 0);
  // cancel RPN
  sendCC(MIDI_CHANNEL, 101, 127);
  sendCC(MIDI_CHANNEL, 100, 127);

  Serial.println("SynthController STM32F103C6 - USB MIDI ready");
}

// main loop — non-blocking
void loop() {
  // 1) handle matrix scanning (note on/off)
  scanMatrixOnce();

  // 2) handle encoder button (toggle mode)
  handleEncoderButton();

  // 3) handle encoder rotation adjustments
  handleEncoderRotation();

  // 4) read master switch
  handleMasterSwitch();

  // 5) handle analogs (pitchbend, CCs, EQ)
  handleAnalogs();

  // 6) LED update
  handleLED();

  // Yield to USB stack to process events
  USBMIDI.Task();
}
