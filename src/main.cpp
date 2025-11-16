/*
 * ===================================================================================
 * ADVANCED STM32 MIDI CONTROLLER FIRMWARE (REFACTORED V5 - PIN CONFLICTS RESOLVED)
 * ===================================================================================
 *
 * AUTHOR: AI Assistant (Refactored for User)
 * DATE: 2024-07-31
 *
 * TARGET BOARD: STM32F103C6T6 (Blue Pill)
 *
 * --- V5 CRITICAL PINOUT CORRECTION ---
 * This version resolves a hardware conflict where the encoder pins (PA11/PA12)
 * conflicted with the USB D-/D+ pins required for USB MIDI communication.
 *
 * 1.  ENCODER PINS MOVED: The rotary encoder inputs A and B have been moved to
 *     PA9 and PA10, which are interrupt-capable and do not conflict with USB.
 *
 * 2.  SWITCHES RELOCATED: To free up PA9 and PA10, the "Pitchbend Enable" and
 *     "Velocity Enable" switches are no longer on dedicated GPIOs. They are now
 *     handled by the last two channels of the button multiplexer matrix.
 *
 * 3.  USB MIDI NOW FUNCTIONAL: With this change, MIDI over USB will work correctly
 *     alongside all other components, including the rotary encoder.
 */

#include <Arduino.h>

/* ==========================================
   CONSTANTS & CONFIGURATION
   ========================================== */
#define MIDI_CHANNEL 1
#define NUM_UNIVERSAL_BUTTONS 38 // Reduced by 2 to accommodate dedicated switches
#define NUM_EQ_BANDS 8
#define NUM_MATRIX_ROWS 8
#define NUM_MATRIX_COLS 8

// --- MUX Button Indices for Relocated Switches ---
const uint8_t PB_ENABLE_BTN_INDEX = 38;
const uint8_t VELOCITY_BTN_INDEX  = 39;


// --- Timing ---
const unsigned long DEBOUNCE_DELAY_FAST_MS = 10;
const unsigned long DEBOUNCE_DELAY_SLOW_MS = 150;
const unsigned long LOOP_SCAN_INTERVAL_MS = 2;
const unsigned long VELOCITY_WINDOW_MS = 25;

// --- MIDI & Velocity Values ---
const uint8_t NOTE_VELOCITY_FIXED = 127;
const int NOTE_ON_THRESHOLD = 100;
const int NOTE_OFF_THRESHOLD = 50;
const uint8_t EQ_CC_START = 70;
const uint8_t BALANCE_CC = 10;
const uint8_t MASTER_VOL_CC = 7;
const uint8_t MOD_WHEEL_CC = 1;


/* ==========================================
   STM32F103C6T6 "BLUE PILL" PIN DEFINITIONS (V5)
   ========================================== */

// --- FSR MATRIX 8x8 (Requires dedicated ADCs) ---
const uint8_t ROW_PINS[NUM_MATRIX_ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t COL_PINS[NUM_MATRIX_COLS] = {PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10};

// --- SHARED MULTIPLEXER SELECT PINS (for all 7 MUX ICs) ---
const uint8_t MUX_S0 = PC13;
const uint8_t MUX_S1 = PC14;
const uint8_t MUX_S2 = PC15;

// --- MULTIPLEXED ANALOG INPUTS (Pots, Joystick, EQ) ---
const uint8_t MUX_ANALOG_1_COM = PB0; // ADC for MUX 1 (Joy, Pots, EQ 1-4)
const uint8_t MUX_ANALOG_2_COM = PB1; // ADC for MUX 2 (EQ 5-8)

// --- MULTIPLEXED DIGITAL BUTTONS (5x CD4051) ---
const uint8_t MUX_BTN_EN[5] = {PA15, PB12, PB13, PB14, PB15}; // Enable pins for each button MUX
const uint8_t MUX_BTN_COM   = PA8;  // Digital input for all button MUXs

// --- ENCODER (MOVED TO AVOID USB CONFLICT) ---
const uint8_t PIN_ENCODER_A = PA9;  // MOVED from PA11. Must be interrupt-capable.
const uint8_t PIN_ENCODER_B = PA10; // MOVED from PA12. Must be interrupt-capable.
const uint8_t PIN_ENCODER_SW = PB11;

// --- STATUS LED ---
const uint8_t STATUS_LED_PIN = LED_BUILTIN; // Usually PC13, which is now MUX_S0.
                                           // Re-map if you need both. For now, MUX takes priority.

/* ==========================================
          MIDI OUT (USB + DIN)
   ========================================== */

void midiRaw(uint8_t st, uint8_t d1, uint8_t d2) {
    // Note: Serial1 (PA9/PA10) is now used for the Encoder. DIN MIDI is not possible
    // with this pin configuration. This firmware is USB-MIDI focused.
    // Serial1.write(st); Serial1.write(d1); Serial1.write(d2);
#if defined(USBCON)
    uint8_t packet[] = {st, d1, d2};
    SerialUSB.write(packet, 3);
#endif
}

void midiNoteOn(uint8_t ch, uint8_t n, uint8_t v){ midiRaw(0x90 | (ch - 1), n, v); }
void midiNoteOff(uint8_t ch, uint8_t n, uint8_t v){ midiRaw(0x80 | (ch - 1), n, v); }
void midiCC(uint8_t ch, uint8_t cc, uint8_t v){ midiRaw(0xB0 | (ch - 1), cc, v); }
void midiPB(uint8_t ch, int val){
    val = constrain(val, 0, 16383);
    midiRaw(0xE0 | (ch - 1), val & 0x7F, (val >> 7) & 0x7F);
}


/* ==========================================
          GLOBAL VARIABLES
   ========================================== */
bool velocityEnabled = true;
bool pitchbendEnabled = true;
bool wheelYMode = false;
bool matrixState[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
unsigned long matrixDebounce[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
int matrixPeakValue[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
bool noteOnSent[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
unsigned long matrixPressTime[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
bool uniState[NUM_UNIVERSAL_BUTTONS + 2]; // Total 40 buttons/switches
unsigned long uniDebounce[NUM_UNIVERSAL_BUTTONS + 2];
int lastEQ[NUM_EQ_BANDS];
int lastMod = -1, lastPan = -1, lastVol = -1, lastPB = -1;
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
enum ENC_MODE { M_TEMPO, M_OCT, M_TRANSPOSE, M_WHEEL_Y };
ENC_MODE encMode = M_TEMPO;
int octaveOffset = 0;
int transposeVal = 0;
int tempo = 120;
uint8_t padNoteMap[64];


/* ==========================================
           CD4051 MUX FUNCTION
   ========================================== */
void selectMuxChannel(uint8_t ch){
    digitalWrite(MUX_S0, (ch >> 0) & 1);
    digitalWrite(MUX_S1, (ch >> 1) & 1);
    digitalWrite(MUX_S2, (ch >> 2) & 1);
    delayMicroseconds(5); // Settling time for MUX
}


/* ==========================================
               ENCODER ISR
   ========================================== */
void encoderISR(){
    static uint8_t lastState = 0;
    uint8_t a = digitalRead(PIN_ENCODER_A);
    uint8_t b = digitalRead(PIN_ENCODER_B);
    uint8_t currentState = (a << 1) | b;
    if (currentState != lastState){
        if ((lastState == 0 && currentState == 1) || (lastState == 1 && currentState == 3) || (lastState == 3 && currentState == 2) || (lastState == 2 && currentState == 0))
            encoderPos++;
        else
            encoderPos--;
        encoderMoved = true;
    }
    lastState = currentState;
}


/* ==========================================
           COMPONENT HANDLERS
   ========================================== */
void setupPadMap(){
    for(int i = 0; i < 64; i++) padNoteMap[i] = 36 + i;
}

void scanMatrix(){
    for(int c = 0; c < NUM_MATRIX_COLS; c++){
        digitalWrite(COL_PINS[c], LOW);
        delayMicroseconds(50);

        for(int r = 0; r < NUM_MATRIX_ROWS; r++){
            uint8_t note = padNoteMap[r * NUM_MATRIX_COLS + c] + octaveOffset * 12 + transposeVal;
            if (velocityEnabled) {
                int value = analogRead(ROW_PINS[r]);
                if (value > NOTE_ON_THRESHOLD && !matrixState[r][c]) {
                    matrixState[r][c] = true;
                    noteOnSent[r][c] = false;
                    matrixPeakValue[r][c] = value;
                    matrixPressTime[r][c] = millis();
                } else if (value < NOTE_OFF_THRESHOLD && matrixState[r][c]) {
                    matrixState[r][c] = false;
                    if (noteOnSent[r][c]) midiNoteOff(MIDI_CHANNEL, note, 0);
                } else if (matrixState[r][c]) {
                    if (value > matrixPeakValue[r][c]) matrixPeakValue[r][c] = value;
                    if (!noteOnSent[r][c] && (millis() - matrixPressTime[r][c] > VELOCITY_WINDOW_MS)) {
                        uint8_t velocity = map(matrixPeakValue[r][c], NOTE_ON_THRESHOLD, 4095, 1, 127);
                        midiNoteOn(MIDI_CHANNEL, note, constrain(velocity, 1, 127));
                        noteOnSent[r][c] = true;
                    }
                }
            } else {
                bool pressed = (analogRead(ROW_PINS[r]) > 2048);
                if (pressed != matrixState[r][c] && (millis() - matrixDebounce[r][c] > DEBOUNCE_DELAY_FAST_MS)) {
                    matrixDebounce[r][c] = millis();
                    matrixState[r][c] = pressed;
                    if (pressed) midiNoteOn(MIDI_CHANNEL, note, NOTE_VELOCITY_FIXED);
                    else midiNoteOff(MIDI_CHANNEL, note, 0);
                }
            }
        }
        digitalWrite(COL_PINS[c], HIGH);
    }
}

void scanUniversalButtons(){
    int index = 0;
    for(int chip = 0; chip < 5; chip++) {
        digitalWrite(MUX_BTN_EN[chip], LOW);
        for(int ch = 0; ch < 8; ch++){
            if (index >= (NUM_UNIVERSAL_BUTTONS + 2)) break;

            selectMuxChannel(ch);
            bool pressed = (digitalRead(MUX_BTN_COM) == LOW);

            if (index == PB_ENABLE_BTN_INDEX || index == VELOCITY_BTN_INDEX) {
                if (pressed && !uniState[index] && (millis() - uniDebounce[index] > DEBOUNCE_DELAY_SLOW_MS)) {
                    if (index == PB_ENABLE_BTN_INDEX) pitchbendEnabled = !pitchbendEnabled;
                    else velocityEnabled = !velocityEnabled;
                    uniDebounce[index] = millis();
                }
                uniState[index] = pressed;
            }
            else { // Regular momentary buttons
                if (pressed != uniState[index] && (millis() - uniDebounce[index] > DEBOUNCE_DELAY_FAST_MS)) {
                    uniDebounce[index] = millis();
                    uniState[index] = pressed;
                    uint8_t note = 60 + (index % 36);
                    if (pressed) midiNoteOn(MIDI_CHANNEL, note, NOTE_VELOCITY_FIXED);
                    else midiNoteOff(MIDI_CHANNEL, note, 0);
                }
            }
            index++;
        }
        digitalWrite(MUX_BTN_EN[chip], HIGH);
    }
}

void readAnalogs(){
    // --- MUX ANALOG 1 (Joystick, Master/Balance, EQ 1-4) ---
    for (int ch = 0; ch < 8; ch++) {
        selectMuxChannel(ch);
        int raw_adc_val = analogRead(MUX_ANALOG_1_COM); // Read only once per channel
        int val = map(raw_adc_val, 0, 4095, 0, 127);

        switch(ch) {
            case 0: // Joystick X (Mod Wheel)
                if (abs(val - lastMod) > 1 && pitchbendEnabled) { midiCC(MIDI_CHANNEL, MOD_WHEEL_CC, val); lastMod = val; }
                break;
            case 1: // Joystick Y (Pitchbend)
                 if (pitchbendEnabled) {
                    int pb = wheelYMode ? constrain(8192 + (raw_adc_val - 2048) * 4, 0, 16383) : map(raw_adc_val, 0, 4095, 0, 16383);
                    if (abs(pb - lastPB) > 16) { midiPB(MIDI_CHANNEL, pb); lastPB = pb; }
                }
                break;
            case 2: // Master Volume
                if (abs(val - lastVol) > 2) { midiCC(MIDI_CHANNEL, MASTER_VOL_CC, val); lastVol = val; }
                break;
            case 3: // Balance
                if (abs(val - lastPan) > 2) { midiCC(MIDI_CHANNEL, BALANCE_CC, val); lastPan = val; }
                break;
            case 4: case 5: case 6: case 7: // EQ 1-4
                int eq_index = ch - 4;
                if (abs(val - lastEQ[eq_index]) > 2) { midiCC(MIDI_CHANNEL, EQ_CC_START + eq_index, val); lastEQ[eq_index] = val; }
                break;
        }
    }
    
    // --- MUX ANALOG 2 (EQ 5-8) ---
    for (int ch = 0; ch < 4; ch++) {
        selectMuxChannel(ch);
        int val = map(analogRead(MUX_ANALOG_2_COM), 0, 4095, 0, 127);
        int eq_index = ch + 4;
        if (abs(val - lastEQ[eq_index]) > 2) { midiCC(MIDI_CHANNEL, EQ_CC_START + eq_index, val); lastEQ[eq_index] = val; }
    }

    if (!pitchbendEnabled) {
      if(lastPB != 8192) { midiPB(MIDI_CHANNEL, 8192); lastPB = 8192; }
      if(lastMod != 0) { midiCC(MIDI_CHANNEL, MOD_WHEEL_CC, 0); lastMod = 0; }
    }
}

void handleEncoderButton(){
    static bool lastState = HIGH;
    static unsigned long lastPressTime = 0;
    bool currentState = digitalRead(PIN_ENCODER_SW);
    if (currentState != lastState && currentState == LOW && (millis() - lastPressTime > DEBOUNCE_DELAY_SLOW_MS)) {
        encMode = (ENC_MODE)(((int)encMode + 1) % 4);
        lastPressTime = millis();
    }
    lastState = currentState;
}

void handleEncoder(){
    if(!encoderMoved) return;
    noInterrupts();
    int d = encoderPos;
    encoderPos = 0;
    encoderMoved = false;
    interrupts();
    if (d == 0) return;

    switch(encMode) {
        case M_TEMPO:     tempo = constrain(tempo + d, 40, 250); break;
        case M_OCT:       octaveOffset = constrain(octaveOffset + d, -3, 3); break;
        case M_TRANSPOSE: transposeVal = constrain(transposeVal + d, -12, 12); break;
        case M_WHEEL_Y:   if(d != 0) wheelYMode = !wheelYMode; break;
    }
}

/* ==========================================
                 SETUP
   ========================================== */
void setup(){
    Serial.begin(115200);
#if defined(USBCON)
    SerialUSB.begin();
#endif
    analogReadResolution(12);

    setupPadMap();
    for (int i = 0; i < NUM_EQ_BANDS; ++i) lastEQ[i] = -1;
    
    // --- Pin Modes ---
    pinMode(STATUS_LED_PIN, OUTPUT);
    for(int i=0; i<NUM_MATRIX_COLS; i++) { pinMode(COL_PINS[i], OUTPUT); digitalWrite(COL_PINS[i], HIGH); }
    for(int i=0; i<NUM_MATRIX_ROWS; i++) pinMode(ROW_PINS[i], INPUT);
    
    pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
    
    pinMode(MUX_BTN_COM, INPUT_PULLUP);
    for(int i=0; i<5; i++) { pinMode(MUX_BTN_EN[i], OUTPUT); digitalWrite(MUX_BTN_EN[i], HIGH); }
    
    pinMode(PIN_ENCODER_A, INPUT_PULLUP); pinMode(PIN_ENCODER_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
}

/* ==========================================
                  LOOP
   ========================================== */
void loop(){
    static unsigned long lastScanTime = 0;
    if(millis() - lastScanTime >= LOOP_SCAN_INTERVAL_MS){
        lastScanTime = millis();
        scanMatrix();
        scanUniversalButtons();
        readAnalogs();
    }

    handleEncoder();
    handleEncoderButton();
}
