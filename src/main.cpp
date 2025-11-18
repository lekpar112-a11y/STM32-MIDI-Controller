/*
 * ===================================================================================
 * STM32 STANDALONE USB MIDI CONTROLLER FIRMWARE (V10 - USB NATIVE)
 * ===================================================================================
 *
 * AUTHOR: AI Assistant
 * DATE: 2024-08-04
 *
 * TARGET BOARD: STM32F103C6T6 (Blue Pill)
 *
 * --- V10 ARCHITECTURE OVERHAUL ---
 * 1.  NEW ARCHITECTURE: The STM32 is now a fully-featured, standalone USB MIDI device.
 *     It no longer relies on the ESP32 for USB connectivity.
 * 2.  DUAL OUTPUT: The firmware sends MIDI data to TWO destinations simultaneously:
 *     a) To the host computer via the native USB port.
 *     b) To the UART (Serial1) port.
 * 3.  MODULAR DESIGN: This allows the ESP32 to function as an optional "BLE MIDI
 *     Expansion Module". You can use the STM32 on its own via USB, or connect
 *     the ESP32 to the UART pins to add wireless capability.
 * 4.  IMPLEMENTATION: The USBComposite library is used to create a native USB MIDI
 *     device. The core MIDI functions (midiNoteOn, etc.) have been updated to call
 *     both the USB MIDI send function and the UART packet send function.
 */

#include <Arduino.h>
#include <USBComposite.h>

/* ==========================================
   USB MIDI CONFIGURATION
   ========================================== */
USBMIDI midi_usb;

/* ==========================================
   CONSTANTS & CONFIGURATION
   ========================================== */
#define NUM_MATRIX_COLS 8
#define NUM_MATRIX_ROWS 8
#define NUM_UNIVERSAL_BUTTONS 38
#define NUM_EQ_BANDS 8
#define UART_BAUD_RATE 115200

// --- Timing ---
const unsigned long DEBOUNCE_DELAY_FAST_MS = 10;
const unsigned long DEBOUNCE_DELAY_SLOW_MS = 150;
const unsigned long LOOP_SCAN_INTERVAL_MS = 2;
const unsigned long VELOCITY_WINDOW_MS = 25;
const unsigned long LED_BLINK_INTERVAL_MS = 500;

// --- Thresholds ---
const int NOTE_ON_THRESHOLD = 100;
const int NOTE_OFF_THRESHOLD = 50;

// --- MUX Button Indices ---
const uint8_t PB_ENABLE_BTN_INDEX = 38;
const uint8_t VELOCITY_BTN_INDEX  = 39;

/* ==========================================
   STM32F103C6T6 PIN DEFINITIONS
   ========================================== */
const uint8_t ROW_PINS[NUM_MATRIX_ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t MATRIX_MUX_S0 = PB3;
const uint8_t MATRIX_MUX_S1 = PB4;
const uint8_t MATRIX_MUX_S2 = PB5;
const uint8_t MATRIX_MUX_DRIVE = PB6;
const uint8_t MUX_S0 = PC13, MUX_S1 = PC14, MUX_S2 = PC15;
const uint8_t MUX_ANALOG_1_COM = PB0, MUX_ANALOG_2_COM = PB1;
const uint8_t MUX_BTN_EN[5] = {PA15, PB12, PB13, PB14, PB15};
const uint8_t MUX_BTN_COM   = PA8;
const uint8_t PIN_ENCODER_A = PB8;
const uint8_t PIN_ENCODER_B = PB9;
const uint8_t PIN_ENCODER_SW = PB11;
const uint8_t STATUS_LED_PIN = PB2;

/* ==========================================
          UART COMMUNICATION PROTOCOL (for optional ESP32 BLE module)
   ========================================== */
#define PACKET_START_BYTE 0xFE
enum PacketType {
  NOTE_ON = 0x90,
  NOTE_OFF = 0x80,
  CONTROL_CHANGE = 0xB0,
  PITCH_BEND = 0xE0
};

void sendUartPacket(PacketType type, uint8_t d1, uint8_t d2) {
    uint8_t checksum = (type + d1 + d2) & 0xFF;
    Serial1.write(PACKET_START_BYTE);
    Serial1.write((uint8_t)type);
    Serial1.write(d1);
    Serial1.write(d2);
    Serial1.write(checksum);
}

void sendUartPitchBendPacket(int val) {
    val = constrain(val, 0, 16383);
    uint8_t d1 = val & 0x7F;
    uint8_t d2 = (val >> 7) & 0x7F;
    uint8_t checksum = (PITCH_BEND + d1 + d2) & 0xFF;
    Serial1.write(PACKET_START_BYTE);
    Serial1.write((uint8_t)PITCH_BEND);
    Serial1.write(d1);
    Serial1.write(d2);
    Serial1.write(checksum);
}

/* ==========================================
          DUAL-OUTPUT MIDI FUNCTIONS
   ========================================== */
// These functions now send MIDI data to both USB and UART
void midiNoteOn(uint8_t ch, uint8_t n, uint8_t v){
    midi_usb.sendNoteOn(ch, n, v);
    sendUartPacket(NOTE_ON, n, v);
}
void midiNoteOff(uint8_t ch, uint8_t n, uint8_t v){
    midi_usb.sendNoteOff(ch, n, v);
    sendUartPacket(NOTE_OFF, n, v);
}
void midiCC(uint8_t ch, uint8_t cc, uint8_t v){
    midi_usb.sendControlChange(ch, cc, v);
    sendUartPacket(CONTROL_CHANGE, cc, v);
}
void midiPB(uint8_t ch, int val){
    midi_usb.sendPitchBend(ch, val);
    sendUartPitchBendPacket(val);
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
bool uniState[40];
unsigned long uniDebounce[40];
int lastEQ[NUM_EQ_BANDS];
int lastMod = -1, lastPan = -1, lastVol = -1, lastPB = -1;
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
enum ENC_MODE { M_TEMPO, M_OCT, M_TRANSPOSE, M_WHEEL_Y, M_PB_RANGE };
ENC_MODE encMode = M_TEMPO;
int octaveOffset = 0;
int transposeVal = 0;
int tempo = 120;
int pitchBendRangePercent = 100; // Range: 10% to 200%
uint8_t padNoteMap[NUM_MATRIX_ROWS * NUM_MATRIX_COLS];


/* ==========================================
           MUX SELECTION FUNCTIONS
   ========================================== */
void selectMuxChannel(uint8_t ch){
    digitalWrite(MUX_S0, (ch >> 0) & 1);
    digitalWrite(MUX_S1, (ch >> 1) & 1);
    digitalWrite(MUX_S2, (ch >> 2) & 1);
    delayMicroseconds(5);
}

void selectMatrixColumnMux(uint8_t col){
    digitalWrite(MATRIX_MUX_S0, (col >> 0) & 1);
    digitalWrite(MATRIX_MUX_S1, (col >> 1) & 1);
    digitalWrite(MATRIX_MUX_S2, (col >> 2) & 1);
    delayMicroseconds(5);
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
    for(int i = 0; i < (NUM_MATRIX_ROWS * NUM_MATRIX_COLS); i++) padNoteMap[i] = 36 + i;
}

void scanMatrix(){
    for(int c = 0; c < NUM_MATRIX_COLS; c++){
        selectMatrixColumnMux(c);
        digitalWrite(MATRIX_MUX_DRIVE, LOW);
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
                    if (noteOnSent[r][c]) {
                        midiNoteOff(0, note, 0);
                        noteOnSent[r][c] = false;
                    }
                }
                if (matrixState[r][c] && !noteOnSent[r][c]) {
                    unsigned long timeHeld = millis() - matrixPressTime[r][c];
                    if (value > matrixPeakValue[r][c]) {
                        matrixPeakValue[r][c] = value;
                    }
                    if (timeHeld > VELOCITY_WINDOW_MS) {
                        int velocity = map(matrixPeakValue[r][c], NOTE_ON_THRESHOLD, 4095, 1, 127);
                        velocity = constrain(velocity, 1, 127);
                        midiNoteOn(0, note, velocity);
                        noteOnSent[r][c] = true;
                    }
                }
            } else { // Fixed velocity
                int value = analogRead(ROW_PINS[r]);
                if (value > NOTE_ON_THRESHOLD && !matrixState[r][c]) {
                    matrixState[r][c] = true;
                    matrixDebounce[r][c] = millis();
                    midiNoteOn(0, note, 127);
                } else if (value < NOTE_OFF_THRESHOLD && matrixState[r][c]) {
                    matrixState[r][c] = false;
                    matrixDebounce[r][c] = millis();
                    midiNoteOff(0, note, 0);
                }
            }
        }
        digitalWrite(MATRIX_MUX_DRIVE, HIGH);
    }
}

void handleButtons(){
    for(int i=0; i < 5; i++){
      digitalWrite(MUX_BTN_EN[i], LOW);
      for(int j=0; j < 8; j++){
          selectMuxChannel(j);
          int btnIndex = i * 8 + j;
          if(btnIndex >= 40) break;
          bool state = digitalRead(MUX_BTN_COM) == LOW;
          if(state != uniState[btnIndex] && millis() - uniDebounce[btnIndex] > DEBOUNCE_DELAY_FAST_MS){
              uniState[btnIndex] = state;
              uniDebounce[btnIndex] = millis();
              if(btnIndex == PB_ENABLE_BTN_INDEX){ pitchbendEnabled = !pitchbendEnabled; }
              else if(btnIndex == VELOCITY_BTN_INDEX){ velocityEnabled = !velocityEnabled; }
              else {
                  uint8_t note = 12 + btnIndex;
                  if(state) midiNoteOn(0, note, 127);
                  else midiNoteOff(0, note, 0);
              }
          }
      }
      digitalWrite(MUX_BTN_EN[i], HIGH);
    }
}

void handleEncoder(){
    if(encoderMoved){
        noInterrupts();
        int delta = encoderPos / 4;
        encoderPos = encoderPos % 4;
        encoderMoved = false;
        interrupts();

        if (delta != 0) {
            switch(encMode){
                case M_TEMPO:
                    tempo = constrain(tempo + delta, 40, 250);
                    break;
                case M_OCT:
                    octaveOffset = constrain(octaveOffset + delta, -3, 3);
                    break;
                case M_TRANSPOSE:
                    transposeVal = constrain(transposeVal + delta, -12, 12);
                    break;
                case M_WHEEL_Y:
                    wheelYMode = !wheelYMode;
                    break;
                case M_PB_RANGE:
                    pitchBendRangePercent = constrain(pitchBendRangePercent + (delta * 5), 10, 200);
                    break;
            }
        }
    }
}

void encoderSW_ISR(){
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 200) {
        encMode = (ENC_MODE)(((int)encMode + 1) % 5);
    }
    last_interrupt_time = interrupt_time;
}

void setup() {
    for(auto p : ROW_PINS) pinMode(p, INPUT_ANALOG);
    pinMode(MATRIX_MUX_S0, OUTPUT); pinMode(MATRIX_MUX_S1, OUTPUT);
    pinMode(MATRIX_MUX_S2, OUTPUT); pinMode(MATRIX_MUX_DRIVE, OUTPUT);
    digitalWrite(MATRIX_MUX_DRIVE, HIGH);
    pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_ANALOG_1_COM, INPUT_ANALOG); pinMode(MUX_ANALOG_2_COM, INPUT_ANALOG);
    for(auto p : MUX_BTN_EN) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
    pinMode(MUX_BTN_COM, INPUT_PULLUP);
    pinMode(PIN_ENCODER_A, INPUT_PULLUP); pinMode(PIN_ENCODER_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), encoderSW_ISR, FALLING);
    pinMode(STATUS_LED_PIN, OUTPUT);

    // Initialize UART for optional BLE module
    Serial1.begin(UART_BAUD_RATE);
    
    // Initialize USB MIDI
    midi_usb.begin();
    
    setupPadMap();
}

void loop() {
    static unsigned long lastScanTime = 0;
    static unsigned long lastLedToggle = 0;

    if (millis() - lastScanTime >= LOOP_SCAN_INTERVAL_MS) {
        lastScanTime = millis();
        scanMatrix();
        handleButtons();
        handleEncoder();

        if(pitchbendEnabled) {
            selectMuxChannel(5);
            int pbX = analogRead(MUX_ANALOG_2_COM);
            selectMuxChannel(6);
            int pbY = analogRead(MUX_ANALOG_2_COM);

            int currentMod = map(pbY, 0, 4095, 0, 127);
            if (abs(currentMod - lastMod) > 1) {
                midiCC(0, 1, currentMod);
                lastMod = currentMod;
            }

            int currentPB;
            if (wheelYMode) {
                 long centeredValue = pbX - 2048;
                 centeredValue = (centeredValue * pitchBendRangePercent) / 100;
                 currentPB = constrain(centeredValue + 8192, 0, 16383);
            } else {
                long scaledRange = 16383;
                long center = 8192;
                long halfRange = (long)( (scaledRange / 2.0) * (pitchBendRangePercent / 100.0) );
                currentPB = map(pbX, 0, 4095, center - halfRange, center + halfRange);
                currentPB = constrain(currentPB, 0, 16383);
            }
            if (abs(currentPB - lastPB) > 2) {
                midiPB(0, currentPB);
                lastPB = currentPB;
            }
        } else {
          if (lastPB != 8192) { midiPB(0, 8192); lastPB = 8192; }
          if (lastMod != 0) { midiCC(0, 1, 0); lastMod = 0; }
        }
    }

    // This is required to process incoming USB messages
    midi_usb.Recv();

    if (millis() - lastLedToggle > LED_BLINK_INTERVAL_MS) {
        lastLedToggle = millis();
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
}
