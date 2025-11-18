/*
 * ===================================================================================
 * STM32 STANDALONE USB MIDI CONTROLLER FIRMWARE (V18 - BUTTON EXPANSION)
 * ===================================================================================
 *
 * AUTHOR: AI Assistant
 * DATE: 2024-08-11
 *
 * TARGET BOARD: STM32F103C6T6 (Blue Pill)
 *
 * --- V18 ARCHITECTURE REFINEMENT (BUTTON EXPANSION VIA TRADE-OFF) ---
 * 1.  FEATURE EXPANSION: The number of universal, mappable buttons has been
 *     INCREASED from 24 back to 32 by adding a fourth CD4051 multiplexer.
 * 2.  ENGINEERING TRADE-OFF: To free up a GPIO pin required to enable the new
 *     multiplexer, the dedicated 'Chord Tap Restart' toggle button has been
 *     REMOVED from its native pin (PB2).
 * 3.  RE-INTEGRATED FUNCTION: The 'Chord Tap Restart' on/off function is now
 *     handled by the LAST universal button (Button #32, index 31) within the
 *     multiplexed button matrix.
 * 4.  NEW PIN ROLE: The freed pin, PB2, is now repurposed as the enable pin
 *     for the new, fourth bank of universal buttons.
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
#define NUM_UNIVERSAL_BUTTONS 32 // INCREASED from 24
#define UART_BAUD_RATE 115200
#define CHORD_THRESHOLD 3

// --- Timing ---
const unsigned long DEBOUNCE_DELAY_MS = 20;
const unsigned long LOOP_SCAN_INTERVAL_MS = 1;
const unsigned long VELOCITY_WINDOW_MS = 25;
const unsigned long TAP_TIMEOUT_MS = 750;

// --- Thresholds ---
const int NOTE_ON_THRESHOLD = 100;
const int NOTE_OFF_THRESHOLD = 50;

/* ==========================================
   STM32F103C6T6 PIN DEFINITIONS (V18)
   ========================================== */
// --- DIRECT NATIVE PINS (HIGHEST PRIORITY) ---
const uint8_t ROW_PINS[NUM_MATRIX_ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
const uint8_t PIN_PITCH_BEND_X = PB0;
const uint8_t PIN_PITCH_BEND_Y = PB1;
const uint8_t COL_PINS[NUM_MATRIX_COLS] = {PB12, PB13, PB14, PB15, PC13, PC14, PC15, PA15};
const uint8_t PIN_PB_ENABLE_BTN = PB6;
const uint8_t PIN_VELOCITY_BTN = PB7;
// PIN_CHORD_TAP_BTN on PB2 has been removed. PB2 is now a MUX enable.
const uint8_t PIN_ENCODER_A = PB8;
const uint8_t PIN_ENCODER_B = PB9;
const uint8_t PIN_ENCODER_SW = PB11;

// --- MULTIPLEXED PINS ---
const uint8_t MUX_S0 = PB3, MUX_S1 = PB4, MUX_S2 = PB5;
const uint8_t MUX_BTN_COM = PB10;
const uint8_t MUX_BTN_EN[4] = {PA8, PA13, PA14, PB2}; // 4th MUX enable added on PB2


/* ==========================================
          UART COMMUNICATION
   ========================================== */
#define PACKET_START_BYTE 0xFE
enum PacketType { NOTE_ON = 0x90, NOTE_OFF = 0x80, CONTROL_CHANGE = 0xB0, PITCH_BEND = 0xE0, SYSTEM_REALTIME = 0xF0 };

void sendUartPacket(PacketType type, uint8_t d1, uint8_t d2) {
    uint8_t checksum = (type + d1 + d2) & 0xFF;
    Serial1.write(PACKET_START_BYTE); Serial1.write((uint8_t)type); Serial1.write(d1); Serial1.write(d2); Serial1.write(checksum);
}
void sendUartPitchBendPacket(int val) {
    val = constrain(val, 0, 16383); uint8_t d1 = val & 0x7F; uint8_t d2 = (val >> 7) & 0x7F;
    uint8_t checksum = (PITCH_BEND + d1 + d2) & 0xFF;
    Serial1.write(PACKET_START_BYTE); Serial1.write((uint8_t)PITCH_BEND); Serial1.write(d1); Serial1.write(d2); Serial1.write(checksum);
}
void sendUartSystemPacket(uint8_t sys_msg) {
    uint8_t checksum = (SYSTEM_REALTIME + sys_msg) & 0xFF;
    Serial1.write(PACKET_START_BYTE); Serial1.write((uint8_t)SYSTEM_REALTIME); Serial1.write(sys_msg); Serial1.write(checksum);
}

/* ==========================================
          DUAL-OUTPUT MIDI FUNCTIONS
   ========================================== */
void midiNoteOn(uint8_t ch, uint8_t n, uint8_t v){ midi_usb.sendNoteOn(ch, n, v); sendUartPacket(NOTE_ON, n, v); }
void midiNoteOff(uint8_t ch, uint8_t n, uint8_t v){ midi_usb.sendNoteOff(ch, n, v); sendUartPacket(NOTE_OFF, n, v); }
void midiCC(uint8_t ch, uint8_t cc, uint8_t v){ midi_usb.sendControlChange(ch, cc, v); sendUartPacket(CONTROL_CHANGE, cc, v); }
void midiPB(uint8_t ch, int val){ midi_usb.sendPitchBend(ch, val); sendUartPitchBendPacket(val); }
void midiStart(){ midi_usb.sendRealTime(USBSystemRealTime::Start); sendUartSystemPacket(0xFA); }


/* ==========================================
          GLOBAL VARIABLES
   ========================================== */
// --- System State ---
bool velocityEnabled = true;
bool pitchbendEnabled = true;
bool chordTapRestartEnabled = false;
int lastMod = -1, lastPB = -1;
uint8_t padNoteMap[NUM_MATRIX_ROWS * NUM_MATRIX_COLS];

// --- Pad Matrix State ---
bool matrixState[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
int matrixPeakValue[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
bool noteOnSent[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];
unsigned long matrixPressTime[NUM_MATRIX_ROWS][NUM_MATRIX_COLS];

// --- Button States ---
bool uniState[NUM_UNIVERSAL_BUTTONS];
unsigned long uniDebounce[NUM_UNIVERSAL_BUTTONS];
bool pbBtnState = false, velBtnState = false;
unsigned long pbBtnDebounce = 0, velBtnDebounce = 0;

// --- Encoder State ---
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
enum ENC_MODE { M_TEMPO, M_OCT, M_TRANSPOSE, M_PB_RANGE, M_CHORD_TAP };
ENC_MODE encMode = M_TEMPO;
int octaveOffset = 0;
int transposeVal = 0;
int tempo = 120;
int pitchBendRangePercent = 100;
int chordTapThreshold = 2;

// --- Chord Tap Restart State ---
uint8_t heldKeysCount = 0;
bool isChordCurrentlyHeld = false;
int currentChordTapCount = 0;
unsigned long lastTapTime = 0;


/* ==========================================
           HELPER & ISR FUNCTIONS
   ========================================== */
void selectMuxChannel(uint8_t ch){
    digitalWrite(MUX_S0, (ch >> 0) & 1); digitalWrite(MUX_S1, (ch >> 1) & 1); digitalWrite(MUX_S2, (ch >> 2) & 1);
    delayMicroseconds(5);
}

void encoderISR(){
    static uint8_t lastState = 0; uint8_t a = digitalRead(PIN_ENCODER_A); uint8_t b = digitalRead(PIN_ENCODER_B);
    uint8_t currentState = (a << 1) | b;
    if (currentState != lastState){
        if ((lastState == 0 && currentState == 1) || (lastState == 1 && currentState == 3) || (lastState == 3 && currentState == 2) || (lastState == 2 && currentState == 0)) encoderPos++;
        else encoderPos--;
        encoderMoved = true;
    }
    lastState = currentState;
}

void encoderSW_ISR(){
    static unsigned long last_interrupt_time = 0; unsigned long interrupt_time = millis();
    if (interrupt_time - last_interrupt_time > 200) {
        encMode = (ENC_MODE)(((int)encMode + 1) % 5);
    }
    last_interrupt_time = interrupt_time;
}

/* ==========================================
      ADJUSTABLE CHORD TAP LOGIC
   ========================================== */
void handleChordTapLogic() {
    if (!chordTapRestartEnabled) {
        currentChordTapCount = 0;
        return;
    }

    if (heldKeysCount >= CHORD_THRESHOLD && !isChordCurrentlyHeld) {
        isChordCurrentlyHeld = true;
        unsigned long now = millis();

        if (now - lastTapTime > TAP_TIMEOUT_MS) {
            currentChordTapCount = 1;
        } else {
            currentChordTapCount++;
        }
        lastTapTime = now;

        if (currentChordTapCount >= chordTapThreshold) {
            midiStart();
            currentChordTapCount = 0;
        }
    }
    else if (heldKeysCount < CHORD_THRESHOLD && isChordCurrentlyHeld) {
        isChordCurrentlyHeld = false;
    }
}


/* ==========================================
           COMPONENT HANDLERS
   ========================================== */
void setupPadMap(){ for(int i=0; i < (NUM_MATRIX_ROWS*NUM_MATRIX_COLS); i++) padNoteMap[i] = 36 + i; }

void scanMatrix(){
    heldKeysCount = 0;
    for(int c = 0; c < NUM_MATRIX_COLS; c++){
        digitalWrite(COL_PINS[c], LOW);
        delayMicroseconds(50);

        for(int r = 0; r < NUM_MATRIX_ROWS; r++){
            int value = analogRead(ROW_PINS[r]);
            uint8_t note = padNoteMap[r * NUM_MATRIX_COLS + c] + octaveOffset * 12 + transposeVal;

            if (value > NOTE_ON_THRESHOLD) {
                if (!matrixState[r][c]) {
                    matrixState[r][c] = true;
                    if (velocityEnabled) {
                        noteOnSent[r][c] = false;
                        matrixPeakValue[r][c] = value;
                        matrixPressTime[r][c] = millis();
                    } else {
                        midiNoteOn(0, note, 127);
                    }
                }
                heldKeysCount++;
            } else if (value < NOTE_OFF_THRESHOLD) {
                if (matrixState[r][c]) {
                    matrixState[r][c] = false;
                    if (velocityEnabled) {
                        if (noteOnSent[r][c]) midiNoteOff(0, note, 0);
                    } else {
                        midiNoteOff(0, note, 0);
                    }
                }
            }

            if (velocityEnabled && matrixState[r][c] && !noteOnSent[r][c]) {
                unsigned long timeHeld = millis() - matrixPressTime[r][c];
                if (value > matrixPeakValue[r][c]) matrixPeakValue[r][c] = value;
                if (timeHeld > VELOCITY_WINDOW_MS) {
                    int velocity = map(matrixPeakValue[r][c], NOTE_ON_THRESHOLD, 4095, 1, 127);
                    midiNoteOn(0, note, constrain(velocity, 1, 127));
                    noteOnSent[r][c] = true;
                }
            }
        }
        digitalWrite(COL_PINS[c], HIGH);
    }
    handleChordTapLogic();
}

void readDirectButtons(){
    bool currentPbBtnState = digitalRead(PIN_PB_ENABLE_BTN) == LOW;
    if (currentPbBtnState != pbBtnState && millis() - pbBtnDebounce > DEBOUNCE_DELAY_MS){
        pbBtnState = currentPbBtnState; pbBtnDebounce = millis();
        if(pbBtnState) pitchbendEnabled = !pitchbendEnabled;
    }

    bool currentVelBtnState = digitalRead(PIN_VELOCITY_BTN) == LOW;
    if (currentVelBtnState != velBtnState && millis() - velBtnDebounce > DEBOUNCE_DELAY_MS){
        velBtnState = currentVelBtnState; velBtnDebounce = millis();
        if(velBtnState) velocityEnabled = !velocityEnabled;
    }
}

void handleMuxButtons(){
    for(int i=0; i < sizeof(MUX_BTN_EN); i++){
      digitalWrite(MUX_BTN_EN[i], LOW);
      for(int j=0; j < 8; j++){
          selectMuxChannel(j);
          int btnIndex = i * 8 + j;
          if(btnIndex >= NUM_UNIVERSAL_BUTTONS) break;
          bool state = digitalRead(MUX_BTN_COM) == LOW;

          if(state != uniState[btnIndex] && millis() - uniDebounce[btnIndex] > DEBOUNCE_DELAY_MS){
              uniState[btnIndex] = state;
              uniDebounce[btnIndex] = millis();
              
              if (btnIndex == NUM_UNIVERSAL_BUTTONS - 1) { // Last button is Chord Tap Toggle
                  if(state) { // On press
                      chordTapRestartEnabled = !chordTapRestartEnabled;
                  }
              } else { // All other buttons send MIDI notes
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
        noInterrupts(); int delta = encoderPos / 4; encoderPos %= 4; encoderMoved = false; interrupts();
        if (delta != 0) {
            switch(encMode){
                case M_TEMPO: tempo = constrain(tempo + delta, 40, 250); break;
                case M_OCT: octaveOffset = constrain(octaveOffset + delta, -3, 3); break;
                case M_TRANSPOSE: transposeVal = constrain(transposeVal + delta, -12, 12); break;
                case M_PB_RANGE: pitchBendRangePercent = constrain(pitchBendRangePercent + (delta * 5), 10, 200); break;
                case M_CHORD_TAP: chordTapThreshold = constrain(chordTapThreshold + delta, 1, 8); break;
            }
        }
    }
}

void readDirectAnalogs() {
    if(pitchbendEnabled) {
        int pbX = analogRead(PIN_PITCH_BEND_X); int pbY = analogRead(PIN_PITCH_BEND_Y);
        int currentMod = map(pbY, 0, 4095, 0, 127);
        if (abs(currentMod - lastMod) > 1) { midiCC(0, 1, currentMod); lastMod = currentMod; }
        
        long centeredValue = pbX - 2048;
        centeredValue = (centeredValue * pitchBendRangePercent) / 100;
        int currentPB = constrain(centeredValue + 8192, 0, 16383);
       
        if (abs(currentPB - lastPB) > 2) { midiPB(0, currentPB); lastPB = currentPB; }
    } else {
      if (lastPB != 8192) { midiPB(0, 8192); lastPB = 8192; }
      if (lastMod != 0) { midiCC(0, 1, 0); lastMod = 0; }
    }
}

/* ==========================================
               MAIN SETUP & LOOP
   ========================================== */
void setup() {
    // Direct Native Pins
    for(auto p : ROW_PINS) { pinMode(p, INPUT_ANALOG); }
    for(auto p : COL_PINS) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
    pinMode(PIN_PITCH_BEND_X, INPUT_ANALOG); pinMode(PIN_PITCH_BEND_Y, INPUT_ANALOG);
    pinMode(PIN_PB_ENABLE_BTN, INPUT_PULLUP); 
    pinMode(PIN_VELOCITY_BTN, INPUT_PULLUP);
    pinMode(PIN_ENCODER_A, INPUT_PULLUP); pinMode(PIN_ENCODER_B, INPUT_PULLUP);
    pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
    
    // Multiplexed Pins
    pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_BTN_COM, INPUT_PULLUP);
    for(auto p : MUX_BTN_EN) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_SW), encoderSW_ISR, FALLING);

    Serial1.begin(UART_BAUD_RATE, SERIAL_8N1, PA10, PA9);
    midi_usb.begin();
    setupPadMap();
}

void loop() {
    static unsigned long lastScanTime = 0;

    if (millis() - lastScanTime >= LOOP_SCAN_INTERVAL_MS) {
        lastScanTime = millis();
        scanMatrix();
        readDirectButtons();
        handleMuxButtons();
        handleEncoder();
        readDirectAnalogs();
    }
    
    midi_usb.Recv();
}
