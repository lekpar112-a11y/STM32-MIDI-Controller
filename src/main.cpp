#include <Arduino.h>
#include <BLEMidi.h> 

#define HW_MIDI_SERIAL  Serial1 
#define BLE_MIDI_SERIAL Serial2 
#define USB_MIDI_SERIAL Serial  

const uint8_t ROW_PINS[8] = {PA4, PA5, PA6, PA7, PB12, PB13, PB14, PB15};
const uint8_t COL_PINS[8] = {PC13, PC14, PC15, PA8, PA11, PA12, PB3, PB4};
const uint8_t PIN_PITCH_X     = PA0; 
const uint8_t PIN_PITCH_Y     = PA1; 
const uint8_t PIN_PITCH_WHEEL = PB0; 
const uint8_t PIN_POT_BALANCE = PB1; 
const uint8_t PIN_POT_MASTER  = PB10;
const uint8_t PIN_EQ[3] = {PA5, PA6, PA7};
const uint8_t PIN_ENCODER_A  = PB6;
const uint8_t PIN_ENCODER_B  = PB7;
const uint8_t PIN_ENCODER_SW = PB8;

const uint16_t MIDI_CHANNEL = 1; 
const uint8_t BASE_NOTE = 48; 
uint8_t padNoteMap[64];
int currentOctaveOffset = 0;
int transpose = 0;
uint8_t matrixState[8][8] = {0};
unsigned long lastDebounce[8][8] = {0};
volatile int encoderPos = 0;
volatile bool encoderMoved = false;
unsigned long lastScanTime = 0;

void sendMidiMessage(uint8_t* message, uint8_t size) {
    HW_MIDI_SERIAL.write(message, size);
    #if defined(USBCON)
        USB_MIDI_SERIAL.write(message, size);
    #endif
    if (BLE.isConnected()) { BLE.write(message, size); }
}
void midiSendNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
    uint8_t msg[] = {(uint8_t)(0x90 | ((ch-1) & 0x0F)), note, vel};
    sendMidiMessage(msg, sizeof(msg));
}
void midiSendNoteOff(uint8_t ch, uint8_t note, uint8_t vel) {
    uint8_t msg[] = {(uint8_t)(0x80 | ((ch-1) & 0x0F)), note, vel};
    sendMidiMessage(msg, sizeof(msg));
}
void midiSendCC(uint8_t ch, uint8_t cc, uint8_t val) {
    uint8_t msg[] = {(uint8_t)(0xB0 | ((ch-1) & 0x0F)), cc, val};
    sendMidiMessage(msg, sizeof(msg));
}

void setupPadMap() {
  for (int i=0; i<64; ++i) { padNoteMap[i] = BASE_NOTE + i; }
}
void scanMatrix() {
  for (int c=0; c<8; ++c) {
    pinMode(COL_PINS[c], OUTPUT); digitalWrite(COL_PINS[c], LOW);
    for (int r=0; r<8; ++r) {
      pinMode(ROW_PINS[r], INPUT_PULLUP);
      bool pressed = (digitalRead(ROW_PINS[r]) == LOW);
      if (pressed != matrixState[r][c] && millis() - lastDebounce[r][c] > 20) {
        lastDebounce[r][c] = millis();
        matrixState[r][c] = pressed;
        uint8_t note = padNoteMap[r*8 + c] + (currentOctaveOffset*12) + transpose;
        if (pressed) { midiSendNoteOn(MIDI_CHANNEL, note, 100); } 
        else { midiSendNoteOff(MIDI_CHANNEL, note, 0); }
      }
    }
    pinMode(COL_PINS[c], INPUT); 
  }
}
void readAnalogs() {
  midiSendCC(MIDI_CHANNEL, 7, map(analogRead(PIN_POT_MASTER), 0, 1023, 0, 127));
}
void encoderISR() { encoderMoved = true; }
void setupPins() {
  for(int i=0; i<8; ++i) { pinMode(ROW_PINS[i], INPUT_PULLUP); pinMode(COL_PINS[i], INPUT); }
  pinMode(PIN_ENCODER_A, INPUT_PULLUP); pinMode(PIN_ENCODER_B, INPUT_PULLUP); pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
}

void setup() {
    Serial.begin(115200);
    HW_MIDI_SERIAL.begin(31250);
    BLE_MIDI_SERIAL.begin(9600);
    BLE.begin("STM32 MIDI", BLE_MIDI_SERIAL);
    setupPins();
    setupPadMap();
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
}
void loop() {
    if (BLE.isConnected()) { BLE.poll(); }
    if (millis() - lastScanTime > 5) {
        lastScanTime = millis();
        scanMatrix();
        readAnalogs();
    }
}
