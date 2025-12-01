#include <Arduino.h>
#include <MIDI.h>
#include "usb_midi_stm32.h"

// Create MIDI instance
MIDI_CREATE_INSTANCE(USBMIDI, MIDIUSB, MIDI);

// ---- CONFIGURATIONS ----
#define NUM_ROWS 8
#define NUM_COLS 8

// Pin mapping (edit sesuai PCB)
uint8_t rowPins[NUM_ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
uint8_t colPins[NUM_COLS] = {PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14};

// MIDI LED activity indicator
#define LED_PIN PC13  

// Optional analog controls
#define PITCH_PIN PA8
#define MOD_PIN   PA9

// ---- VARIABLES ----
uint8_t keyState[NUM_ROWS][NUM_COLS];
unsigned long lastSendTime = 0;

// ---- FUNCTIONS ----
void sendNoteOn(byte note, byte velocity) {
  MIDI.sendNoteOn(note, velocity, 1);
  digitalWrite(LED_PIN, LOW);
}

void sendNoteOff(byte note) {
  MIDI.sendNoteOff(note, 0, 1);
  digitalWrite(LED_PIN, HIGH);
}

void scanMatrix() {
  for (int r = 0; r < NUM_ROWS; r++) {
    digitalWrite(rowPins[r], LOW);

    for (int c = 0; c < NUM_COLS; c++) {
      int state = digitalRead(colPins[c]);
      byte noteNumber = (r * NUM_COLS) + c + 36; // Start note C2 = 36

      if (state == LOW && keyState[r][c] == 0) {
        keyState[r][c] = 1;
        sendNoteOn(noteNumber, 100);
      }
      else if (state == HIGH && keyState[r][c] == 1) {
        keyState[r][c] = 0;
        sendNoteOff(noteNumber);
      }
    }

    digitalWrite(rowPins[r], HIGH);
  }
}

void sendAnalogControls() {
  int pitchVal = analogRead(PITCH_PIN) / 8; // Scale to 0–1023
  int modVal   = analogRead(MOD_PIN) / 8;

  MIDI.sendPitchBend(pitchVal, 1);
  MIDI.sendControlChange(1, modVal / 8, 1); // CC1 Modwheel
}

void setup() {
  // USB MIDI Init
  MIDI.begin(MIDI_CHANNEL_OMNI);
  
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Matrix Setup
  for (int r = 0; r < NUM_ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }

  for (int c = 0; c < NUM_COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  // Analog controls
  pinMode(PITCH_PIN, INPUT_ANALOG);
  pinMode(MOD_PIN, INPUT_ANALOG);
}

void loop() {
  scanMatrix();
  sendAnalogControls();
}
