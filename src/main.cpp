#include <Arduino.h>
#include <USBComposite.h>

#define ROWS 8
#define COLS 16

// Pin STM32 universal (bisa diubah nanti via konfigurasi)
uint8_t rowPins[ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
uint8_t colPins[COLS] = {PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15,
                         PA8, PA9, PA10, PA11, PA12, PA15, PB3, PB4};

bool keyState[ROWS][COLS];
bool lastState[ROWS][COLS];

USBMIDI midi;

// MIDI Note Base
int baseNote = 36; // C2 default keyboard

// Anti‑ghosting function
bool isGhosting(int r, int c) {
    int countRow = 0;
    int countCol = 0;

    for (int x = 0; x < COLS; x++)
        if (keyState[r][x]) countRow++;

    for (int y = 0; y < ROWS; y++)
        if (keyState[y][c]) countCol++;

    return (countRow > 1 && countCol > 1);
}

void setupMatrixPins() {
    for (int r = 0; r < ROWS; r++) {
        pinMode(rowPins[r], OUTPUT);
        digitalWrite(rowPins[r], LOW);
    }
    for (int c = 0; c < COLS; c++) {
        pinMode(colPins[c], INPUT_PULLDOWN);
    }
}

void scanMatrix() {
    for (int r = 0; r < ROWS; r++) {
        digitalWrite(rowPins[r], HIGH);

        delayMicroseconds(250);

        for (int c = 0; c < COLS; c++) {
            keyState[r][c] = digitalRead(colPins[c]);
        }

        digitalWrite(rowPins[r], LOW);
    }
}

void sendMIDI() {
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (keyState[r][c] != lastState[r][c]) {
                
                // Cegah ghosting kirim note random
                if (keyState[r][c] && isGhosting(r, c)) continue;

                int note = baseNote + (r * COLS + c);

                if (keyState[r][c])
                    midi.sendNoteOn(1, note, 127);
                else
                    midi.sendNoteOff(1, note, 0);

                lastState[r][c] = keyState[r][c];
            }
        }
    }
}

void setup() {
    USBComposite.setProductString("Universal Keyboard Matrix MIDI");
    midi.begin();
    setupMatrixPins();
}

void loop() {
    scanMatrix();
    sendMIDI();
    delay(3);
}
