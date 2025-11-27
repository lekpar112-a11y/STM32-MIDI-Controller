#include <Arduino.h>
#include "USBMIDI.h"

USBMIDI midi;

void setup() {
    midi.begin();
}

void loop() {
    midi.sendNoteOn(60, 127, 1);
    delay(500);
    midi.sendNoteOff(60, 127, 1);
    delay(500);
}
