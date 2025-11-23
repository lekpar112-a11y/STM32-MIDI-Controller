#include <Arduino.h>
#include <EEPROM.h>
#include <USBMIDI.h>

// ====== PIN DEFINISI ======
#define BUTTON_PIN PA0

// ====== EEPROM ADDRESS ======
#define EEPROM_ADDR_NOTE 0   // Menyimpan nada terakhir
byte midiNote = 60;          // Default C4

USBMIDI USBmidi;

// ====== SETUP ======
void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    EEPROM.begin();                 // Start EEPROM Emulation
    midiNote = EEPROM.read(EEPROM_ADDR_NOTE);  
    if (midiNote < 10 || midiNote > 120) midiNote = 60;  // Validasi
    USBmidi.begin();
}

// ====== LOOP ======
void loop() {
    static bool lastState = HIGH;
    bool currentState = digitalRead(BUTTON_PIN);

    if (lastState == HIGH && currentState == LOW) {
        USBmidi.sendNoteOn(midiNote, 127, 1);
        EEPROM.write(EEPROM_ADDR_NOTE, midiNote);
        EEPROM.commit();
    }
    if (lastState == LOW && currentState == HIGH) {
        USBmidi.sendNoteOff(midiNote, 0, 1);
    }
    lastState = currentState;
}
