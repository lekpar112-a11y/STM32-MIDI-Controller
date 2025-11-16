#include <Arduino.h>
#include <MIDI.h>

MIDI_CREATE_DEFAULT_INSTANCE();

// =======================
// CD74HC4067 MULTIPLEXER
// =======================
#define MUX_SIG_PIN PA0   // Output multiplexer ke ADC STM32

// S0–S3 multiplexer
#define MUX_S0 PB12
#define MUX_S1 PB13
#define MUX_S2 PB14
#define MUX_S3 PB15

// Total 16 channel
int analogValues[16];

// =======================
// DIGITAL BUTTONS
// =======================
// 16 tombol digital (bebas kamu ganti)
uint8_t buttonPins[16] = {
    PA1, PA2, PA3, PA4,
    PA5, PA6, PA7, PB0,
    PB1, PB3, PB4, PB5,
    PB6, PB7, PB8, PB9
};

bool buttonState[16];
bool lastButtonState[16];

// =======================
// MIDI CONFIG
// =======================
#define UART_MIDI Serial1
uint8_t ccBase = 20;        // CC mulai dari CC20
uint8_t noteBase = 36;      // Note mulai dari C2

// =======================
// SETUP
// =======================
void setup() {
    // USB MIDI
    MIDI.begin(MIDI_CHANNEL_OMNI);

    // UART MIDI
    UART_MIDI.begin(115200);

    // Pins multiplexer
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    // Button pins
    for (int i = 0; i < 16; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        lastButtonState[i] = HIGH;
    }

    // ADC multiplexer signal
    pinMode(MUX_SIG_PIN, INPUT_ANALOG);
}

// =======================
// READ MULTIPLEXER
// =======================
int readMux(uint8_t channel) {
    digitalWrite(MUX_S0, channel & 1);
    digitalWrite(MUX_S1, channel & 2);
    digitalWrite(MUX_S2, channel & 4);
    digitalWrite(MUX_S3, channel & 8);

    delayMicroseconds(5);
    return analogRead(MUX_SIG_PIN);
}

// =======================
// MAIN LOOP
// =======================
void loop() {

    // ======================
    // ANALOG MULTIPLEXER
    // ======================
    for (uint8_t ch = 0; ch < 16; ch++) {
        int val = readMux(ch);
        int midiVal = map(val, 0, 4095, 0, 127);

        if (abs(midiVal - analogValues[ch]) > 2) {  // Threshold
            analogValues[ch] = midiVal;
            MIDI.sendControlChange(ccBase + ch, midiVal, 1);
        }
    }

    // ======================
    // DIGITAL BUTTONS
    // ======================
    for (uint8_t i = 0; i < 16; i++) {
        bool reading = digitalRead(buttonPins[i]);

        if (reading != lastButtonState[i]) {
            delay(5); // debounce
            reading = digitalRead(buttonPins[i]);
        }

        if (reading != buttonState[i]) {
            buttonState[i] = reading;

            if (reading == LOW) {
                MIDI.sendNoteOn(noteBase + i, 100, 1);
            } else {
                MIDI.sendNoteOff(noteBase + i, 0, 1);
            }
        }

        lastButtonState[i] = reading;
    }

    // ======================
    // USB MIDI → UART MIDI
    // ======================
    if (MIDI.read()) {
        UART_MIDI.write(MIDI.getType());
        UART_MIDI.write(MIDI.getData1());
        UART_MIDI.write(MIDI.getData2());
    }

    // ======================
    // UART MIDI → USB MIDI
    // ======================
    if (UART_MIDI.available() >= 3) {
        byte cmd = UART_MIDI.read();
        byte d1 = UART_MIDI.read();
        byte d2 = UART_MIDI.read();
        MIDI.send(cmd, d1, d2, 1);
    }
}
