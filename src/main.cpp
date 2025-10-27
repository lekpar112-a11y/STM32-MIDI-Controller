#include <Arduino.h>
#include <MIDI.h>
#include <BLEMidi.h>

// Buat objek MIDI untuk setiap output
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, hwMidi);
MIDI_CREATE_INSTANCE(Stream, BLE, bleMidi);            

#define BLE_MIDI_SERIAL Serial2

// ... (Fungsi-fungsi pembantu Anda seperti scanMatrix, readAnalogs, dll. perlu diadaptasi di sini)

// Contoh fungsi yang diadaptasi
void sendNoteOnToAll(byte channel, byte pitch, byte velocity) {
    hwMidi.sendNoteOn(pitch, velocity, channel);
    bleMidi.sendNoteOn(pitch, velocity, channel);
    // Kode untuk USB MIDI perlu ditambahkan di sini secara manual
}

void setup() {
    hwMidi.begin(MIDI_CHANNEL_OMNI);
    BLE_MIDI_SERIAL.begin(9600);
    bleMidi.begin(MIDI_CHANNEL_OMNI);
    BLE.begin("STM32 BLE MIDI", BLE_MIDI_SERIAL);
}

void loop() {
    BLE.poll();
    // Panggil fungsi-fungsi Anda di sini
}
