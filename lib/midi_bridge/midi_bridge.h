#pragma once
#include <Arduino.h>
#include <stdint.h>
void midi_init();
void midi_note_on(uint8_t ch, uint8_t note, uint8_t vel);
void midi_note_off(uint8_t ch, uint8_t note);
void midi_control_change(uint8_t ch, uint8_t cc, uint8_t val);
void midi_pitchbend(uint8_t ch, int16_t val);
void midi_program_change(uint8_t ch, uint8_t pc);
void midi_bank_select(uint8_t ch, uint16_t bank);
