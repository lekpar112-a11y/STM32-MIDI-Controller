#pragma once
#include <Arduino.h>
void usb_midi_init();
void usb_midi_send_note_on(uint8_t ch,uint8_t note,uint8_t vel);
void usb_midi_send_note_off(uint8_t ch,uint8_t note);
void usb_midi_send_cc(uint8_t ch,uint8_t cc,uint8_t val);
void usb_midi_send_pitchbend(uint8_t ch,int16_t val);
void usb_midi_send_pc(uint8_t ch,uint8_t pc);
void usb_midi_send_bank_select(uint8_t ch,uint16_t bank);
