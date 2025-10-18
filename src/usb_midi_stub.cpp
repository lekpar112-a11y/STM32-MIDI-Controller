// Fallback stub: if native USB-MIDI not integrated, send MIDI raw bytes over Serial (CDC)
#include "usb_midi_if.h"

void usb_midi_init(){ /* nothing, Serial already init in midi_init() */ }

void usb_midi_send_note_on(uint8_t ch,uint8_t note,uint8_t vel){
  uint8_t msg[3] = {(uint8_t)(0x90 | ((ch-1)&0x0F)), note & 0x7F, vel & 0x7F};
  Serial.write(msg,3);
}
void usb_midi_send_note_off(uint8_t ch,uint8_t note){
  uint8_t msg[3] = {(uint8_t)(0x80 | ((ch-1)&0x0F)), note & 0x7F, 0};
  Serial.write(msg,3);
}
void usb_midi_send_cc(uint8_t ch,uint8_t cc,uint8_t val){
  uint8_t msg[3] = {(uint8_t)(0xB0 | ((ch-1)&0x0F)), cc & 0x7F, val & 0x7F};
  Serial.write(msg,3);
}
void usb_midi_send_pitchbend(uint8_t ch,int16_t val){
  int32_t v = (int32_t)val + 8192;
  uint8_t l = v & 0x7F;
  uint8_t h = (v>>7) & 0x7F;
  uint8_t msg[3] = {(uint8_t)(0xE0 | ((ch-1)&0x0F)), l, h};
  Serial.write(msg,3);
}
// NEW ADDITION: Program Change & Bank Select
void usb_midi_send_pc(uint8_t ch,uint8_t pc){
  uint8_t msg[2] = {(uint8_t)(0xC0 | ((ch-1)&0x0F)), pc & 0x7F};
  Serial.write(msg,2);
}
void usb_midi_send_bank_select(uint8_t ch,uint16_t bank){
  // Bank Select MSB (CC 0)
  uint8_t msg0[3] = {(uint8_t)(0xB0 | ((ch-1)&0x0F)), 0, (uint8_t)((bank>>7) & 0x7F)};
  Serial.write(msg0,3);
  // Bank Select LSB (CC 32)
  uint8_t msg32[3] = {(uint8_t)(0xB0 | ((ch-1)&0x0F)), 32, (uint8_t)(bank & 0x7F)};
  Serial.write(msg32,3);
}
