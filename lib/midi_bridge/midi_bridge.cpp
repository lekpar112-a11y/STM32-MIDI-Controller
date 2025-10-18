#include "midi_bridge.h"
#include "include/config.h"

HardwareSerial &MIDI_UART = Serial1;

// midi_init: start both UART (DIN) and Serial (USB CDC)
void midi_init(){
  MIDI_UART.begin(MIDI_BAUD);
  Serial.begin(115200);
}

// send to UART and to Serial (CDC). Native USB-MIDI will be handled separately if available.
static void send_uart(const uint8_t *b, size_t n){ MIDI_UART.write(b, n); }
static void send_usb(const uint8_t *b, size_t n){ Serial.write(b, n); }

void midi_note_on(uint8_t ch, uint8_t note, uint8_t vel){
  uint8_t msg[3] = {(uint8_t)(0x90 | ((ch-1)&0x0F)), note & 0x7F, vel & 0x7F};
  send_uart(msg,3); send_usb(msg,3);
}
void midi_note_off(uint8_t ch, uint8_t note){
  uint8_t msg[3] = {(uint8_t)(0x80 | ((ch-1)&0x0F)), note & 0x7F, 0};
  send_uart(msg,3); send_usb(msg,3);
}
void midi_control_change(uint8_t ch, uint8_t cc, uint8_t val){
  uint8_t msg[3] = {(uint8_t)(0xB0 | ((ch-1)&0x0F)), cc & 0x7F, val & 0x7F};
  send_uart(msg,3); send_usb(msg,3);
}
void midi_pitchbend(uint8_t ch, int16_t val){
  int32_t v = (int32_t)val + 8192;
  uint8_t l = v & 0x7F;
  uint8_t h = (v>>7) & 0x7F;
  uint8_t msg[3] = {(uint8_t)(0xE0 | ((ch-1)&0x0F)), l, h};
  send_uart(msg,3); send_usb(msg,3);
}
// NEW ADDITION: Program Change & Bank Select
void midi_program_change(uint8_t ch, uint8_t pc){
  uint8_t msg[2] = {(uint8_t)(0xC0 | ((ch-1)&0x0F)), pc & 0x7F};
  send_uart(msg,2); send_usb(msg,2);
}
void midi_bank_select(uint8_t ch, uint16_t bank){
  // Bank Select MSB (CC 0)
  uint8_t msg0[3] = {(uint8_t)(0xB0 | ((ch-1)&0x0F)), 0, (uint8_t)((bank>>7) & 0x7F)};
  send_uart(msg0,3); send_usb(msg0,3);
  // Bank Select LSB (CC 32)
  uint8_t msg32[3] = {(uint8_t)(0xB0 | ((ch-1)&0x0F)), 32, (uint8_t)(bank & 0x7F)};
  send_uart(msg32,3); send_usb(msg32,3);
}
