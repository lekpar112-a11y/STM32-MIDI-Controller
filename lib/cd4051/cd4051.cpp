#include "cd4051.h"
#include "include/config.h"
static uint8_t s0 = CD4051_S0, s1 = CD4051_S1, s2 = CD4051_S2;
static uint8_t cs0 = CD4051_CS0, cs1 = CD4051_CS1, cs2 = CD4051_CS2;
static uint8_t adc = CD4051_ADC_PIN;

void cd4051_init(){
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT); pinMode(s2, OUTPUT);
  pinMode(cs0, OUTPUT); pinMode(cs1, OUTPUT); pinMode(cs2, OUTPUT);
  digitalWrite(cs0, HIGH); digitalWrite(cs1, HIGH); digitalWrite(cs2, HIGH);
  pinMode(adc, INPUT);
}

static void set_channel(uint8_t ch){
  digitalWrite(s0, (ch>>0)&1);
  digitalWrite(s1, (ch>>1)&1);
  digitalWrite(s2, (ch>>2)&1);
  delayMicroseconds(5);
}

static void select_chip(uint8_t chip){
  digitalWrite(cs0, chip==0?LOW:HIGH);
  digitalWrite(cs1, chip==1?LOW:HIGH);
  digitalWrite(cs2, chip==2?LOW:HIGH);
  delayMicroseconds(5);
}

int cd4051_read(uint8_t chip, uint8_t channel){
  if (chip >= CD4051_NUM_CHIPS || channel >= 8) return 0;
  select_chip(chip);
  set_channel(channel);
  delayMicroseconds(40);
  int v = analogRead(adc);
  digitalWrite(cs0, HIGH); digitalWrite(cs1, HIGH); digitalWrite(cs2, HIGH);
  return v;
}
int cd4051_read_flat(uint8_t idx){
  uint8_t chip = idx / 8;
  uint8_t ch = idx % 8;
  return cd4051_read(chip, ch);
}
