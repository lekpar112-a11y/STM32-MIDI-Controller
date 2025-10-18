#pragma once
#include <Arduino.h>
void cd4051_init();
int cd4051_read(uint8_t chip, uint8_t channel);
int cd4051_read_flat(uint8_t idx);
