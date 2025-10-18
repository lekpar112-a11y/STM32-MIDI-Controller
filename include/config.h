#pragma once
#include <Arduino.h>

// Matrix 8x8
#define MATRIX_ROWS 8
#define MATRIX_COLS 8
static const uint8_t ROW_PINS[MATRIX_ROWS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
static const uint8_t COL_PINS[MATRIX_COLS] = {PB0, PB1, PB2, PB10, PB11, PB12, PB13, PB14};

// Universal buttons (22)
static const uint8_t BTN_PINS[22] = {
  PB15, PC13, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PA8,
  PA9, PA10, PB2, PB1, PB0, PC14, PC15, PB14, PB13, PB12,
  PB11, PB10
};

// Encoder
#define ENCODER_A PB3
#define ENCODER_B PB4
#define ENCODER_SW PB5

// Pressure
#define PRESSURE_ENABLE_PIN PC14

// CD4051 wiring
#define CD4051_S0 PB6
#define CD4051_S1 PB7
#define CD4051_S2 PB8
#define CD4051_ADC_PIN A0
#define CD4051_CS0 PC0
#define CD4051_CS1 PC1
#define CD4051_CS2 PC2
#define CD4051_NUM_CHIPS 3

// analog mapping
#define PITCHBEND_X_CHIP 0
#define PITCHBEND_X_CH 0
#define PITCHBEND_Y_CHIP 0
#define PITCHBEND_Y_CH 1 // CH 1 for Pitchbend Y (Expression/Dynamic CC)
#define MASTER_VOL_CHIP 0
#define MASTER_VOL_CH 2
#define MASTER_BAL_CHIP 0
#define MASTER_BAL_CH 3
#define EQ_BASE_CHIP 0
#define EQ_BASE_CH 4
#define MOD_WHEEL_CHIP 0
#define MOD_WHEEL_CH 5 // Added for Mod Wheel (CC 1)
#define PRESSURE_BASE_IDX 12

// MIDI & behaviour
#define MIDI_CHANNEL 1
#define MIDI_BAUD 31250
#define DEBOUNCE_MS 8
#define VELOCITY_DEFAULT 100
#define METRONOME_NOTE_LENGTH_MS 140
#define DEFAULT_BPM 90
#define MIDI_START_NOTE 36

// Encoder Modes
#define ENCODER_MODE_OCTAVE 0
#define ENCODER_MODE_TEMPO 1
#define ENCODER_MODE_TRANSPOSE 2
#define ENCODER_MODE_PBRANGE 3 // NEW: Pitch Bend Range
#define ENCODER_MODE_COUNT 4 // Updated to 4

// Y-AXIS CC TARGETS
#define Y_CC_EXPRESSION 11 // Default
#define Y_CC_MODULATION 1
#define Y_CC_FILTER_CUTOFF 74
#define Y_CC_TARGET_COUNT 3
