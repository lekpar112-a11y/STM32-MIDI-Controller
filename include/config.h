#ifndef CONFIG_H
#define CONFIG_H

// =========================
// KONFIGURASI MATRIX KEYBOARD
// =========================
#define MATRIX_ROWS 8
#define MATRIX_COLS 12

// Pin baris (rows) STM32F103C6T6
static const uint8_t rowPins[MATRIX_ROWS] = {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7
};

// Pin kolom (columns)
static const uint8_t colPins[MATRIX_COLS] = {
    PB0, PB1, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12
};

// =========================
// LED STATUS
// =========================
#define LED_PIN PC13   // LED akan berkedip saat mode setting & saat save selesai

// =========================
// PIN SETTING / SAVE MAPPING (PAKE JUMPER)
// =========================
#define SETTING_PIN PB13   // Hubungkan ke GND untuk masuk mode setting
#define SAVE_PIN PB14      // Hubungkan ke GND untuk menyimpan dan restart

// =========================
// POTENSIOMETER / SLIDER (TIDAK MASUK MATRIX)
// =========================
#define POT_MAIN_VOLUME PA8
#define POT_ACCOMP_VOLUME PA9
#define POT_FX1 PA10
#define POT_FX2 PA11
#define POT_FX3 PA12
#define POT_FX4 PB15
#define POT_FX5 PC14
#define POT_FX6 PC15
#define ROTARY_DT PA15
#define ROTARY_CLK PB2
#define ROTARY_SW  PB9

// =========================
// PITCHBEND (TIDAK MASUK MATRIX)
// =========================
#define PITCHBEND_PIN PA7   // Bisa diubah nanti via setting

// =========================
// EEPROM EMULATION CONFIG
// =========================
#define EEPROM_BASE_ADDRESS 0x0801F800  // Flash page terakhir (aman)
#define EEPROM_MAX_KEYS (MATRIX_ROWS * MATRIX_COLS)
#define EEPROM_SIGNATURE 0xBEEFCAFE     // Untuk validasi apakah data EEPROM sudah ada

#endif
