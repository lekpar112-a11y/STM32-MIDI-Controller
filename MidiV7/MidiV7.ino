/*
 * STM32 MIDI CONTROLLER - FINAL VERSION (V7)
 * 
 * FITUR LENGKAP:
 * 1. USB MIDI Device (Plug & Play)
 * 2. Bluetooth MIDI (via PA9)
 * 3. Matrix Piano 8x8 (Hybrid: Row via 74HC595, Col via Direct GPIO) -> PIN UPLOAD AMAN
 * 4. 32 Tombol Universal (via 74HC165) -> Termasuk tombol fitur
 * 5. 8 Potensio EQ + Balance (via CD4051)
 * 6. Joystick, Velocity FSR, Master Vol
 * 7. LED Activity Indicator (PB11)
 * 8. Upload Code AMAN via ST-Link (PA13/PA14) atau USB-TTL (PA9/PA10)
 */

#include <USBComposite.h>

// ================= PIN DEFINITIONS =================

// --- LED INDIKATOR ---
#define PIN_LED PB11

// --- PORT KOMUNIKASI ---
#define SerialBT Serial1 // PA9 (TX) ke ESP32. PA10 (RX) Tidak dipakai (Safe)

// --- MATRIX BARIS (OUTPUT via 74HC595) ---
#define M_DATA  PB15 // DS (Pin 14 IC)
#define M_LATCH PC13 // ST_CP (Pin 12 IC)
#define M_CLOCK PC14 // SH_CP (Pin 11 IC)

// --- MATRIX KOLOM (INPUT PULLUP - Direct STM32) ---
// Menggunakan PB3 & PB4 yang sudah di-unlock (JTAG Released)
byte colPins[8] = {PB0, PB1, PB3, PB4, PB5, PB6, PB7, PB8};

// --- CD4051 (POTENSIO SELECTOR) ---
#define MUX_S0  PB12
#define MUX_S1  PB13
#define MUX_S2  PB14
#define PIN_MUX_IN PA0

// --- 74HC165 (TOMBOL UNIVERSAL - 4 IC Daisy Chain) ---
#define PIN_165_LOAD PA4
#define PIN_165_CLK  PA5
#define PIN_165_DATA PA6

// --- ANALOG SENSORS ---
#define PIN_JOY_X      PA1
#define PIN_JOY_Y      PA2
#define PIN_VELOCITY   PA3
#define PIN_MASTER_VOL PA7

// --- ROTARY ENCODER ---
#define ENC_A   PA8
#define ENC_B   PB9
#define ENC_SW  PB10

// ================= GLOBAL VARIABLES =================
USBMIDI_ CompositeSerial;

// Settings
int globalTranspose = 0;
int globalOctave = 0;
bool velocityOn = true;     // Default Velocity ON
int tremoloMode = 0;        // 0: Pitchbend, 1: Modulation
volatile int encoderVal = 0;

// States (Penyimpan Status Terakhir)
bool lastKeyState[8][8]; 
uint32_t lastBtn165 = 0; 
int lastFader[8];
int lastJoyX = -1, lastJoyY = -1, lastVol = -1;

// Timers
unsigned long tScan = 0, tAnalog = 0, tLed = 0;

// ================= SETUP =================
void setup() {
  // 1. KONFIGURASI PIN SAKRAL (PENTING!)
  // Matikan JTAG (biar PB3/PB4 bisa dipakai) 
  // TAPI biarkan SWD (PA13/PA14) Hidup untuk ST-Link Upload
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // 2. Init LED
  pinMode(PIN_LED, OUTPUT);
  // Kedip 2x tanda hidup
  digitalWrite(PIN_LED, HIGH); delay(100); digitalWrite(PIN_LED, LOW); delay(100);
  digitalWrite(PIN_LED, HIGH); delay(100); digitalWrite(PIN_LED, LOW);

  // 3. Init 74HC595 (Matrix Rows)
  pinMode(M_DATA, OUTPUT); pinMode(M_LATCH, OUTPUT); pinMode(M_CLOCK, OUTPUT);
  // Reset 595 (High semua = Matrix Off)
  shiftOut(M_DATA, M_CLOCK, LSBFIRST, 0xFF);
  digitalWrite(M_LATCH, HIGH);

  // 4. Init Matrix Cols
  for(int i=0; i<8; i++) pinMode(colPins[i], INPUT_PULLUP);

  // 5. Init Mux CD4051
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT); pinMode(MUX_S2, OUTPUT);
  pinMode(PIN_MUX_IN, INPUT);

  // 6. Init 74HC165 & Analog & Encoder
  pinMode(PIN_165_LOAD, OUTPUT); pinMode(PIN_165_CLK, OUTPUT); pinMode(PIN_165_DATA, INPUT);
  pinMode(PIN_JOY_X, INPUT); pinMode(PIN_JOY_Y, INPUT);
  pinMode(PIN_VELOCITY, INPUT); pinMode(PIN_MASTER_VOL, INPUT);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP); pinMode(ENC_SW, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

  // 7. START USB MIDI & BLUETOOTH
  // Ini baris sakti yang bikin STM32 terbaca "MIDI Device" di PC/HP
  USBComposite.setProductId(0x0030);
  USBComposite.setManufacturerString("Custom DIY");
  USBComposite.setProductString("STM32 Piano Matrix");
  CompositeSerial.registerComponent();
  USBComposite.begin();
  
  SerialBT.begin(31250); // Baudrate MIDI Standar untuk Bluetooth
}

// ================= MAIN LOOP =================
void loop() {
  unsigned long now = millis();

  // 1. SCAN MATRIX PIANO (Prioritas Tinggi)
  scanMatrix595();

  // 2. SCAN TOMBOL UNIVERSAL (Tiap 5ms)
  if (now - tScan > 5) {
    scan165();
    tScan = now;
  }

  // 3. SCAN ANALOG & ENCODER (Tiap 15ms)
  if (now - tAnalog > 15) {
    scanAnalog();
    tAnalog = now;
  }

  // 4. MATIKAN LED (Non-blocking blink)
  if (digitalRead(PIN_LED) == HIGH && now - tLed > 30) {
    digitalWrite(PIN_LED, LOW);
  }
}

// ================= FUNGSI-FUNGSI =================

// --- 1. MATRIX SCANNING (HYBRID) ---
void scanMatrix595() {
  for (int r = 0; r < 8; r++) {
    // Kirim "Walking Zero" ke 74HC595
    byte rowData = ~(1 << r); 
    digitalWrite(M_LATCH, LOW);
    shiftOut(M_DATA, M_CLOCK, LSBFIRST, rowData);
    digitalWrite(M_LATCH, HIGH);

    // Baca Kolom Langsung
    for (int c = 0; c < 8; c++) {
      bool pressed = !digitalRead(colPins[c]); // Active Low
      
      if (pressed != lastKeyState[r][c]) {
        int note = 36 + (r * 8) + c + (globalOctave * 12) + globalTranspose;
        
        if (pressed) {
          int vel = 127;
          if (velocityOn) { // Cek status Velocity ON/OFF
            int fsr = analogRead(PIN_VELOCITY);
            vel = map(fsr, 50, 4000, 10, 127); // Kalibrasi FSR
            vel = constrain(vel, 1, 127);
          }
          sendMidi(0x90, note, vel);
        } else {
          sendMidi(0x80, note, 0);
        }
        lastKeyState[r][c] = pressed;
      }
    }
  }
}

// --- 2. TOMBOL UNIVERSAL SCANNING ---
void scan165() {
  digitalWrite(PIN_165_LOAD, LOW); delayMicroseconds(5);
  digitalWrite(PIN_165_LOAD, HIGH);
  
  uint32_t raw = 0;
  for(int i=0; i<32; i++) {
    raw = (raw << 1) | digitalRead(PIN_165_DATA);
    digitalWrite(PIN_165_CLK, HIGH); digitalWrite(PIN_165_CLK, LOW);
  }
  
  if (raw != lastBtn165) {
    for (int i=0; i<32; i++) {
      int bitNow = (raw >> (31-i)) & 1;
      int bitLast = (lastBtn165 >> (31-i)) & 1;
      
      if (bitNow == 1 && bitLast == 0) { // Tombol Ditekan
        // === MAPPING FITUR ===
        if(i==0) { // Tombol 1: Velocity Toggle
          velocityOn = !velocityOn;
          sendMidi(0xB0, 120, velocityOn ? 127 : 0); // Indikator ke DAW
        }
        else if(i==1) { // Tombol 2: Tap Tempo
          sendMidi(0xB0, 64, 127);
        }
        else if(i==2) { // Tombol 3: Mode Joystick (Pitch/Mod)
          tremoloMode = !tremoloMode;
          sendMidi(0xB0, 121, tremoloMode ? 127 : 0);
        }
        else { // Tombol Sisa (Note/Pad)
          sendMidi(0x90, 50+i, 100);
        }
      }
      // Note Off untuk tombol sisa (agar bisa dilepas)
      else if (bitNow == 0 && bitLast == 1 && i > 2) {
          sendMidi(0x80, 50+i, 0);
      }
    }
    lastBtn165 = raw;
  }
}

// --- 3. ANALOG SCANNING ---
void scanAnalog() {
  // Joystick X (Pitch / Mod)
  int x = analogRead(PIN_JOY_X);
  if(abs(x - lastJoyX) > 15) {
    if (tremoloMode == 0) { // Mode Pitchbend
      int pb = map(x, 0, 4095, 0, 16383);
      CompositeSerial.sendPitchBend(0, pb);
      SerialBT.write(0xE0); SerialBT.write(pb & 0x7F); SerialBT.write(pb >> 7);
      // Nyalakan LED manual karena PB tidak panggil sendMidi
      digitalWrite(PIN_LED, HIGH); tLed = millis(); 
    } else { // Mode Tremolo
      sendMidi(0xB0, 1, map(x, 0, 4095, 0, 127));
    }
    lastJoyX = x;
  }
  
  // Joystick Y (Expression/Breath)
  int y = analogRead(PIN_JOY_Y);
  if(abs(y - lastJoyY) > 15) {
    sendMidi(0xB0, 2, map(y, 0, 4095, 0, 127));
    lastJoyY = y;
  }

  // CD4051 Potensio (EQ + Balance)
  for(int i=0; i<8; i++) {
    digitalWrite(MUX_S0, i&1); digitalWrite(MUX_S1, i&2); digitalWrite(MUX_S2, i&4);
    delayMicroseconds(30); // Stabilisasi analog
    int val = analogRead(PIN_MUX_IN);
    if(abs(val - lastFader[i]) > 15) {
      int cc = (i<7) ? 16+i : 8; // Ch0-6: CC16-22, Ch7: Balance(CC8)
      sendMidi(0xB0, cc, map(val,0,4095,0,127));
      lastFader[i] = val;
    }
  }
  
  // Master Volume
  int mv = analogRead(PIN_MASTER_VOL);
  if(abs(mv - lastVol) > 15) {
    sendMidi(0xB0, 7, map(mv,0,4095,0,127));
    lastVol = mv;
  }

  // Encoder Logic (Menu / Transpose)
  if(encoderVal != 0) { 
    globalTranspose += encoderVal; // Default ubah Transpose
    encoderVal = 0; 
    // Kedip LED tanda setting berubah
    digitalWrite(PIN_LED, HIGH); tLed = millis();
  }
}

// --- 4. ENCODER INTERRUPT ---
void isrEncoder() {
  static uint8_t old_AB = 0;
  old_AB <<= 2;                   
  old_AB |= ( (digitalRead(ENC_A)) | (digitalRead(ENC_B)<<1) );
  old_AB &= 0x0f; 
  if ( old_AB == 0x03 || old_AB == 0x0c ) return;
  if ( old_AB == 0x01 || old_AB == 0x0e || old_AB == 0x08 || old_AB == 0x07 ) encoderVal++;
  else encoderVal--;
}

// --- 5. HELPER: KIRIM MIDI & NYALAKAN LED ---
void sendMidi(byte cmd, byte d1, byte d2) {
  // Kirim USB
  if ((cmd & 0xF0) == 0x90) CompositeSerial.sendNoteOn(0, d1, d2);
  else if ((cmd & 0xF0) == 0x80) CompositeSerial.sendNoteOff(0, d1, d2);
  else CompositeSerial.sendControlChange(0, d1, d2);
  
  // Kirim Bluetooth (Serial)
  SerialBT.write(cmd); SerialBT.write(d1); SerialBT.write(d2);

  // NYALAKAN LED (Feedback Visual)
  digitalWrite(PIN_LED, HIGH);
  tLed = millis(); // Reset timer LED
}
