#include <Arduino.h>
#include <vector>
#include "include/config.h"
#include "cd4051.h"
#include "midi_bridge.h"
#include "usb_midi_if.h"

// runtime state
bool keyState[MATRIX_ROWS][MATRIX_COLS];
unsigned long keyDebounce[MATRIX_ROWS][MATRIX_COLS];
bool btnState[22];
unsigned long btnDebounce[22];
int bpm = DEFAULT_BPM;
unsigned long tickIntervalMs = 60000UL / DEFAULT_BPM;
unsigned long lastTick = 0;
uint8_t tempo_div = 4; // Default 4/4
uint8_t met_step = 0;
bool tempo_chord_enabled=false, chord_scan_enabled=false;
bool single_pressure_enabled=false;
int octave_offset=0, transpose=0;

// NEW ADDITION: Arranger State
uint8_t current_cc_bank = 0; // 0 sampai 3 (CC 20-27, 28-35, 36-43, 44-51)
uint8_t current_program = 0; // 0 sampai 127
uint16_t current_bank_msb_lsb = 0; // 0x0000 sampai 0x3FFF
unsigned long last_tap_time = 0;
int current_pb_range = 2; // Default 2 semitones
uint8_t current_y_cc_target_idx = 0;
const uint8_t y_cc_targets[Y_CC_TARGET_COUNT] = {Y_CC_EXPRESSION, Y_CC_MODULATION, Y_CC_FILTER_CUTOFF};
uint8_t current_song_select = 0; // 0-127 for Song Select (Style/Performance)
// END Arranger State

// Encoder mode state
uint8_t encoder_mode = ENCODER_MODE_OCTAVE; // Default mode

inline int rc_to_index(uint8_t r, uint8_t c){ return r*MATRIX_COLS + c; }

void setup_pins(){
  for (uint8_t r=0;r<MATRIX_ROWS;r++){
    pinMode(ROW_PINS[r], OUTPUT); digitalWrite(ROW_PINS[r], HIGH);
    for (uint8_t c=0;c<MATRIX_COLS;c++){ keyState[r][c]=false; keyDebounce[r][c]=0; }
  }
  for (uint8_t c=0;c<MATRIX_COLS;c++) pinMode(COL_PINS[c], INPUT_PULLUP);
  for (uint8_t i=0;i<22;i++){ pinMode(BTN_PINS[i], INPUT_PULLUP); btnState[i]=false; btnDebounce[i]=0; }
  pinMode(ENCODER_A, INPUT_PULLUP); pinMode(ENCODER_B, INPUT_PULLUP); pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(PRESSURE_ENABLE_PIN, INPUT_PULLUP);
  cd4051_init();
}

void update_tick(){ tickIntervalMs = 60000UL / max(1, bpm); }

// NEW ADDITION: MIDI Sender Helpers
void send_program_change(uint8_t pc, uint16_t bank){
  // Kirim Bank Select CC 0 (MSB) dan CC 32 (LSB)
  midi_bank_select(MIDI_CHANNEL, bank);
  usb_midi_send_bank_select(MIDI_CHANNEL, bank);
  // Kirim Program Change
  midi_program_change(MIDI_CHANNEL, pc);
  usb_midi_send_pc(MIDI_CHANNEL, pc);
  
  Serial.print("PC/Bank: "); Serial.print(pc); Serial.print(" / "); Serial.println(bank, HEX);
}

// NEW ADDITION: RPN Pitch Bend Range Sender
void send_rpn_pitchbend_range(uint8_t ch, uint8_t semitones){
    uint8_t range = constrain(semitones, 0, 24); // Batasi range 0-24 semiton
    // RPN LSB: 0 (CC 100)
    midi_control_change(ch, 100, 0); usb_midi_send_cc(ch, 100, 0); 
    // RPN MSB: 0 (CC 101)
    midi_control_change(ch, 101, 0); usb_midi_send_cc(ch, 101, 0);
    // Data Entry (Nilai RPN dalam semiton): CC 6
    midi_control_change(ch, 6, range); usb_midi_send_cc(ch, 6, range);
    
    // RPN Null (Opsional, untuk mencegah perubahan RPN yang tidak disengaja)
    midi_control_change(ch, 100, 127); usb_midi_send_cc(ch, 100, 127);
    midi_control_change(ch, 101, 127); usb_midi_send_cc(ch, 101, 127);
    
    Serial.print("PB Range: "); Serial.println(range);
}

// NEW ADDITION: MIDI Song Select (for Performance/Style Index)
void send_song_select(uint8_t ss){
    uint8_t msg[2] = {0xF3, ss & 0x7F}; // F3 = Song Select
    // MIDI DIN:
    HardwareSerial &MIDI_UART = Serial1;
    MIDI_UART.write(msg, 2);
    // USB CDC (Stub):
    Serial.write(msg, 2);
    // TIDAK menggunakan usb_midi_send_pc/cc karena ini adalah System Common Message
    Serial.print("Song Select: "); Serial.println(ss);
}
// END MIDI Sender Helpers

void handle_metronome(){
  // MODIFIKASI: Menggunakan tempo_chord_enabled sebagai SWIT ON/OFF
  if (!tempo_chord_enabled) return; 
  unsigned long now = millis();
  if (now - lastTick >= (tickIntervalMs / max(1, tempo_div))){
    lastTick = now; met_step = (met_step + 1) % tempo_div;
    if (met_step == 0 && chord_scan_enabled){
      std::vector<uint8_t> pressed;
      for (uint8_t r=0;r<MATRIX_ROWS;r++) for (uint8_t c=0;c<MATRIX_COLS;c++) if (keyState[r][c]) pressed.push_back(MIDI_START_NOTE + rc_to_index(r,c) + octave_offset*12 + transpose);
      bool pc[12]={0}; for (auto n: pressed) pc[n%12]=true;
      uint8_t root=0,type=0;
      for (uint8_t cand=0;cand<12;cand++){
        if (pc[cand] && pc[(cand+4)%12] && pc[(cand+7)%12]){ root=cand; type=1; break; }
        if (pc[cand] && pc[(cand+3)%12] && pc[(cand+7)%12]){ root=cand; type=2; break; }
      }
      if (type){
        int base = MIDI_START_NOTE + 12;
        int rootNote = (base/12)*12 + root;
        uint8_t n1=rootNote, n2=rootNote + (type==1?4:3), n3=rootNote+7;
        uint8_t vel = VELOCITY_DEFAULT;
        if (single_pressure_enabled) vel = map(cd4051_read_flat(0),0,1023,1,127);
        midi_note_on(MIDI_CHANNEL, n1, vel); usb_midi_send_note_on(MIDI_CHANNEL,n1,vel);
        midi_note_on(MIDI_CHANNEL, n2, vel); usb_midi_send_note_on(MIDI_CHANNEL,n2,vel);
        midi_note_on(MIDI_CHANNEL, n3, vel); usb_midi_send_note_on(MIDI_CHANNEL,n3,vel);
        delay(METRONOME_NOTE_LENGTH_MS);
        midi_note_off(MIDI_CHANNEL, n1); usb_midi_send_note_off(MIDI_CHANNEL,n1);
        midi_note_off(MIDI_CHANNEL, n2); usb_midi_send_note_off(MIDI_CHANNEL,n2);
        midi_note_off(MIDI_CHANNEL, n3); usb_midi_send_note_off(MIDI_CHANNEL,n3);
      }
    }
  }
}

void scan_matrix_and_dispatch(){
  int analogs[CD4051_NUM_CHIPS*8];
  for (uint8_t chip=0; chip<CD4051_NUM_CHIPS; chip++) for (uint8_t ch=0; ch<8; ch++) analogs[chip*8+ch] = cd4051_read(chip,ch);
  single_pressure_enabled = (digitalRead(PRESSURE_ENABLE_PIN)==LOW);

  // PITCHBEND X
  int pbx = analogs[PITCHBEND_X_CHIP*8 + PITCHBEND_X_CH];
  int16_t pbx_m = (int16_t)map(pbx, 0, 1023, -8192, 8191);
  midi_pitchbend(MIDI_CHANNEL, pbx_m); usb_midi_send_pitchbend(MIDI_CHANNEL,pbx_m);

  // PITCHBEND Y (Dynamically mapped to CC)
  uint8_t y_cc = y_cc_targets[current_y_cc_target_idx]; // Ambil target CC
  int pby = analogs[PITCHBEND_Y_CHIP*8 + PITCHBEND_Y_CH];
  uint8_t pby_m = map(pby, 0, 1023, 0, 127);
  midi_control_change(MIDI_CHANNEL, y_cc, pby_m); usb_midi_send_cc(MIDI_CHANNEL,y_cc,pby_m);
  midi_control_change(MIDI_CHANNEL, 18, current_y_cc_target_idx); usb_midi_send_cc(MIDI_CHANNEL,18,current_y_cc_target_idx); // CC 18 untuk indikasi Y-Axis mode
  
  // NEW ADDITION: MODULATION WHEEL (CC 1)
  int mod = analogs[MOD_WHEEL_CHIP*8 + MOD_WHEEL_CH];
  uint8_t mod_m = map(mod, 0, 1023, 0, 127);
  midi_control_change(MIDI_CHANNEL, 1, mod_m); usb_midi_send_cc(MIDI_CHANNEL,1,mod_m); // CC 1 for Modulation
  // END NEW ADDITION

  uint8_t master_cc = map(analogs[MASTER_VOL_CHIP*8 + MASTER_VOL_CH],0,1023,0,127);
  uint8_t balance_cc = map(analogs[MASTER_BAL_CHIP*8 + MASTER_BAL_CH],0,1023,0,127);
  midi_control_change(MIDI_CHANNEL, 7, master_cc); usb_midi_send_cc(MIDI_CHANNEL,7,master_cc);
  midi_control_change(MIDI_CHANNEL, 10, balance_cc); usb_midi_send_cc(MIDI_CHANNEL,10,balance_cc);
  
  // CC Fader Bank Switching
  uint8_t base_cc = 20 + (current_cc_bank * 8); // Bank 0: CC 20-27, Bank 1: CC 28-35, dst.
  for (uint8_t i=0;i<8;i++){
    uint8_t v = map(analogs[EQ_BASE_CHIP*8 + (EQ_BASE_CH + i)],0,1023,0,127);
    midi_control_change(MIDI_CHANNEL, base_cc + i, v); usb_midi_send_cc(MIDI_CHANNEL,base_cc+i,v);
  }
  midi_control_change(MIDI_CHANNEL, 19, current_cc_bank); usb_midi_send_cc(MIDI_CHANNEL, 19, current_cc_bank); // CC 19 untuk CC Bank Indication

  for (uint8_t r=0;r<MATRIX_ROWS;r++){
    for (uint8_t rr=0; rr<MATRIX_ROWS; rr++) digitalWrite(ROW_PINS[rr], rr==r?LOW:HIGH);
    delayMicroseconds(30);
    for (uint8_t c=0;c<MATRIX_COLS;c++){
      bool pressed = (digitalRead(COL_PINS[c])==LOW);
      unsigned long now = millis();
      if (pressed != keyState[r][c]){
        if (now - keyDebounce[r][c] > DEBOUNCE_MS){
          keyState[r][c] = pressed; keyDebounce[r][c] = now;
          uint8_t note = MIDI_START_NOTE + rc_to_index(r,c) + octave_offset*12 + transpose;
          if (pressed){
            uint8_t vel = VELOCITY_DEFAULT;
            int pidx = PRESSURE_BASE_IDX + rc_to_index(r,c);
            if (pidx < CD4051_NUM_CHIPS*8) vel = map(cd4051_read_flat(pidx),0,1023,1,127);
            else if (single_pressure_enabled) vel = map(analogs[12],0,1023,1,127);
            midi_note_on(MIDI_CHANNEL, note, vel); usb_midi_send_note_on(MIDI_CHANNEL,note,vel);
          } else {
            midi_note_off(MIDI_CHANNEL, note); usb_midi_send_note_off(MIDI_CHANNEL,note);
          }
        }
      } else { keyDebounce[r][c]=now; }
    }
  }
}

unsigned long encoderSwDebounce = 0; // Tambahkan debounce untuk tombol encoder
void read_buttons(){
  for (uint8_t i=0;i<22;i++){
    bool st = (digitalRead(BTN_PINS[i])==LOW);
    unsigned long now = millis();
    if (st != btnState[i]){
      if (now - btnDebounce[i] > DEBOUNCE_MS){
        btnState[i]=st; btnDebounce[i]=now;
        if (st){
          switch(i){
            case 0: chord_scan_enabled = !chord_scan_enabled; break;
            case 1: // SWIT ON/OFF untuk Tempo di Cord (tempo_chord_enabled)
                tempo_chord_enabled = !tempo_chord_enabled; 
                Serial.print("Tempo Chord: "); Serial.println(tempo_chord_enabled?"ON":"OFF");
                break;
            case 2: tempo_div = 2; break; // Ketukan 2
            case 3: tempo_div = 3; break; // Ketukan 3
            case 4: tempo_div = 4; break; // Ketukan 4
            case 5: transpose++; break;
            case 6: transpose--; break;
            case 7: octave_offset++; break;
            case 8: octave_offset--; break;
            // NEW ADDITION: Arranger Control Buttons
            case 9: // PC UP
              current_program = (current_program + 1) % 128;
              send_program_change(current_program, current_bank_msb_lsb);
              break;
            case 10: // PC DOWN
              current_program = (current_program - 1 + 128) % 128;
              send_program_change(current_program, current_bank_msb_lsb);
              break;
            case 11: // CC Bank Cycle (0-3)
              current_cc_bank = (current_cc_bank + 1) % 4;
              Serial.print("CC Bank: "); Serial.println(current_cc_bank);
              break;
            case 12: // TAP TEMPO
              {
                unsigned long now = millis();
                if (last_tap_time != 0 && (now - last_tap_time) < 3000){
                  unsigned long interval = now - last_tap_time;
                  int new_bpm = 60000UL / interval;
                  bpm = constrain(new_bpm, 30, 240);
                  update_tick();
                  midi_control_change(MIDI_CHANNEL, 23, map(bpm,30,240,0,127));
                  usb_midi_send_cc(MIDI_CHANNEL, 23, map(bpm,30,240,0,127));
                  Serial.print("TAP BPM: "); Serial.println(bpm);
                }
                last_tap_time = now;
              }
              break;
            case 13: // Bank Select LSB Cycle
                current_bank_msb_lsb = (current_bank_msb_lsb & 0xFF80) | ((current_bank_msb_lsb + 1) & 0x7F);
                send_program_change(current_program, current_bank_msb_lsb);
                break;
            case 14: // Bank Select MSB Cycle
                current_bank_msb_lsb = (current_bank_msb_lsb & 0x007F) | (((current_bank_msb_lsb + 128) & 0x7F00));
                send_program_change(current_program, current_bank_msb_lsb);
                break;
            case 15: // Y-Axis CC Target Cycle
                current_y_cc_target_idx = (current_y_cc_target_idx + 1) % Y_CC_TARGET_COUNT;
                Serial.print("Y-Axis CC: "); 
                Serial.println(y_cc_targets[current_y_cc_target_idx]);
                break;
            case 16: // MIDI STOP (Analog to Style STOP)
                MIDI_UART.write(0xFC); Serial.write(0xFC); // FC = Stop
                Serial.println("MIDI STOP");
                break;
            case 17: // MIDI START/CONTINUE (Analog to Style START)
                MIDI_UART.write(0xFA); Serial.write(0xFA); // FA = Start
                Serial.println("MIDI START");
                break;
            case 18: // Performance/Style Select UP (Menggunakan Song Select)
                current_song_select = (current_song_select + 1) % 128;
                send_song_select(current_song_select);
                break;
            case 19: // Performance/Style Select DOWN
                current_song_select = (current_song_select - 1 + 128) % 128;
                send_song_select(current_song_select);
                break;
            case 20: // TEMPO RESET ke DEFAULT_BPM
                bpm = DEFAULT_BPM;
                update_tick();
                midi_control_change(MIDI_CHANNEL, 23, map(bpm,30,240,0,127));
                usb_midi_send_cc(MIDI_CHANNEL, 23, map(bpm,30,240,0,127));
                Serial.print("BPM Reset: "); Serial.println(bpm);
                break;
            // END NEW ADDITION
            default: midi_control_change(MIDI_CHANNEL, 40 + (i%64), 127); usb_midi_send_cc(MIDI_CHANNEL,40+(i%64),127); break;
          }
        }
      }
    }
  }

  // Handle ENCODER_SW
  bool encoderSwPressed = (digitalRead(ENCODER_SW) == LOW);
  unsigned long now = millis();
  if (encoderSwPressed && (now - encoderSwDebounce > DEBOUNCE_MS)) {
    // Only cycle mode on press, not hold
    if (!btnState[ENCODER_SW]) {
        encoder_mode = (encoder_mode + 1) % ENCODER_MODE_COUNT;
        Serial.print("Encoder Mode: ");
        if (encoder_mode == ENCODER_MODE_OCTAVE) Serial.println("Octave");
        else if (encoder_mode == ENCODER_MODE_TEMPO) Serial.println("Tempo");
        else if (encoder_mode == ENCODER_MODE_TRANSPOSE) Serial.println("Transpose");
        else if (encoder_mode == ENCODER_MODE_PBRANGE) Serial.println("PB Range");
        midi_control_change(MIDI_CHANNEL, 22, encoder_mode); // CC 22 to indicate encoder mode
        usb_midi_send_cc(MIDI_CHANNEL, 22, encoder_mode);
    }
    encoderSwDebounce = now;
  }
  btnState[ENCODER_SW] = encoderSwPressed;
}

int lastA=HIGH, enc_acc=0;
void encoder_poll(){
  int a = digitalRead(ENCODER_A), b = digitalRead(ENCODER_B);
  if (a != lastA){ if (b != a) enc_acc++; else enc_acc--; lastA = a; }
  if (enc_acc != 0){
    int d = enc_acc; enc_acc = 0;
    switch(encoder_mode){
      case ENCODER_MODE_OCTAVE:
        octave_offset = constrain(octave_offset + (d>0?1:-1)*abs(d), -4, 4);
        midi_control_change(MIDI_CHANNEL,21,octave_offset+8); 
        usb_midi_send_cc(MIDI_CHANNEL,21,octave_offset+8);
        Serial.print("Octave: "); Serial.println(octave_offset);
        break;
      case ENCODER_MODE_TEMPO:
        bpm = constrain(bpm + (d>0?1:-1)*abs(d), 30, 240);
        update_tick();
        midi_control_change(MIDI_CHANNEL,23,map(bpm,30,240,0,127)); 
        usb_midi_send_cc(MIDI_CHANNEL,23,map(bpm,30,240,0,127));
        Serial.print("BPM: "); Serial.println(bpm);
        break;
      case ENCODER_MODE_TRANSPOSE:
        transpose = constrain(transpose + (d>0?1:-1)*abs(d), -12, 12);
        midi_control_change(MIDI_CHANNEL,24,transpose+12);
        usb_midi_send_cc(MIDI_CHANNEL,24,transpose+12);
        Serial.print("Transpose: "); Serial.println(transpose);
        break;
      case ENCODER_MODE_PBRANGE: // NEW: Pitch Bend Range
        current_pb_range = constrain(current_pb_range + (d>0?1:-1)*abs(d), 0, 24);
        send_rpn_pitchbend_range(MIDI_CHANNEL, current_pb_range);
        break;
    }
  }
}

void setup(){
  setup_pins();
  midi_init();
  usb_midi_init(); 
  update_tick();
  lastTick = millis();
  pinMode(ENCODER_SW, INPUT_PULLUP);
  // Initial Arranger State Sync
  send_program_change(current_program, current_bank_msb_lsb);
  send_rpn_pitchbend_range(MIDI_CHANNEL, current_pb_range);
  send_song_select(current_song_select); // Kirim Song Select awal
  
  Serial.println("STM32 MIDI USB firmware ready");
  Serial.print("Encoder Mode: ");
  if (encoder_mode == ENCODER_MODE_OCTAVE) Serial.println("Octave");
  else if (encoder_mode == ENCODER_MODE_TEMPO) Serial.println("Tempo");
  else if (encoder_mode == ENCODER_MODE_TRANSPOSE) Serial.println("Transpose");
  else if (encoder_mode == ENCODER_MODE_PBRANGE) Serial.println("PB Range");
}

void loop(){
  scan_matrix_and_dispatch();
  read_buttons(); 
  encoder_poll();
  handle_metronome();
  delay(2);
}
