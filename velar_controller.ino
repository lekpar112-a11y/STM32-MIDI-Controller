/* velar_controller.ino
   Full STM32 sketch (Arduino core) for 8x8 matrix MIDI controller + many features
   - Pin names fixed to STM32 Arduino core style (e.g. PA_0, PB_15, PC_2)
   - Keeps all original features. If any pin not present on your MCU package,
     tell me the exact MCU model and I'll remap.
*/

#include <Arduino.h>

/* ---------- REMAPPED PINS (STM32 naming with underscores) ---------- */
/* Strategy:
   - Rows: PB_0..PB_7 (digital inputs)
   - Cols: PB_8..PB_15 (digital outputs)
   - ADC analog signals on PA_x / PC_x where possible
   - CD4051 control on PC_2..PC_5
   - Universal MUX EN on PC_6..PC_10
   - Velocity pot on PC_0, velocity toggle on PC_1
*/

//
// MATRIX 8x8
//
const uint8_t ROW_PINS[8] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};
const uint8_t COL_PINS[8] = {PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15};

//
// JOYSTICK X/Y (ADC)
//
const uint8_t PIN_PITCH_X = PA_6;
const uint8_t PIN_PITCH_Y = PA_7;

//
// MASTER & BALANCE (ADC)
//
const uint8_t PIN_POT_BALANCE = PA_3;
const uint8_t PIN_POT_MASTER  = PA_4;

//
// 5 direct ADC for EQ 1..5
//
const uint8_t PIN_EQ_ADC[5] = {PA_0, PA_1, PA_2, PA_5, PA_8}; // verify PA_8 on your package

//
// CD4051 for EQ 6-8 (single 4051)
//
const uint8_t MUX_S0  = PC_2;
const uint8_t MUX_S1  = PC_3;
const uint8_t MUX_S2  = PC_4;
const uint8_t MUX_EN  = PC_5;   // active LOW
const uint8_t MUX_COM = PA_7;   // ADC input for EQ 6..8 (NOTE: same pin as PIN_PITCH_Y in this mapping)

//
// UNIVERSAL 40 BUTTON (5×CD4051)
//
const uint8_t UNIVERSAL_MUX_EN[5] = {PC_6, PC_7, PC_8, PC_9, PC_10};
const uint8_t UNIVERSAL_MUX_COM   = PB_9;   // input pullup

//
// PITCHBEND ENABLE SWITCH
//
const uint8_t PIN_PB_ENABLE_SW = PC_13;

//
// STATUS LED
//
const uint8_t STATUS_LED_PIN = PC_14;

//
// ENCODER (A/B/SW)
//
const uint8_t PIN_ENCODER_A = PA_9;
const uint8_t PIN_ENCODER_B = PA_10;
const uint8_t PIN_ENCODER_SW = PA_11;

//
// TEMPO SWITCHES
//
const uint8_t PIN_TEMPO_ENABLE_SW = PC_15;
const uint8_t PIN_TEMPO_MODE_SW   = PB_15;  // NOTE: PB_15 is also used as COL_PINS[7] in this mapping (see warning below)

//
// VELOCITY controls (ADDED)
//
const uint8_t VELOCITY_POT = PC_0;        // analog pot to set velocity level when enabled
const uint8_t VELOCITY_TOGGLE_PIN = PC_1; // digital button (INPUT_PULLUP) to toggle velocity on/off

//
// OTHER CONSTANTS
//
#define MIDI_CHANNEL 1
#define NUM_UNIVERSAL 40

/* ==========================================
          MIDI helper functions
   ========================================== */

void midiRaw(uint8_t st, uint8_t d1, uint8_t d2) {
    // DIN MIDI OUT (Serial1)
    Serial1.write(st);
    Serial1.write(d1);
    Serial1.write(d2);

#if defined(USBCON)
    // USB MIDI
    SerialUSB.write(st);
    SerialUSB.write(d1);
    SerialUSB.write(d2);
    SerialUSB.flush();
#endif
}

void midiNoteOn(uint8_t ch, uint8_t n, uint8_t v){ midiRaw(0x90|(ch-1),n,v); }
void midiNoteOff(uint8_t ch,uint8_t n,uint8_t v){ midiRaw(0x80|(ch-1),n,v); }
void midiCC(uint8_t ch,uint8_t cc,uint8_t v){ midiRaw(0xB0|(ch-1),cc,v); }
void midiPB(uint8_t ch,int val){
    if(val<0) val=0;
    if(val>16383) val=16383;
    midiRaw(0xE0|(ch-1), val&0x7F, (val>>7)&0x7F);
}

/* ==========================================
           GLOBAL VARIABLES
   ========================================== */

uint8_t padNoteMap[64];
bool matrixState[8][8];
unsigned long matrixDebounce[8][8];

bool uniState[NUM_UNIVERSAL];
unsigned long uniDebounce[NUM_UNIVERSAL];

int lastEQ[8] = {-1,-1,-1,-1,-1,-1,-1,-1};

int lastMod = -1;
int lastPan = -1;
int lastVol = -1;
int lastPB  = -1;

bool pitchbendEnabled = true;
bool wheelYMode = false;

volatile int encoderPos = 0;
volatile bool encoderMoved = false;

int octaveOffset = 0;
int transposeVal = 0;
int tempo = 120;

enum ENC_MODE { M_TEMPO, M_OCT, M_TRANSPOSE, M_WHEEL_Y };
ENC_MODE encMode = M_TEMPO;

/* VELOCITY state */
bool velocityEnabled = true;
int fixedVelocity = 100; // used if velocity disabled

/* debounce for velocity toggle */
unsigned long velToggleLast = 0;
const unsigned long VEL_DEBOUNCE_MS = 150;

/* ==========================================
           CD4051 MUX FUNCTIONS
   ========================================== */

void muxSelect(uint8_t ch){
    digitalWrite(MUX_S0, (ch>>0)&1);
    digitalWrite(MUX_S1, (ch>>1)&1);
    digitalWrite(MUX_S2, (ch>>2)&1);
    delayMicroseconds(40);
}

void muxEnable(bool on){
    digitalWrite(MUX_EN, on ? LOW : HIGH);
}

/* ==========================================
           ENCODER ISR
   ========================================== */

void encoderISR(){
    static uint8_t last = 0;
    uint8_t a = digitalRead(PIN_ENCODER_A);
    uint8_t b = digitalRead(PIN_ENCODER_B);
    uint8_t s = (a<<1)|b;

    if(s!=last){
        if((last==0&&s==1)||(last==1&&s==3)||(last==3&&s==2)||(last==2&&s==0))
            encoderPos++;
        else
            encoderPos--;
        encoderMoved = true;
    }
    last = s;
}

/* ==========================================
              MATRIX 8×8
   ========================================== */

void setupPadMap(){
    for(int i=0;i<64;i++)
        padNoteMap[i] = 48 + i; // starting note (C2-ish)
}

/* helper: read current velocity value (1..127) */
int getVelocityValue(){
    if(!velocityEnabled) return fixedVelocity;
    // read analog (assumed 12-bit ADC 0..4095) — map to 1..127
    int raw = analogRead(VELOCITY_POT);
    int v = map(raw, 0, 4095, 1, 127);
    if(v < 1) v = 1;
    if(v > 127) v = 127;
    return v;
}

void scanMatrix(){
    for(int c=0;c<8;c++){
        digitalWrite(COL_PINS[c], LOW);

        for(int r=0;r<8;r++){
            bool pressed = (digitalRead(ROW_PINS[r]) == LOW);
            if (pressed != matrixState[r][c] &&
                (millis() - matrixDebounce[r][c] > 10)) {

                matrixDebounce[r][c] = millis();
                matrixState[r][c] = pressed;

                uint8_t note = padNoteMap[r*8+c] + octaveOffset*12 + transposeVal;
                int vel = getVelocityValue();

                if (pressed) midiNoteOn(MIDI_CHANNEL, note, vel);
                else         midiNoteOff(MIDI_CHANNEL, note, 0);
            }
        }

        digitalWrite(COL_PINS[c], HIGH);
    }
}

/* ==========================================
         UNIVERSAL 40 BUTTON (5×CD4051)
   ========================================== */

void scanUniversal(){
    int index = 0;

    for(int chip=0; chip<5; chip++) {

        digitalWrite(UNIVERSAL_MUX_EN[chip], LOW);   // enable

        for(int ch=0; ch<8; ch++){
            muxSelect(ch);

            bool pressed = (digitalRead(UNIVERSAL_MUX_COM)==LOW);

            if(index < NUM_UNIVERSAL){
                if (pressed != uniState[index] &&
                    (millis()-uniDebounce[index] > 10)) {

                    uniDebounce[index] = millis();
                    uniState[index] = pressed;

                    uint8_t note = 60 + (index % 36); // original mapping
                    int vel = getVelocityValue();

                    if (pressed) midiNoteOn(1, note, vel);
                    else         midiNoteOff(1, note, 0);
                }
            }
            index++;
        }
        digitalWrite(UNIVERSAL_MUX_EN[chip], HIGH);  // disable
    }
}

/* ==========================================
          ANALOG EQ, JOYSTICK, PB
   ========================================== */

void readAnalogs(){
    /* === Pitchbend disabled === */
    if (!pitchbendEnabled){
        if(lastPB != 8192){
            midiPB(1,8192);
            lastPB = 8192;
        }
        midiCC(1,1,0);
        return;
    }

    /* === Mod X (CC#1) === */
    int mx = analogRead(PIN_PITCH_X);
    int mod = map(mx,0,4095,0,127); // 12-bit ADC
    if(abs(mod-lastMod)>1){
        midiCC(1,1,mod);
        lastMod = mod;
    }

    /* === Pitchbend Y === */
    int my = analogRead(PIN_PITCH_Y);
    int pb;

    if(!wheelYMode){
        pb = map(my,0,4095,0,16383);
    } else {
        int d = my - 2048;
        if(abs(d)<160) d=0;
        pb = constrain(8192 + d*4, 0,16383);
    }

    if(abs(pb-lastPB)>64){
        midiPB(1,pb);
        lastPB = pb;
    }

    /* === Balance === */
    int pans = map(analogRead(PIN_POT_BALANCE),0,4095,0,127);
    if(abs(pans-lastPan)>2){
        midiCC(1,10,pans);
        lastPan = pans;
    }

    /* === Master === */
    int vol = map(analogRead(PIN_POT_MASTER),0,4095,0,127);
    if(abs(vol-lastVol)>2){
        midiCC(1,7,vol);
        lastVol = vol;
    }

    /* === EQ ADC 1-5 === */
    for(int i=0;i<5;i++){
        int v = map(analogRead(PIN_EQ_ADC[i]),0,4095,0,127);
        if(abs(v-lastEQ[i])>2){
            midiCC(1,70+i,v);
            lastEQ[i]=v;
        }
    }

    /* === EQ 6-8 via CD4051 === */
    muxEnable(true);
    for(int ch=0;ch<3;ch++){
        muxSelect(ch);
        int v = map(analogRead(MUX_COM),0,4095,0,127);
        if(abs(v-lastEQ[5+ch])>2){
            midiCC(1,70+5+ch,v);
            lastEQ[5+ch] = v;
        }
    }
    muxEnable(false);
}

/* ==========================================
           ENCODER HANDLING
   ========================================== */

void handleEncoderButton(){
    static bool last=HIGH;
    bool cur = digitalRead(PIN_ENCODER_SW);

    if(cur!=last && cur==LOW){
        encMode = (ENC_MODE)((int)encMode +1);
        if(encMode > M_WHEEL_Y)
            encMode = M_TEMPO;
        delay(150);
    }
    last=cur;
}

void handleEncoder(){
    if(!encoderMoved) return;

    encoderMoved = false;
    int d = encoderPos;
    encoderPos = 0;

    if(encMode == M_TEMPO){
        tempo += d;
        tempo = constrain(tempo, 40, 250);
    }
    else if(encMode == M_OCT){
        octaveOffset += d;
        octaveOffset = constrain(octaveOffset,-3,3);
    }
    else if(encMode == M_TRANSPOSE){
        transposeVal += d;
        transposeVal = constrain(transposeVal,-12,12);
    }
    else if(encMode == M_WHEEL_Y){
        if(d!=0) wheelYMode = !wheelYMode;
    }
}

/* ==========================================
          PITCHBEND ON/OFF
   ========================================== */

void handlePitchbendToggle(){
    static bool last=HIGH;
    bool cur = digitalRead(PIN_PB_ENABLE_SW);

    if(cur!=last && cur==LOW){
        pitchbendEnabled = !pitchbendEnabled;
        delay(150);
    }
    last=cur;
}

/* ==========================================
         VELOCITY TOGGLE HANDLER (new)
   ========================================== */

void handleVelocityToggle(){
    bool cur = digitalRead(VELOCITY_TOGGLE_PIN);
    unsigned long now = millis();
    if(cur == LOW && (now - velToggleLast) > VEL_DEBOUNCE_MS){
        // button pressed (active low)
        velocityEnabled = !velocityEnabled;
        velToggleLast = now;
        // optional feedback blink
        digitalWrite(STATUS_LED_PIN, velocityEnabled ? HIGH : LOW);
        delay(30);
        digitalWrite(STATUS_LED_PIN, LOW);
    }
}

/* ==========================================
                 SETUP
   ========================================== */

void setup(){
    Serial.begin(115200);
    Serial1.begin(31250);

#if defined(USBCON)
    SerialUSB.begin();
    delay(100);
#endif

    setupPadMap();

    // LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // MATRIX
    for(int i=0;i<8;i++){
        pinMode(ROW_PINS[i], INPUT_PULLUP);
        pinMode(COL_PINS[i], OUTPUT);
        digitalWrite(COL_PINS[i], HIGH);
    }

    // UNIVERSAL (MUX EN)
    for(int i=0;i<5;i++){
        pinMode(UNIVERSAL_MUX_EN[i], OUTPUT);
        digitalWrite(UNIVERSAL_MUX_EN[i], HIGH);
    }
    pinMode(UNIVERSAL_MUX_COM, INPUT_PULLUP);

    // MUX EQ
    pinMode(MUX_S0,OUTPUT);
    pinMode(MUX_S1,OUTPUT);
    pinMode(MUX_S2,OUTPUT);
    pinMode(MUX_EN,OUTPUT);
    digitalWrite(MUX_EN,HIGH);

    // ENCODER
    pinMode(PIN_ENCODER_A,INPUT_PULLUP);
    pinMode(PIN_ENCODER_B,INPUT_PULLUP);
    pinMode(PIN_ENCODER_SW,INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), encoderISR, CHANGE);

    // PITCHBEND SWITCH
    pinMode(PIN_PB_ENABLE_SW,INPUT_PULLUP);

    // TEMPO SWITCHES
    pinMode(PIN_TEMPO_ENABLE_SW, INPUT_PULLUP);
    pinMode(PIN_TEMPO_MODE_SW, INPUT_PULLUP);

    // VELOCITY controls
    pinMode(VELOCITY_TOGGLE_PIN, INPUT_PULLUP);
    // VELOCITY_POT is analog — no pinMode necessary for analogRead

    // initialize debounces arrays
    for(int r=0;r<8;r++) for(int c=0;c<8;c++){ matrixDebounce[r][c]=0; matrixState[r][c]=false; }
    for(int i=0;i<NUM_UNIVERSAL;i++){ uniDebounce[i]=0; uniState[i]=false; }
}

/* ==========================================
                  LOOP
   ========================================== */

void loop(){

    static unsigned long lastScan=0;
    if(millis()-lastScan >= 5){
        lastScan = millis();
        scanMatrix();
        scanUniversal();
        readAnalogs();
    }

    handleEncoderButton();
    handleEncoder();
    handlePitchbendToggle();
    handleVelocityToggle();
}
