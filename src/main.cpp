#include <Arduino.h>

/* ==========================================
   STM32 PIN DEFINITIONS (SAFE + USB OK)
   ========================================== */

// MATRIX 8x8
const uint8_t ROW_PINS[8] = {PA_0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7};
const uint8_t COL_PINS[8] = {PB_0, PB_1, PB_2, PB_10, PB_11, PB_12, PB_13, PB_14};

// JOYSTICK X/Y
const uint8_t PIN_PITCH_X = PA_0;
const uint8_t PIN_PITCH_Y = PA_1;

// MASTER & BALANCE
const uint8_t PIN_POT_BALANCE = PA_3;
const uint8_t PIN_POT_MASTER  = PA_4;

// 5 langsung ADC
const uint8_t PIN_EQ_ADC[5] = {PA_5, PA_6, PA_7, PB_0, PB_1};

// CD4051 for EQ 6-8
const uint8_t MUX_S0  = PB_3;
const uint8_t MUX_S1  = PB_4;
const uint8_t MUX_S2  = PB_5;
const uint8_t MUX_EN  = PC_11;   // active LOW
const uint8_t MUX_COM = PA_2;    // ADC

// UNIVERSAL BUTTON (40 BTN = 5×CD4051)
const uint8_t UNIVERSAL_MUX_EN[5] = {PC_6, PC_7, PC_8, PC_9, PC_10};
const uint8_t UNIVERSAL_MUX_COM   = PB_9;

// PITCHBEND ENABLE
const uint8_t PIN_PB_ENABLE_SW = PC_13;

// LED
const uint8_t STATUS_LED_PIN = PC_14;

// ENCODER
const uint8_t PIN_ENCODER_A = PB_6;
const uint8_t PIN_ENCODER_B = PB_7;
const uint8_t PIN_ENCODER_SW = PB_8;

// TEMPO SWITCHES
const uint8_t PIN_TEMPO_ENABLE_SW = PC_15;
const uint8_t PIN_TEMPO_MODE_SW   = PB_15;

// CONSTANTS
#define MIDI_CHANNEL 1
#define NUM_UNIVERSAL 40


/* ==========================================
          MIDI OUT (USB + DIN)
   ========================================== */

void midiRaw(uint8_t st, uint8_t d1, uint8_t d2) {
    // DIN MIDI OUT
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
        padNoteMap[i] = 48 + i;
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

                if (pressed) midiNoteOn(MIDI_CHANNEL, note, 100);
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

                    if (pressed) midiNoteOn(1, 60+(index%36), 100);
                    else         midiNoteOff(1, 60+(index%36), 0);
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
    int mod = map(mx,0,1023,0,127);
    if(abs(mod-lastMod)>1){
        midiCC(1,1,mod);
        lastMod = mod;
    }

    /* === Pitchbend Y === */
    int my = analogRead(PIN_PITCH_Y);
    int pb;

    if(!wheelYMode){
        pb = map(my,0,1023,0,16383);
    } else {
        int d = my - 512;
        if(abs(d)<40) d=0;
        pb = constrain(8192 + d*16, 0,16383);
    }

    if(abs(pb-lastPB)>16){
        midiPB(1,pb);
        lastPB = pb;
    }

    /* === Balance === */
    int pans = map(analogRead(PIN_POT_BALANCE),0,1023,0,127);
    if(abs(pans-lastPan)>2){
        midiCC(1,10,pans);
        lastPan = pans;
    }

    /* === Master === */
    int vol = map(analogRead(PIN_POT_MASTER),0,1023,0,127);
    if(abs(vol-lastVol)>2){
        midiCC(1,7,vol);
        lastVol = vol;
    }

    /* === EQ ADC 1-5 === */
    for(int i=0;i<5;i++){
        int v = map(analogRead(PIN_EQ_ADC[i]),0,1023,0,127);
        if(abs(v-lastEQ[i])>2){
            midiCC(1,70+i,v);
            lastEQ[i]=v;
        }
    }

    /* === EQ 6-8 via CD4051 === */
    muxEnable(true);
    for(int ch=0;ch<3;ch++){
        muxSelect(ch);
        int v = map(analogRead(MUX_COM),0,1023,0,127);
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

    // MATRIX
    for(int i=0;i<8;i++){
        pinMode(ROW_PINS[i], INPUT_PULLUP);
        pinMode(COL_PINS[i], OUTPUT);
        digitalWrite(COL_PINS[i], HIGH);
    }

    // UNIVERSAL
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
}
