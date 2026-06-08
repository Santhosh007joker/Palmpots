//Angle Sensing
#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>

// Pin Definitions

#define MUX_S0     4        // CD4051 address bit 0  
#define MUX_S1     5        // CD4051 address bit 1
#define MUX_S2     6        // CD4051 address bit 2  
#define MUX_OUT    A0       // CD4051 COM pin  →  MCU ADC input

#define IR_TX_PIN  3        // This bot's IR LED  (OC2B, Timer2 output)

// Configuration 

#define SLOT_MS           15    // Duration of each bot's IR-on transmission (ms)
#define GUARD_MS          3     // Dead-time between consecutive slots (ms)
#define IR_STABILISE_US   500   // Wait after neighbor IR turns on before reading (µs)

#define MUX_SETTLE_US     15    // Mux switch + RC filter settling time (µs)
#define ADC_GAP_US        50    // Gap between successive analogRead() calls (µs)
#define ADC_AVG_SAMPLES   4     // Number of ADC reads averaged per channel

// A bearing is reported only if the total weighted signal exceeds this threshold.
// Tune upward or lower based on experimental res.
#define MIN_WEIGHT_THRESH  30

#define EEPROM_ID_ADDR    0     // EEPROM byte that stores this bot's ID (0–5)

// Sensor Geometry 

// Photodiodes are mounted at 60° intervals, starting from the bot's nose (0 deg).
// CCW is positive by convention, consistent with standard math / atan2.

const float SENSOR_ANGLES_RAD[6] = {
    0.0f,                       //   0°
    M_PI / 3.0f,                //  60°
    2.0f * M_PI / 3.0f,        // 120°
    M_PI,                       // 180°
    4.0f * M_PI / 3.0f,        // 240°
    5.0f * M_PI / 3.0f         // 300°
};

// Global State 

uint8_t BOT_ID;            // Loaded from EEPROM in setup()

// Ambient ADC readings (raw, 0–1023) with all IR LEDs off.
// Sampled once at the start of every sensing cycle.
int g_ambient[6];

// Net IR intensity from each neighbor at each sensor.
// g_net_intensity[j][k] = g_ambient[k] - raw_reading_during_slot_j
// Positive- more IR than ambient (neighbor detected)
// Zero/neg- no IR from that neighbor
int g_net_intensity[6][6];

// Computed body-frame bearing to each bot (radians, range (-π, π]).
// NAN- self, or neighbor not detected this cycle.
float g_bearing_to[6];



//Write a 3-bit address to the CD4051 select pins, choosing which of the 6 photdiodes is connected to input pin.
 
void selectChannel(uint8_t ch) {
    digitalWrite(MUX_S0, (ch     ) & 0x01);
    digitalWrite(MUX_S1, (ch >> 1) & 0x01);
    digitalWrite(MUX_S2, (ch >> 2) & 0x01);
    delayMicroseconds(MUX_SETTLE_US);   // RC filter + mux switch settling
}


// Select channel ch, then read the ADC n times and return the integer average.
 
int readChannelAveraged(uint8_t ch, uint8_t n) {
    selectChannel(ch);
    long sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += analogRead(MUX_OUT);
        delayMicroseconds(ADC_GAP_US);  // let ADC capacitor recharge between reads
    }
    return (int)(sum / n);
}


void readAllSensors(int out[6], uint8_t n_avg) {
    for (uint8_t k = 0; k < 6; k++) {
        out[k] = readChannelAveraged(k, n_avg);
    }
}


/*
 * irOn() / irOff()
 * Control this bot's IR LED using Timer2 CTC mode on pin 3 (OC2B).
 *
 * Frequency calculation  (ATmega328P @ 8 MHz, no prescaler):
 *   f = F_CPU / (2 × (OCR2A + 1))
 *     = 8,000,000 / (2 × 105)
 *     = 38,095 Hz  ≈  38 kHz
 *
 * Timer2 register bits used:
 *   COM2B0 = 1  →  toggle OC2B (pin 3) on compare match
 *   WGM21  = 1  →  CTC mode (clear timer on compare with OCR2A)
 *   CS20   = 1  →  no prescaler (clock source = F_CPU)
 */
void irOn() {
    pinMode(IR_TX_PIN, OUTPUT);
    OCR2A  = 104;
    TCCR2A = (1 << COM2B0) | (1 << WGM21);   // CTC, toggle OC2B
    TCCR2B = (1 << CS20);                     
}

void irOff() {
    TCCR2A = 0;
    TCCR2B = 0;
    digitalWrite(IR_TX_PIN, LOW);
}



void sampleAmbient() {
    irOff();    // Ensure this bot's LED is off
    readAllSensors(g_ambient, ADC_AVG_SAMPLES);
}



float computeBearing(uint8_t bot_j) {
    float sum_sin   = 0.0f;
    float sum_cos   = 0.0f;
    float total_w   = 0.0f;

    for (uint8_t k = 0; k < 6; k++) {

        float w = (float)g_net_intensity[bot_j][k];
        if (w < 0.0f) w = 0.0f;

        sum_sin += w * sinf(SENSOR_ANGLES_RAD[k]);
        sum_cos += w * cosf(SENSOR_ANGLES_RAD[k]);
        total_w += w;
    }

    // If total signal is below noise floor, declare neighbor undetected
    if (total_w < (float)MIN_WEIGHT_THRESH) {
        return NAN;
    }

    return atan2f(sum_sin, sum_cos);   
}

float* distance_sensing(uint8_t bot_j){
    float dist[6];
    dist[bot_j] = 0;
    for(i = 0; i<6; i++){
        if(g_net_intensity[bot_j][i] > 0){
            //dist = f(I) ....to be calliberated.
        }

    }
    return dist;
}


 
void runSensingCycle() {

    sampleAmbient();

    for (uint8_t bot_j = 0; bot_j < 6; bot_j++) {

        unsigned long slot_start = millis();

        if (bot_j == BOT_ID) {
            // OWN SLOT: turn IR on for the full slot duration
            irOn();
            // Timer2 runs autonomously; just wait out the slot.
            while (millis() - slot_start < (unsigned long)SLOT_MS) { } //wait 
            irOff();

        }
        else {
            // NEIGHBOR'S SLOT: read all photodiodes
            //
            // Brief stabilisation wait: gives the neighbor bot's IR LED and
            // our photodiode RC circuit time to reach steady state after the
            // slot boundary.
            delayMicroseconds(IR_STABILISE_US);

            int raw[6];
            readAllSensors(raw, ADC_AVG_SAMPLES);

            for (uint8_t k = 0; k < 6; k++) {
                g_net_intensity[bot_j][k] = g_ambient[k] - raw[k]; // Reson: intensity(propn) = (1023 - raw_reading) - (1023 - ambient) = ambient - raw
            }

            // Wait out the remainder of the slot
            while (millis() - slot_start < (unsigned long)SLOT_MS) { } //wait
        }

        // Guard interval: all IRs off, no readings taken.
        irOff();
        delay(GUARD_MS);
    }

    
    for (uint8_t bot_j = 0; bot_j < 6; bot_j++) {
        if (bot_j == BOT_ID) {
            g_bearing_to[bot_j] = NAN;      // No bearing to self
            continue;
        }
        g_bearing_to[bot_j] = computeBearing(bot_j);
    }
}


void setup() {
    Serial.begin(115200);

    // Load bot ID from EEPROM
    BOT_ID = EEPROM.read(EEPROM_ID_ADDR);

    // CD4051 mux control pins
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    selectChannel(0);   // Default selection

    analogReference(DEFAULT);

    // Discard the first few ADC readings — the ATmega328P ADC takes one or two
    // "warm-up" conversions after analogReference() changes.
    for (uint8_t i = 0; i < 5; i++) analogRead(MUX_OUT);

    pinMode(IR_TX_PIN, OUTPUT);
    irOff();
    
    // initialization
    for (uint8_t j = 0; j < 6; j++) {
        g_bearing_to[j] = NAN;
        for (uint8_t k = 0; k < 6; k++) {
            g_net_intensity[j][k] = 0;
        }
    }
    for (uint8_t k = 0; k < 6; k++) {
        g_ambient[k] = 0;     
    }

}


void loop() {

}