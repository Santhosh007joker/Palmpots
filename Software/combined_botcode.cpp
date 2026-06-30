#include <iostream>
#include <cmath>
#include <RF24.h>
#include <SPI.h>
using namespace std;

#define MUX_S0     4        // CD4051 address bit 0  
#define MUX_S1     5        // CD4051 address bit 1
#define MUX_S2     6        // CD4051 address bit 2  
#define MUX_OUT    A0       // CD4051 COM pin: MCU ADC input

#define IR_TX_PIN  3        // This bot's IR LED  (OC2B, Timer2 output)

// Configuration 

#define SLOT_MS           15    // Duration of each bot's IR-on transmission (ms)
#define GUARD_MS          3     // Dead-time between consecutive slots (ms)
#define IR_STABILISE_US   500   // Wait after neighbor IR turns on before reading (µs)

#define MUX_SETTLE_US     15    // Mux switch + RC filter settling time (µs)
#define ADC_GAP_US        50    // Gap between successive analogRead() calls (µs)
#define ADC_AVG_SAMPLES   4     // Number of ADC reads averaged per channel

const float SENSOR_ANGLES_RAD[6] = {
    0.0f,                      //   0°
    M_PI / 3.0f,               //  60°
    2.0f * M_PI / 3.0f,        // 120°
    M_PI,                      // 180°
    4.0f * M_PI / 3.0f,        // 240°
    5.0f * M_PI / 3.0f         // 300°
};

uint8_t BOT_ID;
int g_ambient[6];
int g_net_intensity[6][6];
float g_bearing_to[6];

void selectChannel(uint8_t ch) {
    digitalWrite(MUX_S0, (ch     ) & 0x01);
    digitalWrite(MUX_S1, (ch >> 1) & 0x01);
    digitalWrite(MUX_S2, (ch >> 2) & 0x01);
    delayMicroseconds(MUX_SETTLE_US);   // RC filter + mux switch settling
}

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

float* distance_sensing(uint8_t bot_j){
    float dist[6];
    dist[bot_j] = 0;
    for(i = 0; i<6; i++){
        if(g_net_intensity[bot_j][i] > 0){
            //dist[i] = f(I) ....to be calliberated.
        }

    }
    return dist;
}



int state = 0;
bool flag = false;
float v[2];
float distances[5];
float angles[5] = [0, 0, 0, 0, 0, 0];
const float moving_time = 600   // ms  // we'll choose the right value after experimentation 
const float alpha = 0.1
const unsigned float eps = 0.0001

float anglereq1 = 30*M_PI/180;
float anglereq2 = 60*M_PI/180;
float anglereq3 = 90*M_PI/180;
float anglereq4 = 120*M_PI/180;

void move(float* v)
{
  float vx = v[0];
  float vy = v[1];
  float Voltages[2];
  v[0] = 0;
  v[1] = 0;
  //Get the required voltages to produce motion in the desired direction
  get_voltages(vx, vy, Voltages);
  //Now PWM Write these voltages to the terminals of the vibration motors
}



void setup() 
{
  // put your setup code here, to run once:
}

void loop() 
{
    while(abs(angles[1] - anglereq1)>eps || abs(angles[2] - anglereq2)>eps || abs(angles[3] - anglereq3)>eps || abs(angles[4] - anglereq4)>eps){
    runSensingCycle();
    uint8_t next_bot = (BOT_ID + 1) % 6
    for(int i = 1; i < 6; i++){
        angles[i] = abs(g_bearing_to[(i+BOT_ID)%6] - g_bearing_to[next_bot])
        distances[i] = distance_sensing(BOT_ID)[(i+BOT_ID)%6]
    }
    float v1[2], v2[2], v3[2], v4[2], v5[2];
    update(distances, angles, v, v1, v2, v3, v4, v5);
    float next_pos[2];
    next_pos[0] = -alpha*v[0]
    next_pos[1] = -alpha*v[1]
    unsigned long start = millis();
    while(millis() - start < moving_time){
        move(next_pos);   // all bots try to go as close as poss to the desired neext/new posn
    }
  }
}



void update(float* dist, float* ang, float* v, float* v1, float* v2, float* v3, float* v4, float* v5)
{
  float* p1[2] = [dist[0]*cos(ang[0]), dist[0]*sin(ang[0])];
  float* p2[2] = [dist[1]*cos(ang[1]), dist[1]*sin(ang[1])];
  float* p3[2] = [dist[2]*cos(ang[2]), dist[2]*sin(ang[2])];
  float* p4[2] = [dist[3]*cos(ang[3]), dist[3]*sin(ang[3])];
  float* p5[2] = [dist[4]*cos(ang[4]), dist[4]*sin(ang[4])];

  get_vdot(p1, p2, p3, p4, p5, 1, v1);
  get_vdot(p1, p2, p3, p4, p5, 2, v2);
  get_vdot(p1, p2, p3, p4, p5, 3, v3);
  get_vdot(p1, p2, p3, p4, p5, 4, v4);
  get_vdot(p1, p2, p3, p4, p5, 5, v5);
  
  v[0] = -(v1[0] + v2[0] + v3[0] + v4[0] + v5[0]);
  v[1] = -(v1[1] + v2[1] + v3[1] + v4[1] + v5[1]);
}

float get_distance(float* p1, float* p2)
{
  return sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]));
}

float get_angle(float* p1, float* p2, float* p3)
{
   float angle = acos(((p1[0] - p2[0])*(p1[0] - p3[0]) + (p1[1] - p2[1])*(p1[1] - p3[1]))/(get_distance(p1, p2)*get_distance(p1, p3)));
   return angle;
}

void get_energy(float* p1, float* p2, float* p3, float* p4, float* p5)
{
  float p0[2] = [0.0, 0.0];
  float angle1 = get_angle(p0, p1, p2);
  float angle2 = get_angle(p0, p1, p3);
  float angle3 = get_angle(p0, p1, p4);
  float angle4 = get_angle(p0, p1, p5);

  float E = 4 - cos(angle1req - angle1) - cos(angle2req - angle2) - cos(angle3req - angle3) - cos(angle4req - angle4);
  return E;
}

void get_vdot(float* p1, float* p2, float* p3, float* p4, float* p5, int i, float* v)
{
  float h = 0.01;
  if(i==1)
  {
    p1[0] = p1[0] + h;
    float Epx = get_energy(p1, p2, p3, p4, p5);
    p1[0] = p1[0] - 2*h;
    float Emx = get_energy(p1, p2, p3, p4, p5);
    p1[0] = p1[0] + h;

    p1[1] = p1[1] + h;
    float Epy = get_energy(p1, p2, p3, p4, p5);
    p1[1] = p1[1] - 2*h;
    float Emy = get_energy(p1, p2, p3, p4, p5);
    p1[1] = p1[1] + h;

    v[0] = (Emx - Epx)/(2*h);
    v[1] = (Emy - Epy)/(2*h);
    return
  }
  if(i==2)
  {
    p2[0] = p2[0] + h;
    float Epx = get_energy(p1, p2, p3, p4, p5);
    p2[0] = p2[0] - 2*h;
    float Emx = get_energy(p1, p2, p3, p4, p5);
    p2[0] = p2[0] + h;

    p2[1] = p2[1] + h;
    float Epy = get_energy(p1, p2, p3, p4, p5);
    p2[1] = p2[1] - 2*h;
    float Emy = get_energy(p1, p2, p3, p4, p5);
    p2[1] = p2[1] + h;

    v[0] = (Emx - Epx)/(2*h);
    v[1] = (Emy - Epy)/(2*h);
    return
  }
  if(i==3)
  {
    p3[0] = p3[0] + h;
    float Epx = get_energy(p1, p2, p3, p4, p5);
    p3[0] = p3[0] - 2*h;
    float Emx = get_energy(p1, p2, p3, p4, p5);
    p3[0] = p3[0] + h;

    p3[1] = p3[1] + h;
    float Epy = get_energy(p1, p2, p3, p4, p5);
    p3[1] = p3[1] - 2*h;
    float Emy = get_energy(p1, p2, p3, p4, p5);
    p3[1] = p3[1] + h;

    v[0] = (Emx - Epx)/(2*h);
    v[1] = (Emy - Epy)/(2*h);
    return
  }
  if(i==4)
  {
    p4[0] = p4[0] + h;
    float Epx = get_energy(p1, p2, p3, p4, p5);
    p4[0] = p4[0] - 2*h;
    float Emx = get_energy(p1, p2, p3, p4, p5);
    p4[0] = p4[0] + h;

    p4[1] = p4[1] + h;
    float Epy = get_energy(p1, p2, p3, p4, p5);
    p4[1] = p4[1] - 2*h;
    float Emy = get_energy(p1, p2, p3, p4, p5);
    p4[1] = p4[1] + h;

    v[0] = (Emx - Epx)/(2*h);
    v[1] = (Emy - Epy)/(2*h);
    return
  }
  if(i==5)
  {
    p5[0] = p5[0] + h;
    float Epx = get_energy(p1, p2, p3, p4, p5);
    p5[0] = p5[0] - 2*h;
    float Emx = get_energy(p1, p2, p3, p4, p5);
    p5[0] = p5[0] + h;

    p5[1] = p5[1] + h;
    float Epy = get_energy(p1, p2, p3, p4, p5);
    p5[1] = p5[1] - 2*h;
    float Emy = get_energy(p1, p2, p3, p4, p5);
    p5[1] = p5[1] + h;

    v[0] = (Emx - Epx)/(2*h);
    v[1] = (Emy - Epy)/(2*h);
    return
  }
}
