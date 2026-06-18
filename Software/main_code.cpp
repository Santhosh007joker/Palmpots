#include <iostream>
#include <cmath>
using namespace std;

int state = 0;
bool flag = false;
float v[2];
float distances[5];
float angles[5];

float anglereq1 = 30*np.pi/180;
float anglereq2 = 60*np.pi/180;
float anglereq3 = 90*np.pi/180;
float anglereq4 = 120*np.pi/180;

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
  // put your main code here, to run repeatedly:
  //First step: Move as per the vectors stored.
  if(state==id)
    move(v);
  if(state==0)
  {
    //Active phase of the bot, in which it moves and is the receiver


    //Second step: Get bearing and distance data.
    float intensities[5];
    for(int i = 1; i < 6; i++)
    {
      get_intensities(intensities); //Gets the six intensity values for one bot-to-bot measurement 
      extract_data(intensities, distances, angles);
      nrf_send(STOP, i); //Broadcast stop bit to bot i, and also to bot (i+1)%6 to signal it to switch on IR
      nrf_send(STOP, (i+1)%6);
      delay(5); //Delay to prevent errors from overlapping time instants before bot i switches off and after bot (i+1)%6 turns on
    }

    //Third step: Get update move.
    float v1[2], v2[2], v3[2], v4[2], v5[2];
    update(distances, angles, v, v1, v2, v3, v4, v5);
    nrf_send(V1DATA, (id+1)%6);
    nrf_send(V2DATA, (id+2)%6);
    nrf_send(V3DATA, (id+3)%6);
    nrf_send(V4DATA, (id+4)%6);
    nrf_send(V5DATA, (id+5)%6);
    delay(20); //Delay to make sure everyone receives their data before updating state

    //Fourth step: Update state.
    state = (state+1)%6;
  }
  else
  {
    //Check if any bit is received from bot (id-state)%6; If yes, then swtich on IR; If no, keep checking.
    ir_on();
    //Check if any bit is received from bot (id-state)%6; If yes, then swtich off IR; If no, keep checking.
    ir_off();
    //Check if any data is received from bot (id-state)%6; If yes, then execute the following; If no, keep checking.
    delay(20);
    state = (state+1)%6;
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
