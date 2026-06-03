#include <iostream>
#include <cmath>
using namespace std;

angle1req = 30;
angle2req = 60;
angle3req = 90;
angle4req = 120;
float get_distance(float* p1, float* p2)
{
  return sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]));
}
float get_angle(float* p1, float* p2, float* p3)
{
   float angle = acos(((p1[0] - p2[0])*(p1[0] - p3[0]) + (p1[1] - p2[1])*(p1[1] - p3[1]))/(get_distance(p1, p2)*get_distance(p1, p3)));
   return angle;
}
float get_vdotangle(float* dist, float* ang)
{
  float Emin = 100;
  float imin = 1000;
  float r = 0.001;

  float* p1[2] = [dist[0]*cos(ang[0]), dist[0]*sin(ang[0])];
  float* p2[2] = [dist[1]*cos(ang[1]), dist[1]*sin(ang[1])];
  float* p3[2] = [dist[2]*cos(ang[2]), dist[2]*sin(ang[2])];
  float* p4[2] = [dist[3]*cos(ang[3]), dist[3]*sin(ang[3])];
  float* p5[2] = [dist[4]*cos(ang[4]), dist[4]*sin(ang[4])];

  for(float i = 0; i<2*np.pi; i+=0.12)
  {
    float* p0[2] = [r*cos(i), r*sin(i)];
    float angle1 = get_angle(p0, p1, p2);
    float angle2 = get_angle(p0, p1, p3);
    float angle3 = get_angle(p0, p1, p4);
    float angle4 = get_angle(p0, p1, p5);

    float E = 4 - cos(angle1req - angle1) - cos(angle2req - angle2) - cos(angle3req - angle3) - cos(angle4req - angle4);
    if(E < Emin)
    {
      Emin = E;
      imin = i;
    }
  }
  return imin;
}