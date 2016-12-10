#include "pid_follower.h"
#include <arduino.h>
//


void read_sensor_values(void)
{
  /*
    sensor[0]=digitalRead(A0);
    sensor[1]=digitalRead(A1);
    sensor[2]=digitalRead(A2);
    sensor[3]=digitalRead(A3);
    sensor[4]=digitalRead(A4);
  */
  sensor[0] = digitalRead(A4);
  sensor[1] = digitalRead(A3);
  sensor[2] = digitalRead(A2);
  sensor[3] = digitalRead(A1);
  sensor[4] = digitalRead(A0);

  if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == off) && (sensor[3] == off) && (sensor[4] == on))
    error = 4;
  else if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == off) && (sensor[3] == on) && (sensor[4] == on))
    error = 3;
  else if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == off) && (sensor[3] == on) && (sensor[4] == off))
    error = 2;
  else if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == on) && (sensor[3] == on) && (sensor[4] == off))
    error = 1.5;
  else if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == on) && (sensor[3] == on) && (sensor[4] == off))
    error = 1;
  else if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == on) && (sensor[3] == off) && (sensor[4] == off))
    error = 0;
    else if ((sensor[0] == off) && (sensor[1] == on) && (sensor[2] == on) && (sensor[3] == on) && (sensor[4] == off))
    error = 0;
  else if ((sensor[0] == off) && (sensor[1] == on) && (sensor[2] == on) && (sensor[3] == off) && (sensor[4] == off))
    error = -1;
    else if ((sensor[0] == on) && (sensor[1] == on) && (sensor[2] == on) && (sensor[3] == off) && (sensor[4] == off))
    error = -1.5;
  else if ((sensor[0] == off) && (sensor[1] == on) && (sensor[2] == off) && (sensor[3] == off) && (sensor[4] == off))
    error = -2;
  else if ((sensor[0] == on) && (sensor[1] == on) && (sensor[2] == off) && (sensor[3] == off) && (sensor[4] == off))
    error = -3;
  else if ((sensor[0] == on) && (sensor[1] == off) && (sensor[2] == off) && (sensor[3] == off) && (sensor[4] == off))
    error = -4;
  else if ((sensor[0] == off) && (sensor[1] == off) && (sensor[2] == off) && (sensor[3] == off) && (sensor[4] == off))
    if (error < 0) error = -5;
    else error = 5;

}

