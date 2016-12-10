#include "pid_follower.h"

void eviteur(void)
{
  gauche(128);
  delay( temps);

  avance(128);
  delay( temps);

  droite(128);
  delay(temps);

  avance(128);
  delay(temps);

  droite(128);
  delay(temps);


  read_sensor_values();
  avance(128);

  while (!((sensor[0] == on) || (sensor[1] == on) || (sensor[2] == on) || (sensor[3] == on) || (sensor[4] == on)))read_sensor_values();

  }
