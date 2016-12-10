#include "pid_follower.h"

void avance(unsigned int speed)
{
  analogWrite(pwm_right, speed);
  analogWrite(pwm_left, speed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
}
void gauche(unsigned int speed)
{
  analogWrite(pwm_right, speed);
  analogWrite(pwm_left, speed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}
void droite(unsigned int speed)
{
  analogWrite(pwm_right, speed);
  analogWrite(pwm_left, speed);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
}


void stopp()
{
 analogWrite(pwm_right, 0);
  analogWrite(pwm_left, 0);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward,LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}


