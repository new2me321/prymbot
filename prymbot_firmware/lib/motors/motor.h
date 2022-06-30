#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void left(bool motor_direction, int speed);
void right(bool motor_direction, int speed);
void motor_setup();

#endif