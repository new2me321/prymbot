#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

void enc_R_CW();
void enc_L_CW();
void enc_R_CCW();
void enc_L_CCW();
short read_left_RPM(bool motor_direction);
short read_right_RPM(bool motor_direction);

short read_left_ticks(bool motor_direction);
short read_right_ticks(bool motor_direction);

void encoder_setup();

#endif