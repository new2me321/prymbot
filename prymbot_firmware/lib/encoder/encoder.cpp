/*
 * Optical Encoder library
 *
 * By: Kwasi
 */

#include <encoder.h>

#define ENCODER_1_PIN 2
#define ENCODER_2_PIN 3
#define RESOLUTION 20 // number of disk ticks

volatile long counter1 = 0;
volatile long counter2 = 0;

volatile long tick_r = 0;
volatile long tick_l = 0;

unsigned long Start_Timer1 = 0;
unsigned long Start_Read_Timer1 = 0;
unsigned long Start_Timer2 = 0;
unsigned long Start_Read_Timer2 = 0;

void encoder_setup()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), enc_R_CW, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), enc_L_CW, RISING);
}

// clockwise ticks counter
void enc_R_CW()
{
  counter1 = counter1 + 1;
  tick_r++;
}

void enc_L_CW()
{
  counter2 = counter2 + 1;
  tick_l ++;
}

// counter clockwise(reversed) ticks counter
void enc_R_CCW()
{
  counter1 = counter1 - 1;
  tick_r--;
}

void enc_L_CCW()
{
  counter2 = counter2 - 1;
  tick_l--;
}

//  function reads RPM on right encoder and return value
short read_right_RPM(bool motor_direction)
{
  unsigned long RPM_1 = 0;
  unsigned long TimeDif1 = 0;

  detachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN));

  TimeDif1 = millis() - Start_Timer1;
  RPM_1 = 60000 * counter1;
  RPM_1 = RPM_1 / TimeDif1;
  RPM_1 = RPM_1 / RESOLUTION;

  counter1 = 0;
  Start_Timer1 = millis();
  if (motor_direction)
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), enc_R_CW, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), enc_R_CCW, RISING);
  }
  // Serial.print("RPM1: ");
  // Serial.println(RPM_1);
  return (short)RPM_1;
}

//  function reads RPM on left encoder and return value
short read_left_RPM(bool motor_direction)
{
  unsigned long RPM_2 = 0;
  unsigned long TimeDif2 = 0;

  detachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN));

  TimeDif2 = millis() - Start_Timer2;
  RPM_2 = 60000 * counter2;
  RPM_2 = RPM_2 / TimeDif2;
  RPM_2 = RPM_2 / RESOLUTION;

  counter2 = 0;
  Start_Timer2 = millis();
  if (motor_direction)
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), enc_L_CW, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), enc_L_CCW, RISING);
  }
  // Serial.print("RPM2: ");
  // Serial.println(RPM_2);
  return (short)RPM_2;
}

//  function reads Ticks count on right encoder and return value
short read_right_ticks(bool motor_direction)
{
  unsigned long tick_1 = 0;

  detachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN));
  tick_1 = tick_r;

  if (motor_direction)
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), enc_R_CW, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN), enc_R_CCW, RISING);
  }
  return (short)tick_1;
}

//  function reads Ticks count on left encoder and return value
short read_left_ticks(bool motor_direction)
{
  unsigned long tick_2 = 0;

  detachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN));
  tick_2 = tick_l;

  if (motor_direction)
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), enc_L_CW, RISING);
  }
  else
  {
    attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN), enc_L_CCW, RISING);
  }
  return (short)tick_2;
}
