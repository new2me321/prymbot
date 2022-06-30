#include <motor.h>

// Define motor pins
#define right_motor_fwd 2 // right_forward
#define right_motor_bwd 3 // right_backward
#define left_motor_fwd 4 // left_forward
#define left_motor_bwd 5 // left_backward
#define ena_pin 9  // enable PWM for right motor
#define enb_pin 10 // enable PWM for left motor

//  receives velocity mgs

void motor_setup()
{
    //  Set L298N pins on nano
    pinMode(right_motor_fwd, OUTPUT);
    pinMode(right_motor_bwd, OUTPUT);
    pinMode(left_motor_fwd, OUTPUT);
    pinMode(left_motor_bwd, OUTPUT);
    pinMode(ena_pin, OUTPUT);
    pinMode(enb_pin, OUTPUT);

    // ensure that the motor is in neutral state during bootup
    analogWrite(ena_pin, abs(0));
    analogWrite(enb_pin, abs(0));
}

void left(bool motor_direction, int speed)
{
    //  Drive Left motor
    digitalWrite(left_motor_fwd, motor_direction);
    digitalWrite(left_motor_bwd, !motor_direction);
    analogWrite(enb_pin, speed);
}

void right(bool motor_direction, int speed)
{
    //  Drive Right motor
    digitalWrite(right_motor_fwd, motor_direction);
    digitalWrite(right_motor_bwd, !motor_direction);
    analogWrite(ena_pin, speed);
}
