/*
 * rosserial Differential Drive node
 * Drives motors on a callback
 *
 * By: Kwasi
 */

#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <motor.h>

double rightSpeed = 0;
double leftSpeed = 0;
boolean motor_left;
boolean motor_right;
float linear_x;
float angular_z;

// Instantiate the node handle
ros::NodeHandle nh;

long remap(float x, float in_min, float in_max, long out_min, long out_max)
{
  if (abs(x) > 1)
  {
    x = 1;
  }
  else
  {
    x = abs(x);
  }

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drive()
{
  rightSpeed = linear_x + (angular_z / 2);
  leftSpeed = linear_x - (angular_z / 2);

  if (linear_x > 0 && angular_z == 0)
  {
    // forward
    motor_left = 1;
    motor_right = 1;
    // nh.loginfo("forward");
  }
  if (linear_x < 0 && angular_z == 0)
  {
    // backward
    motor_left = 0;
    motor_right = 0;
    // nh.loginfo("backward");
  }
  if (linear_x == 0 && angular_z > 0)
  {
    // left
    motor_left = 0;
    motor_right = 1;
    // nh.loginfo("left");
  }
  if (linear_x == 0 && angular_z < 0)
  {
    // right
    motor_left = 1;
    motor_right = 0;
    // nh.loginfo("right");
  }
  if (linear_x > 0 && (angular_z > 0 || angular_z < 0))
  {
    // make forward circular turn
    motor_left = 1;
    motor_right = 1;
    // nh.loginfo("make forward circular turn ");
  }

  if (linear_x < 0 && (angular_z > 0 || angular_z < 0))
  {
    // make backward circular turn
    motor_left = 0;
    motor_right = 0;
    // nh.loginfo("make backward circular turn ");
  }

  // Adjusts speed of robot wrt sensitivity of controller
  rightSpeed = remap(rightSpeed, 0, 1, 0, 255);
  leftSpeed = remap(leftSpeed, 0, 1, 0, 255);

  left(motor_left, leftSpeed);
  right(motor_right, rightSpeed);
}

void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel)
{

  linear_x = cmd_vel.linear.x;
  angular_z = cmd_vel.angular.z;
}

// // ROS logging to display cmd_vel values received from subscriber
// void cmd_vel_logger()
// {
//   char log_lx[8];
//   char log_az[8];
//   char log_msg[32];

//   dtostrf(linear_x, 6, 2, log_lx);
//   dtostrf(angular_z, 6, 2, log_az);

//   sprintf(log_msg, "Linear_x =%s Angular_z =%s ", log_lx, log_az);
//   nh.loginfo(log_msg);

//   char rs[8];
//   char ls[8];
//   char log_speed[32];

//   dtostrf(rightSpeed, 6, 2, rs);
//   dtostrf(leftSpeed, 6, 2, ls);

//   sprintf(log_speed, "Left_speed =%s Right_speed =%s ", ls, rs);
//   nh.loginfo(log_speed);
// }

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/prymbot/cmd_vel", &cmd_vel_cb);
void setup()
{

  // // Initialize ROS node
  nh.initNode();
  nh.subscribe(cmd_vel_sub); // Initialize Subscriber
  motor_setup();
  // nh.loginfo("Motor driver ready");
}

void loop()
{
  drive();
  // cmd_vel_logger();
  nh.spinOnce();
  delay(10); // 100 Hz
}
