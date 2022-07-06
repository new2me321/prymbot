/*
 * rosserial Differential Drive node
 * Drives motors on a callback
 *
 * By: Kwasi
 */

#include <Arduino.h>
#include <ros.h>
#include <prymbot_msgs/Encoder.h>
#include <geometry_msgs/Twist.h>
#include <motor.h>
#include <encoder.h>

// #define TICKS_PER_METER 95 // Number of ticks covered in 1 meter travel

const int LOOP_RATE = 100;           // ROS rate of 10Hz
const double WHEEL_DIAMETER = 0.065; // diameter of wheel in meters
const int COMMAND_TIMEOUT = 1000;    // 2s interval to reset encoder ticks if robot is not moving

// Initial motor params
// motor speed 0-255
double rightSpeed = 0;
double leftSpeed = 0;
bool motor_left;
bool motor_right;

float linear_x = 0;
float angular_z = 0;
float wheel_separation = 0.135; // m prev=1.26

// robot kinematics
float left_angularVel = 0.0;
float right_angularVel = 0.0;
float left_linearVel = 0.0;
float right_linearVel = 0.0;
short int right_RPM;
short int left_RPM;

unsigned long last_command = millis();
unsigned long last_time = millis();

// Instantiate the node handle
ros::NodeHandle nh;

prymbot_msgs::Encoder encoder;

// Publishers
ros::Publisher speed_pub("/prymbot/wheel", &encoder);

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
  rightSpeed = linear_x + angular_z * wheel_separation / 2;
  leftSpeed = linear_x - angular_z * wheel_separation / 2;

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
  last_command = millis();
}

void get_velocities()
{

  // get encoder ticks
  encoder.left_ticks = read_left_ticks(motor_left);
  encoder.right_ticks = read_right_ticks(motor_right);

  // get RPM
  left_RPM = read_left_RPM(motor_left);
  right_RPM = read_right_RPM(motor_right);

  // calculate angular velocity in rad/s
  left_angularVel = left_RPM * 2 * PI / 60;
  right_angularVel = right_RPM * 2 * PI / 60;

  // calculate linear velocity in m/s
  left_linearVel = left_angularVel * WHEEL_DIAMETER / 2;
  right_linearVel = right_angularVel * WHEEL_DIAMETER / 2;

  encoder.left_speed = left_linearVel;
  encoder.right_speed = right_linearVel;
  encoder.header.stamp = nh.now();

  //  publish
  speed_pub.publish(&encoder);
}

// Subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_cb);

void setup()
{

  nh.getHardware()->setBaud(57600); // set Arduino baudrate

  // Initialize ROS node
  nh.initNode();

  // Initialize Publishers & Subscribers
  nh.subscribe(cmd_vel_sub);
  nh.advertise(speed_pub);

  // initialiaze base
  motor_setup();
  encoder_setup();
  // reset encoder ticks
  encoder.left_ticks = 0;
  encoder.right_ticks = 0;
}

void loop()
{
  // If 100ms have passed drive motors and read encoder RPM
  if ((millis() - last_time) >= LOOP_RATE)
  {
    // reset velocities when last command received is greater than timeout
    if ((millis() - last_command) >= COMMAND_TIMEOUT)
    {
      linear_x = 0;
      angular_z = 0;

      // reset encoder ticks
      encoder.left_ticks = 0;
      encoder.right_ticks = 0;
    }

    // Drive robot
    drive();

    // get volocities
    get_velocities();

    last_time = millis(); // reset timer
  }

  // cmd_vel_logger();
  nh.spinOnce();
}
