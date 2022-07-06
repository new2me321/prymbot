/*
 * Odometry publisher node
 * subscribes to /prymbot/wheel
 * publishes to /odom
 *
 * This CPP script is a ported version of Jon Stephan diff_tf.py
 *
 * Author: Kwasi
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <prymbot_msgs/Encoder.h>

// Robot constants
int ticks_meter = 95;
float base_width = 0.135; // m

// params
double encoder_low_wrap;
double encoder_high_wrap;
double prev_lencoder = 0;
double prev_rencoder = 0;
double encoder_max = 32768;
double encoder_min = -32768;
int lmult = 0;
int rmult = 0;
double left = 0;
double right = 0;
int old_enc_left = 0;
int old_enc_right = 0;
float d_left;
float d_right;

float new_x = 0;
float new_y = 0;
float new_th = 0;

float dx;
float dr;

ros::Time t_next, t_delta;
ros::Time current_time, last_time;

void wheelCallback(prymbot_msgs::Encoder msg)
{
    // # left
    volatile int enc_left = msg.left_ticks;

    if (enc_left < encoder_low_wrap && prev_lencoder > encoder_high_wrap)
    {
        lmult = lmult + 1;
    }

    if (enc_left > encoder_high_wrap and prev_lencoder < encoder_low_wrap)
    {
        lmult = lmult - 1;
    }

    left = 1.0 * (enc_left + lmult *
                                 (encoder_max - encoder_min));
    prev_lencoder = enc_left;

    // # right
    volatile int enc_right = msg.right_ticks;
    if (enc_right < encoder_low_wrap and prev_rencoder > encoder_high_wrap)
    {
        rmult = rmult + 1;
    }
    if (enc_right > encoder_high_wrap and prev_rencoder < encoder_low_wrap)
    {
        rmult = rmult - 1;
    }

    right = 1.0 * (enc_right + rmult *
                                   (encoder_max - encoder_min));
    prev_rencoder = enc_right;
}

void update(ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster)
{
    geometry_msgs::TransformStamped odom_trans;
    current_time = ros::Time::now();
    if (current_time > t_next)
    {
        double elapsed = (current_time - last_time).toSec();
        last_time = current_time;

        // # calculate odometry
        if (old_enc_left == 0)
        {
            d_left = 0;
            d_right = 0;
        }
        else
        {
            d_left = (left - old_enc_left) / ticks_meter;
            d_right = (right - old_enc_right) / ticks_meter;
        }

        old_enc_left = left;
        old_enc_right = right;

        // # distance traveled is the average of the two wheels
        float d = (d_left + d_right) / 2;
        // # this approximation works (in radians) for small angles
        float th = (d_right - d_left) / base_width;
        // # calculate velocities
        dx = d / elapsed;
        dr = th / elapsed;

        if (d != 0)
        {
            // # calculate distance traveled in x and y
            float x = cos(th) * d;
            float y = -sin(th) * d;
            // # calculate the final position of the robot
            new_x = new_x + (cos(new_th) * x - sin(new_th) * y);
            new_y = new_y + (sin(new_th) * x + cos(new_th) * y);
        }

        if (th != 0)
        {
            new_th = new_th + th;
        }
    }

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(new_th);

    // first, we'll publish the transform over tf
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = new_x;
    odom_trans.transform.translation.y = new_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // set the position
    odom.pose.pose.position.x = new_x;
    odom.pose.pose.position.y = new_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dr;

    // publish the message
    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber wheel_sub = n.subscribe<prymbot_msgs::Encoder>("/prymbot/wheel", 1, wheelCallback);
    tf::TransformBroadcaster odom_broadcaster;

    encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
    encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min;

    double rate = 100;
    ros::Duration t_delta(1 / rate);
    t_next = ros::Time::now() + t_delta;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    std::cout << t_delta << std::endl;
    std::cout << t_next << std::endl;

    ros::Rate r(rate);
    ROS_INFO("Odometry publisher started!");
    while (n.ok())
    {
        ros::spinOnce(); // check for incoming messages
        update(odom_pub, odom_broadcaster);
        r.sleep();
    }
    return 0;
}