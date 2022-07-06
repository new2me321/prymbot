#!/usr/bin/env python3

import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rospy


class ds4_controller():
    def __init__(self):
        # Speed factor is the maximum speed allowed
        self.speed_linear = speed_linear
        self.speed_turn = speed_turn
        rospy.loginfo('Using speed_linear: [%.2f]' % self.speed_linear)
        rospy.loginfo('Using speed_turn: [%.2f]' % self.speed_turn)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.onEvent = False  # boolean for waiting for data from the subscriber
        self.button_0 = False
        self.button_1 = False
        self.button_2 = False
        self.button_3 = False
        # self.start()

    def joy_cb(self, msg):
        self.linear_x = msg.axes[1]
        self.angular_z = msg.axes[2]
        # Scale velocities
        if msg.buttons[3] == 1:
            self.button_3 = True
        if msg.buttons[1] == 1:
            self.button_1 = True

        if msg.buttons[0] == 1:
            self.button_0 = True
        if msg.buttons[2] == 1:
            self.button_2 = True

        self.onEvent = True

    def pub_velocity(self, event=None):

        cmd_msg = Twist()

        if self.onEvent:
            cmd_msg.linear.x = self.linear_x * self.speed_linear
            cmd_msg.angular.z = self.angular_z * self.speed_turn

            # publish commands
            self.pub.publish(cmd_msg)
        self.onEvent = False

    def scale_velocity(self, event=None):
        if self.button_1 == 1:
            self.speed_linear -= 0.05
            rospy.loginfo('Using speed_linear: [%.2f]' % self.speed_linear)
        if self.button_3 == 1:
            self.speed_linear += 0.05
            rospy.loginfo('Using speed_linear: [%.2f]' % self.speed_linear)
        if self.button_0 == 1:
            self.speed_turn -= 0.05
            rospy.loginfo('Using speed_turn: [%.2f]' % self.speed_turn)
        if self.button_2 == 1:
            self.speed_turn += 0.05
            rospy.loginfo('Using speed_turn: [%.2f]' % self.speed_turn)
            
        self.button_0 = False
        self.button_1 = False
        self.button_2 = False
        self.button_3 = False
        self.onEvent = False

    # def start(self):
    #     rate = rospy.Rate(1)
    #     while not rospy.is_shutdown():
    #         self.scale_velocity()
    #         rate.sleep()


if __name__ == '__main__':
    rospy.init_node('joy_teleop')
    speed_linear = 0.5
    speed_turn = 4.0

    app = ds4_controller()

    # Publish joystick velocities at a fixed rate of 10 Hz
    rospy.Timer(rospy.Duration(1.0/15.0), app.pub_velocity)
    rospy.Timer(rospy.Duration(1.0/2.0), app.scale_velocity)

    rospy.spin()
