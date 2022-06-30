#!/usr/bin/env python3

import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rospy


class ds4_controller():
    def __init__(self):
        rospy.init_node('joy_teleop')

        # Speed factor is the maximum speed allowed
        self.speed_linear = 1.0 
        self.speed_turn = 1.5 
        rospy.loginfo('Using speed_linear: [%.1f]' % self.speed_linear)
        rospy.loginfo('Using speed_turn: [%.1f]' % self.speed_turn)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_cb)

    def joy_cb(self, msg):
        cmd_msg = Twist()
        if msg.buttons[5] == 1: # R1 button is pressed
            cmd_msg.linear.x = self.speed_linear * msg.axes[1]
            cmd_msg.angular.z = self.speed_turn * msg.axes[2]
        else:
            cmd_msg.linear.x = 0
            cmd_msg.angular.z =0
            
        # publish commands 
        self.pub.publish(cmd_msg)

if __name__ == '__main__':
    app = ds4_controller()
    rospy.spin()
