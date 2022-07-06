#!/usr/bin/env python3

import rospy
from prymbot_msgs.msg import Encoder


class Odom():
    def __init__(self):
        self.distance_left = 0
        self.distance_right = 0
        self.TICKS_PER_METER = 95
        self.WHEEL_DIAMETER = 0.065
        self.PI = 3.141592654

        rospy.Subscriber("prymbot/wheel", Encoder, self.encoderCb)

    def encoderCb(self, msg):
        self.distance_left = (
            msg.left_ticks / self.TICKS_PER_METER) * self.PI * self.WHEEL_DIAMETER
        self.distance_right = (
            msg.right_ticks / self.TICKS_PER_METER) * self.PI * self.WHEEL_DIAMETER

        rospy.loginfo("distance_left=%.2f, distance_right=%.2f",
                      self.distance_left, self.distance_right)


def main():
    rospy.init_node('distance_calculation')
    rate = rospy.Rate(20)

    odom = Odom()
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
