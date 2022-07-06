#!/usr/bin/env python3.7
# Description:
# - Subscriber to convert ROS Image to OpenCV
#
# Author:
# - Kwasi

import rospy  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge 
import cv2  


def callback(data):

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # Display image
    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def receive_message():
    # Initialize ROS
    rospy.init_node('camera_subscriber', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/prymbot/image_raw', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    receive_message()
