#!/usr/bin/env python3

from sensor_msgs.msg import CompressedImage
import rospy
import cv2
import numpy as np


def callback(msg):

    # Used to convert between ROS and OpenCV images
    #### direct conversion to CV2 ####
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Display image
    cv2.imshow("camera", image_np)

    cv2.waitKey(1)


def receive_message():
    # Initialize ROS
    rospy.init_node('camera_subscriber', anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber('/image_raw/compressed', CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    receive_message()

