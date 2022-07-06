#!/usr/bin/env python3.7

# Description:
# - Publishes image frames from the raspberry pi camera on the topic prymbot/image/raw
#
# Author:
# - Kwasi

import rospy
# Package responsible for converting between ROS and OpenCV Images
from cv_bridge import CvBridge
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np

from sensor_msgs.msg import CompressedImage


def publish_message():
    # Initialize ROS
    rospy.init_node('camera_publisher_compressed', anonymous=True)
    # Publisher
    pub = rospy.Publisher('/prymbot/image_raw/compressed',
                          CompressedImage, queue_size=10)

    # Go through the loop 10 times per second
    rate = rospy.Rate(10)  # 10hz

    # Create a VideoCapture object
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.rotation = 180
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # While ROS is still running.
    while not rospy.is_shutdown():

        # Capture frame-by-frame
        camera.capture(rawCapture, format="bgr", use_video_port=True)
        frame = rawCapture.array

        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])[1]).tostring()

        pub.publish(msg)

        # delete buffer
        rawCapture.truncate(0)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
