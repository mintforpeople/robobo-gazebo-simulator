#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import *
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

image = None


def imageCallback(data):
    print("New image")
    nparr = np.frombuffer(data.data, np.uint8)

    global image
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)


if __name__ == '__main__':
    cv2.namedWindow('image')
    rospy.init_node('test_consumer', anonymous=True)
    imageSub = rospy.Subscriber(
        'camera/image/compressed', CompressedImage, imageCallback)
    while not rospy.is_shutdown():
        if image is not None:
            cv2.imshow('image', image)
            cv2.waitKey(1)

    # print("Error reported: %d"%call_fun(lspeed, rspeed, time))
