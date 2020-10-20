#!/usr/bin/env python

import rospy
import sys
import threading
from std_msgs.msg import *
from robobo_msgs.msg import *
from robobo_msgs.srv import *
from geometry_msgs.msg import Accel, Quaternion
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

image = None
lspeed = 20
rspeed = -20
time = 3000
blockid = 1
panPos = 130
panSpeed = 40
panUnlockId = 2
tiltPos = 50
tiltSpeed = 40
tiltUnlockId = 3


def imageCallback(data):
    # print("New image")
    nparr = np.frombuffer(data.data, np.uint8)

    global image
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)


def accelCallback(data):
    print("Accel:", data)


def orientationCallback(data):
    print("Orientation:", data)


def commandTheShip():
    rospy.sleep(1)
    movePanTiltPub.publish(MovePanTiltCommand(Int16(panPos), Int8(panSpeed), Int16(
        panUnlockId), Int16(tiltPos), Int8(tiltSpeed), Int16(tiltUnlockId)))

    moveWheelsPub.publish(MoveWheelsCommand(
        Int8(lspeed), Int8(rspeed), Int32(time), Int16(blockid)))

    rospy.sleep(4)
    resp1 = moveWheelsServ(Int8(-lspeed), Int8(
        -rspeed), Int32(time), Int16(blockid))
    resp2 = movePanTiltServ(Int16(360-panPos), Int8(panSpeed), Int16(
        panUnlockId), Int16(130-tiltPos), Int8(tiltSpeed), Int16(tiltUnlockId))


if __name__ == '__main__':
    global moveWheelsServ, movePanTiltServ, imageSub, accelSub, orientationSub, moveWheelsPub, movePanTiltPub, funThread
    cv2.namedWindow('image')
    rospy.init_node('test_consumer', anonymous=True)
    rospy.wait_for_service('moveWheels')
    rospy.wait_for_service('movePanTilt')

    # Services
    moveWheelsServ = rospy.ServiceProxy('moveWheels', MoveWheels)
    movePanTiltServ = rospy.ServiceProxy('movePanTilt', MovePanTilt)

    # Subscribers
    imageSub = rospy.Subscriber(
        'camera/image/compressed', CompressedImage, imageCallback)

    accelSub = rospy.Subscriber(
        'accel', Accel, accelCallback)
    orientationSub = rospy.Subscriber(
        'orientation', Quaternion, orientationCallback)

    # Publishers
    moveWheelsPub = rospy.Publisher(
        'moveWheels', MoveWheelsCommand, queue_size=1)

    movePanTiltPub = rospy.Publisher('movePanTilt',
                                     MovePanTiltCommand, queue_size=1)

    funThread = threading.Thread(target=commandTheShip)
    funThread.start()
    while not rospy.is_shutdown():
        if image is not None:
            cv2.imshow('image', image)
            cv2.waitKey(1)

    # print("Error reported: %d"%call_fun(lspeed, rspeed, time))
