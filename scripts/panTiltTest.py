#!/usr/bin/env python

import rospy
import sys
from robobo_msgs.srv import *
from robobo_msgs.msg import MovePanTiltCommand
from std_msgs.msg import *

TEST = "TOPIC"


def call_service(panPos, panSpeed, panUnlockId, tiltPos, tiltSpeed, tiltUnlockId):
    rospy.wait_for_service('movePanTilt')
    # rate = rospy.Rate(10) # 10hz
    try:
        movePanTilt = rospy.ServiceProxy('movePanTilt', MovePanTilt)
        resp1 = movePanTilt(Int16(panPos), Int8(panSpeed), Int16(
            panUnlockId), Int16(tiltPos), Int8(tiltSpeed), Int16(tiltUnlockId))
        return resp1.error.data
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def call_topic(panPos, panSpeed, panUnlockId, tiltPos, tiltSpeed, tiltUnlockId):
    pub = rospy.Publisher('movePanTilt',
                          MovePanTiltCommand, queue_size=10)
    rospy.sleep(1)
    pub.publish(MovePanTiltCommand(Int16(panPos), Int8(panSpeed), Int16(
        panUnlockId), Int16(tiltPos), Int8(tiltSpeed), Int16(tiltUnlockId)))


if __name__ == '__main__':
    rospy.init_node('test_consumer', anonymous=True)
    if TEST is "TOPIC":
        call_fun = call_topic
    else:
        call_fun = call_service

    panPos = rospy.get_param("~panPos", 0)
    panSpeed = rospy.get_param("~panSpeed", 0)
    panUnlockId = rospy.get_param("~panUnlockId", 0)
    tiltPos = rospy.get_param("~tiltPos", 0)
    tiltSpeed = rospy.get_param("~tiltSpeed", 0)
    tiltUnlockId = rospy.get_param("~tiltUnlockId", 0)
    print("Got args: %d, %d, %d, %d, %d, %d" %
          (panPos, panSpeed, panUnlockId, tiltPos, tiltSpeed, tiltUnlockId))
    call_fun(panPos, panSpeed, panUnlockId, tiltPos, tiltSpeed, tiltUnlockId)
