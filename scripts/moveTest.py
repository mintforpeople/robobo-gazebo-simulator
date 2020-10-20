#!/usr/bin/env python

import rospy
import sys
from robobo_msgs.srv import *
from robobo_msgs.msg import MoveWheelsCommand
from std_msgs.msg import *

TEST = "TOPIC"


def call_service(lspeed, rspeed, time, blockid=0):
    rospy.wait_for_service('moveWheels')
    #rate = rospy.Rate(10) # 10hz
    try:
        moveWheels = rospy.ServiceProxy('moveWheels', MoveWheels)
        resp1 = moveWheels(Int8(lspeed), Int8(rspeed), Int32(time), Int16(blockid))
        return resp1.error.data
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def call_topic(lspeed, rspeed, time, blockid=0):
    pub = rospy.Publisher('moveWheels', MoveWheelsCommand, queue_size=10)
    rospy.sleep(1)
    pub.publish(MoveWheelsCommand(Int8(lspeed), Int8(rspeed), Int32(time), Int16(blockid)))

if __name__ == '__main__':
    rospy.init_node('test_consumer',anonymous=True)
    if TEST is "TOPIC":
        call_fun = call_topic
    else:
        call_fun = call_service

    lspeed = rospy.get_param("~lspeed",0)
    rspeed = rospy.get_param("~rspeed",0)
    time   = rospy.get_param("~time",0)
    print ("Got args: %d, %d, %d"%(lspeed, rspeed, time))
    call_fun(lspeed, rspeed, time)
    # print("Error reported: %d"%call_fun(lspeed, rspeed, time)) 
