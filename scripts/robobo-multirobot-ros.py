'''This work has been funded by rosin.org (contract agreement 732287) 
 through EP project "Robobo AI"'''

#!/usr/bin/env python

'''CHALLENGE 
Create a program that makes Robobo 1 move forward until it detects an obstacle:
Through the front infrared sensors, Robobo 1 will detect the obstacle and it will stop to avoid the collision. 
Then, it should start Robobo 2 operation.
Robobo 2 will move forward until it detects another obstacle.
Through the front infrared sensors, Robobo 2 will detect the obstacle and it will stop to avoid the collision.'''

import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, MovePanTilt
from robobo_msgs.msg import IRs

rospy.init_node("robobo__multirobot")

# Topics
def irs1_callback(data):
    global ir1_frontC, ir1_frontLL, ir1_frontRR
    ir1_frontC = data.FrontC.range
    ir1_frontLL = data.FrontLL.range
    ir1_frontRR = data.FrontRR.range

def irs2_callback(data):
    global ir2_frontC, ir2_frontLL, ir2_frontRR
    ir2_frontC = data.FrontC.range
    ir2_frontLL = data.FrontLL.range
    ir2_frontRR = data.FrontRR.range


def callback_robobo2(data):
    if data.data == True:
        robobo2_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
        while (ir2_frontC < closeIRValue) and (ir2_frontRR < closeIRValue) and (ir2_frontLL < closeIRValue):
            robobo2_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
        robobo2_move_srv(Int8(0), Int8(0), Int32(1000), Int16(0)) # Stop motors
        rospy.sleep(1)
        pub_robobo1.publish(Bool(True))
        

rospy.Subscriber("/robot/robobo1/irs", IRs, irs1_callback)
rospy.Subscriber("/robot/robobo2/irs", IRs, irs2_callback)

pub_robobo2 = rospy.Publisher("/start/robobo2", Bool, queue_size=0)
rospy.Subscriber("/start/robobo2", Bool, callback_robobo2)

# Services
robobo1_move_srv = rospy.ServiceProxy('robot/robobo1/moveWheels', MoveWheels)
robobo2_move_srv = rospy.ServiceProxy('robot/robobo2/moveWheels', MoveWheels)


# Programa principal
ir1_frontC = 0
ir1_frontRR = 0
ir1_frontLL = 0
ir2_frontC = 0
ir2_frontRR = 0
ir2_frontLL = 0
closeIRValue = 60
speed = 20


robobo1_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
while (ir1_frontC < closeIRValue) and (ir1_frontRR < closeIRValue) and (ir1_frontLL < closeIRValue):
    robobo1_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))

robobo1_move_srv(Int8(0), Int8(0), Int32(1000), Int16(0)) # Stop motors
print('Come on Robobo 2!'))
rospy.sleep(1)
pub_robobo2.publish(Bool(True))
rospy.spin()
