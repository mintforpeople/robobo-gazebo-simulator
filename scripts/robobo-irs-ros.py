'''This work has been funded by rosin.org (contract agreement 732287) 
 through EP project "Robobo AI"'''

#!/usr/bin/env python

'''CHALLENGE 
Create a program that makes Robobo move forward until it detects an obstacle:
Through the front infrared sensors, Robobo will detect the obstacle and it will stop to avoid the collision. 
Then, it should move his head back and then move backwards for a few seconds.
Finally, make Robobo return its head to its normal position.'''

import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, MovePanTilt
from robobo_msgs.msg import IRs

rospy.init_node("robobo__demo")

# Topics
def irs_callback(data):
    global ir_backC, ir_backL, ir_backR, ir_frontC, ir_frontL, ir_frontLL, ir_frontR, ir_frontRR
    ir_backC = data.BackC.range
    ir_backL = data.BackL.range
    ir_backR = data.BackR.range
    ir_frontC = data.FrontC.range
    ir_frontL = data.FrontL.range
    ir_frontLL = data.FrontLL.range
    ir_frontR = data.FrontR.range
    ir_frontRR = data.FrontRR.range

rospy.Subscriber("robot/robobo/irs", IRs, irs_callback)

# Services
robobo_move_srv = rospy.ServiceProxy('robot/robobo/moveWheels', MoveWheels)
robobo_pantilt_srv = rospy.ServiceProxy('robot/robobo/movePanTilt', MovePanTilt)

# Programa principal
ir_frontC = 0
ir_frontRR = 0
ir_frontR = 0
ir_frontLL = 0
ir_frontL = 0
ir_backC = 0
ir_backR = 0
ir_backL = 0
closeIRValue = 60
speed = 20

robobo_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
while (ir_frontC < closeIRValue) and (ir_frontRR < closeIRValue) and (ir_frontLL < closeIRValue):
    robobo_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))

robobo_move_srv(Int8(0), Int8(0), Int32(1000), Int16(0)) # Stop motors 

rospy.sleep(2)

robobo_pantilt_srv(Int16(0), Int8(0), Int16(1), Int16(50), Int8(15), Int16(1))

rospy.sleep(2)

robobo_move_srv(Int8(-speed), Int8(-speed), Int32(2000), Int16(0))

rospy.sleep (2)

robobo_pantilt_srv(Int16(0), Int8(0), Int16(1), Int16(75), Int8(15), Int16(1))
