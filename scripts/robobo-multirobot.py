#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8, Int16, Int32, Bool
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk, MovePanTilt, SetLed, PlaySound
from robobo_msgs.msg import IRs, Led, Tap

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
        robobo2_led_srv(String('all'),String('green'))
        robobo2_talk_srv(String('Alla voy!'))
        robobo2_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
        while (ir2_frontC < closeIRValue) and (ir2_frontRR < closeIRValue) and (ir2_frontLL < closeIRValue):
            robobo2_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
        robobo2_move_srv(Int8(0), Int8(0), Int32(1000), Int16(0)) # Stop motors (Puedo implementar funcion para que sea igual que en python)
        robobo2_talk_srv(String('Uy, casi choco!'))
        robobo2_talk_srv(String('Te toca Robobo 1!'))
        rospy.sleep(1)
        robobo2_led_srv(String('all'),String('red'))
        pub_robobo1.publish(Bool(True))


def callback_robobo1(data):
    if data.data == True:
        robobo1_led_srv(String('all'),String('green'))
        robobo1_pantilt_srv(Int16(), Int8(0), Int16(1), Int16(50), Int8(15), Int16(1))
        robobo1_talk_srv(String('Alla voy!'))
        robobo1_move_srv(Int8(-speed), Int8(-speed), Int32(2000), Int16(0))
        rospy.sleep(2)
        robobo1_led_srv(String('all'),String('red'))
        rospy.signal_shutdown("End of program")

rospy.Subscriber("/robot/robobo1/irs", IRs, irs1_callback)
rospy.Subscriber("/robot/robobo2/irs", IRs, irs2_callback)

pub_robobo1 = rospy.Publisher("/start/robobo1", Bool, queue_size=0)
rospy.Subscriber("/start/robobo1", Bool, callback_robobo1)
pub_robobo2 = rospy.Publisher("/start/robobo2", Bool, queue_size=0)
rospy.Subscriber("/start/robobo2", Bool, callback_robobo2)

# Services
robobo1_move_srv = rospy.ServiceProxy('robot/robobo1/moveWheels', MoveWheels)
robobo1_pantilt_srv = rospy.ServiceProxy('robot/robobo1/movePanTilt', MovePanTilt)
robobo1_led_srv = rospy.ServiceProxy('robot/robobo1/setLed', SetLed)
robobo1_talk_srv = rospy.ServiceProxy('robot/robobo1/talk', Talk)

robobo2_move_srv = rospy.ServiceProxy('robot/robobo2/moveWheels', MoveWheels)
robobo2_led_srv = rospy.ServiceProxy('robot/robobo2/setLed', SetLed)
robobo2_talk_srv = rospy.ServiceProxy('robot/robobo2/talk', Talk)


# Programa principal
ir1_frontC = 0
ir1_frontRR = 0
ir1_frontLL = 0
ir2_frontC = 0
ir2_frontRR = 0
ir2_frontLL = 0
closeIRValue = 60
speed = 20


robobo1_led_srv(String('all'),String('green'))
robobo2_led_srv(String('all'),String('red'))
robobo1_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
while (ir1_frontC < closeIRValue) and (ir1_frontRR < closeIRValue) and (ir1_frontLL < closeIRValue):
    robobo1_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))

robobo1_move_srv(Int8(0), Int8(0), Int32(1000), Int16(0)) # Stop motors (Puedo implementar funcion para que sea igual que en python)
robobo1_talk_srv(String('Adelante Robobo 2!'))
robobo1_led_srv(String('all'),String('red'))
rospy.sleep(1)
pub_robobo2.publish(Bool(True))
rospy.spin()
