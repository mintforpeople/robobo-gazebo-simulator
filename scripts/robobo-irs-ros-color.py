#!python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk, MovePanTilt, SetLed, PlaySound
from robobo_msgs.msg import IRs, Led, Tap

rospy.init_node("robobo__demo")

# Topics
def irs_callback(data):
    global ir_frontC, ir_frontLL, ir_frontRR
    ir_frontC = data.FrontC.range
    ir_frontLL = data.FrontLL.range
    ir_frontRR = data.FrontRR.range

rospy.Subscriber("/robot/irs", IRs, irs_callback)

# Services
robobo_move_srv = rospy.ServiceProxy('robot/moveWheels', MoveWheels)
robobo_led_srv = rospy.ServiceProxy('robot/setLed', SetLed)

# Programa principal
ir_frontC = 0
ir_frontRR = 0
ir_frontLL = 0
closeIRValue = 80
mediumIRValue = 25
farIRValue = 5
speed = 10

robobo_led_srv(String('all'),String('blue'))
robobo_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))
while (ir_frontC < farIRValue) and (ir_frontRR < farIRValue) and (ir_frontLL < farIRValue):
    robobo_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))

robobo_led_srv(String('all'),String('green'))

while (ir_frontC < mediumIRValue) and (ir_frontRR < mediumIRValue) and (ir_frontLL < mediumIRValue):
    robobo_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))

robobo_led_srv(String('all'),String('magenta'))

while (ir_frontC < closeIRValue) and (ir_frontRR < closeIRValue) and (ir_frontLL < closeIRValue):
    robobo_move_srv(Int8(speed), Int8(speed), Int32(1000), Int16(0))

robobo_led_srv(String('all'),String('red'))
robobo_move_srv(Int8(0), Int8(0), Int32(1000), Int16(0)) # Stop motors (Puedo implementar funcion para que sea igual que en python)

rospy.sleep(1)
robobo_led_srv(String('all'),String('blue'))