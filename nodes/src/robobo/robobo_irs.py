#!/usr/bin/env python
# -*- coding: utf-8 -*- 
# /*******************************************************************************
#  *
#  *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L. 
#  *   <http://www.mintforpeople.com>
#  *
#  *   Redistribution, modification and use of this software are permitted under
#  *   terms of the Apache 2.0 License.
#  *
#  *   This software is distributed in the hope that it will be useful,
#  *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
#  *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  *   Apache 2.0 License for more details.
#  *
#  *   You should have received a copy of the Apache 2.0 License along with    
#  *   this software. If not, see <http://www.apache.org/licenses/>.
#  *
#  ******************************************************************************/


#This program is used to publish all infrared sensor values in one topic like in the real Robobo

# ROS libraries
import rospy

# ROS messages
from sensor_msgs.msg import Range
from robobo_msgs.msg import IRs


# Class that does all the work
class ROBOBO_IRS(object):

    # Initialize ROS publishers, subscribers and variables
    def __init__(self, robobo_name='robobo'):

        self.robobo_name = robobo_name
        self.pan_position = None
        self.tilt_position = None
        self.irs_dict = dict((key, None) for key in
                             ['front_c', 'front_l', 'front_ll', 'front_r', 'front_rr', 'back_c', 'back_l', 'back_r'])
        rospy.init_node('RoboboIRS', anonymous=True)
        # ROS publishers
        # IRs
        self.irs_pub = rospy.Publisher('irs', IRs, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber('front_c', Range, self.irs_cb, 'front_c')
        rospy.Subscriber('front_l', Range, self.irs_cb, 'front_l')
        rospy.Subscriber('front_ll', Range, self.irs_cb, 'front_ll')
        rospy.Subscriber('front_r', Range, self.irs_cb, 'front_r')
        rospy.Subscriber('front_rr', Range, self.irs_cb, 'front_rr')
        rospy.Subscriber('back_c', Range, self.irs_cb, 'back_c')
        rospy.Subscriber('back_l', Range, self.irs_cb, 'back_l')
        rospy.Subscriber('back_r', Range, self.irs_cb, 'back_r')

    def irs_cb(self, value, id):
        self.irs_dict[id] = value
        if not None in self.irs_dict.values():
            self.irs_pub.publish(
                self.irs_dict['front_c'],
                self.irs_dict['front_r'],
                self.irs_dict['front_rr'],
                self.irs_dict['front_l'],
                self.irs_dict['front_ll'],
                self.irs_dict['back_c'],
                self.irs_dict['back_r'],
                self.irs_dict['back_l']
            )

    def run(self):
        rospy.spin()


def main():
    # Initializes and cleanup ROS node
    instance = ROBOBO_IRS()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down Robobo IRS"


if __name__ == '__main__':
    main()
