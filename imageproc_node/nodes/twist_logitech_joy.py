#!/usr/bin/env python

# Copyright (c) 2010-2013, Regents of the University of California
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# - Neither the name of the University of California, Berkeley nor the names
# of its contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revisions:
# Andrew J. Chen    2013-05-27  Initial release

import rospy
import serial

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

'''
twist_logitech_joy.py sends Twist commands from a joystick.
'''

def main():
    global pub
    rospy.loginfo("starting Twist Publishing via Joystick...")
    rospy.init_node('twist_logitech_joy')

    # TODO(andrew.chen) configureable topic
    pub = rospy.Publisher('safe_cmd_vel', Twist)
    rospy.Subscriber("joy", Joy, joystickChanged)
    rospy.spin()

def joystickChanged(data):
    #print str(data.axes[1]) + ","+ str(data.axes[4])

    #TODO(andrew.chen) configurable axes, constants?
    msg = Twist()
    msg.linear.x =  data.axes[1] + data.axes[4]
    msg.angular.z = data.axes[4] - data.axes[1]
    pub.publish(msg)

if __name__ == "__main__":
    main()
