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
import sys
import math
import random
import time

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

'''
provides a .05 second heartbeat when the robot is supposed to be operating
can be enabled or disabled with boolean messages at /robotEnabled
'''

BUTTON_NUM = 5
POLL_TIME = 0.05

def main():
    global pub
    global robotEnabled
    robotEnabled = False
    rospy.loginfo("starting heartbeat")
    rospy.init_node('heartbeat')
    pub = rospy.Publisher("heartbeat", UInt64)
    rospy.Subscriber("/joy", Joy, joystickChanged)

    countNumber = 0

    while (not rospy.is_shutdown()):
        if robotEnabled:
            rospy.loginfo("Enabled! count=" + str(countNumber))
            pub.publish(countNumber)
            countNumber = countNumber + 1
        rospy.loginfo("Disabled! count=" + str(countNumber))
        rospy.sleep(POLL_TIME)

def joystickChanged(data):
    global robotEnabled
    if(data.buttons[BUTTON_NUM]):
        robotEnabled = True
    else:
        robotEnabled = False

if __name__ == '__main__':
    main()
