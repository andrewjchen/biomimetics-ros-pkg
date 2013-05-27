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
import geometry_msgs.msg
import time

from imageproc_py.stream.imageproc_uart_stream import *
from imageproc_py.stream.uart_robot_stream import *

def clamp(value, minVal, maxVal):
    return max(minVal, min(maxVal, value))

def handle_command(msg):
    # print 'desired linear vel ' + str(msg.linear.x)
    # print 'desired angular rate ' + str(msg.angular.z)

    left_throttle = linearGain * msg.linear.x - angularGain * msg.angular.z
    right_throttle = linearGain * msg.linear.x + angularGain * msg.angular.z
    left_throttle = -left_throttle if invertLeft else left_throttle
    right_throttle = -right_throttle if invertRight else right_throttle
    left_throttle = clamp(left_throttle, minThrottle, maxThrottle)
    right_throttle = clamp(right_throttle, minThrottle, maxThrottle)

    print 'setting thrust left=%d  right=%d' %(left_throttle, right_throttle)
    robot.set_thrust_open_loop(left_throttle, right_throttle)

def main():
    print "nodetest.py running..."

    rospy.init_node('imageproc_node')
    rospy.Subscriber('cmd_vel',
                     geometry_msgs.msg.Twist,
                     handle_command)

    global invertLeft, invertRight, minThrottle, maxThrottle, linearGain, angularGain
    device = rospy.get_param('~device', '/dev/ttyUSB0')
    invertLeft = rospy.get_param('~invertLeft', False)
    invertRight = rospy.get_param('~invertRight', False)
    minThrottle = rospy.get_param('~minThrottle', -500)
    maxThrottle = rospy.get_param('~maxThrottle', 500)
    linearGain = rospy.get_param('~linearGain', 2400)
    angularGain = rospy.get_param('~angularGain', 2400)

    print "invertLeft=" + str(invertLeft)
    print "invertRight=" + str(invertRight)
    print "minThrottle=" + str(minThrottle)
    print "maxThrottle=" + str(invertLeft)
    print "linearGain=" + str(linearGain)
    print "angularGain=" + str(angularGain)

    print "device=" + str(device)

    global robot
    ipu = ImageprocUARTStream(port=device)
    robot = UARTRobotStream(ipu)

    rospy.spin()

if __name__ == '__main__':
    main()
