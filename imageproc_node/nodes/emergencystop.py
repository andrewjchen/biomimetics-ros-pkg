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

from geometry_msgs.msg import *
from std_msgs.msg import *

'''
takes command velocities from /safeCmdVel, and if periodically receiving heartbeat signal, relays it to robot
'''

POLL_TIME = .25

ZERO_TWIST = Twist()
ZERO_TWIST.linear.x = 0
ZERO_TWIST.linear.y = 0
ZERO_TWIST.linear.z = 0
ZERO_TWIST.angular.x = 0
ZERO_TWIST.angular.y = 0
ZERO_TWIST.angular.z = 0

def main():
    global pub
    rospy.loginfo("starting emergency stop node")
    rospy.init_node('emergency_stop')
    pub = rospy.Publisher("cmd_vel", Twist)

    global lastMessageReceivedTime
    lastMessageReceivedTime = 0

    rospy.Subscriber("heartbeat", UInt64, updateTime)
    rospy.Subscriber("safe_cmd_vel", Twist, updateVelocity)


    while not rospy.is_shutdown():
        if time.time() - lastMessageReceivedTime > POLL_TIME:
            stopRobot()
        rospy.sleep(POLL_TIME)

def updateTime(msg):
    global lastMessageReceivedTime
    lastMessageReceivedTime = time.time()

def updateVelocity(safeCmdVel):
    global lastMessageReceivedTime
    #if last message received was too long ago
    if time.time() - lastMessageReceivedTime > POLL_TIME:
        stopRobot() #stop robot
    else: #last message received was recent enough
        pub.publish(safeCmdVel) #robot drives

def stopRobot():
    global pub
    rospy.loginfo("Stopping!")
    pub.publish(ZERO_TWIST)

main()
