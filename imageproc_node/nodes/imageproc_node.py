#!/usr/bin/env python

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
