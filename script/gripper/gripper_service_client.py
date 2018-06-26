#! /usr/bin/env python

import rospy
from manipulation_controller_moveit.srv import *

def gripper_client(size_to_open):
    rospy.init_node('gripper_service_client')
    rospy.wait_for_service('gripper_control')
    try:
        gripper = rospy.ServiceProxy('gripper_control', Gripperv6)
        gripper_control = gripper(size_to_open)
        print "Send state: ", gripper_control.result
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e


if __name__ == "__main__":

    print 'Client node ready'
    user_input = raw_input('Please give in the size to open (0mm - 185mm). ')
    command = int(120/185.0 * int(user_input))
    command = 120 - command
    size_to_open =  command & 0xFF
    print "Requesting %s degree" %size_to_open
    gripper_client(size_to_open)
