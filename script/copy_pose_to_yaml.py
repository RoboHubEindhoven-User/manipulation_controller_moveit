#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_modern_driver.srv import *
import yaml

class PoseToYaml:
    def __init__(self):
        # Start ROS node
        rospy.init_node('PoseYaml', anonymous=True)
        # Defining group for coping pose.
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        print self.group.get_current_pose()
        self.copied_pose = dict()
        adr = '/home/user/catkin_ws/src/ur_modern_driver/yaml/ros2.yaml'
        self.copy_current_pose(adr)

    def copy_current_pose(self, fileaddr):
        name = open(fileaddr, 'r')
        y = yaml.load(name)
        name.close()
        print "Copy the whole file"
        self.copied_pose = y
        new_name = raw_input("Name of the position: ")
        
        if self.copied_pose == None :
            self.copied_pose = dict()

        p = self.group.get_current_pose()
        print p
        self.copied_pose[new_name] = p
        
        print "Add new pose to file "
        name = open(fileaddr, 'w+')
        yaml.dump(self.copied_pose, name)
        name.close()
    
    def delete_pose(self, fileaddr):
        name = open(fileaddr, 'r')
        y = yaml.load(name)
        name.close()
        
        
if __name__ == '__main__':
    try:
        x = PoseToYaml()
        
    except rospy.ROSInterruptException:
        pass