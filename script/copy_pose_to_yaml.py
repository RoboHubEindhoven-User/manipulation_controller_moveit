#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from manipulation_controller_moveit.srv import *
import yaml

class PoseToYaml:
    def __init__(self):
        # Start ROS node
        rospy.init_node('PoseYaml', anonymous=True)
        # Defining group for coping pose.
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        print self.group.get_current_pose()
        self.copied_pose = dict()
        self.adr = '/home/user/catkin_ws/src/manipulation_controller_moveit/yaml/ros2.yaml'
        self.statemachine()

    def copy_current_pose(self):
        name = open(self.adr, 'r')
        y = yaml.load(name)
        name.close()
        print "Copy the whole file"
        self.copied_pose = y
        if self.copied_pose == None :
            self.copied_pose = dict()

        new_name = raw_input("Name of the position to add: ")
        if y.get(new_name, 0):
            print "Name '%s' already exists in the file, try other name." % new_name
        else:
            p = self.group.get_current_pose()
            print p
            self.copied_pose[new_name] = p
            
            print "Added new pose to file "
            name = open(self.adr, 'w+')
            yaml.dump(self.copied_pose, name)
        name.close()

        self.statemachine()
    
    def delete_pose(self):
        name = open(self.adr, 'r')
        y = yaml.load(name)
        name.close()

        pose_to_delete = raw_input("Name of the position to delete: ")
        try:
            y.pop(pose_to_delete)
        except:
            print("No pose with '%s' as name." % pose_to_delete)
            self.delete_pose()
        if y.get(pose_to_delete, 0):
            del y[pose_to_delete]

        print "Add new pose to file "
        name = open(self.adr, 'w+')
        yaml.dump(self.copied_pose, name)
        name.close()

        self.statemachine()
        
    def statemachine(self):
        state = raw_input("Do you want to add or delete a pose from %s, 'a' to add and 'd' to delete" % self.adr)
        if state == "a":
            self.copy_current_pose()
        elif state == "d":
            self.delete_pose()
        #else:
            #self.statemachine()
        
        
if __name__ == '__main__':
    try:
        x = PoseToYaml()
        
    except rospy.ROSInterruptException:
        pass
