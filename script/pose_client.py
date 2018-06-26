#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from manipulation_controller_moveit.srv import *
import yaml
import actionlib
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import roslib
from manipulation_controller_moveit.msg import *
from manipulation_controller_moveit.srv import *

class ArmStateMachine:
    def __init__(self):
        # Start ROS node
        rospy.init_node('pose_client', anonymous=True)
        # Defining group for coping pose.
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        #print self.group.get_current_pose()
        self.copied_pose = {}
        self.addr = "/home/user/catkin_ws/src/manipulation_controller_moveit/yaml/ros2.yaml"
        # Publishers for the gripper:
        self.gripper_pub = rospy.Publisher('valueGripper', Float64, queue_size=10)
        self.led_pub = rospy.Publisher('valueLED', Bool, queue_size=10)

        #Action for the arm movement from the core:
        self.server = actionlib.SimpleActionServer('move_arm', MoveArmAction, self.execute, False)
        self.feedback = MoveArmFeedback()
        self.result = MoveArmResult()
        self.server.start()

        self.prepick_height = 0.05

        self.statemachines()
    
    # pose_name needs to be a raw_input() if manual
    def get_pose_from_yaml(self, pose_name):
        name = open(self.addr, 'r')
        y = yaml.load(name)
        name.close()
        poses = y
        wanted_pose = poses.get(pose_name)
        #print pose_name
        return wanted_pose
        
    def gripper_client(self, size_to_open):
        
        rospy.wait_for_service('gripper_control')
        try:
            gripper = rospy.ServiceProxy('gripper_control', GripperControl)
            gripper_control = gripper(size_to_open)
            print "Send state: ", gripper_control.completed

        except rospy.ServiceException, e:
            print "Service call failed: %s" %e

    def send_pose(self, pose):
        #TODO: Something goes wrong here, Alot of times you print service call failed:....
        rospy.wait_for_service('single_pose')
        try:
            pose_service = rospy.ServiceProxy('single_pose', PoseService)
            status = pose_service(pose)
            print "Pose send to service"
            print "feedback of service:"
            print status.status
            return status.status
            
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e 


    def preform_pose(self, name):
        self.feedback.status = name
        self.server.publish_feedback(self.feedback)
        self.send_pose(self.get_pose_from_yaml(name).pose)

    def pre_pick(self, pose):
        self.feedback.status = "Going to Prepick"
        self.server.publish_feedback(self.feedback)
        pre_pick_pose = pose
        pre_pick_pose.position.z = pose.position.z + self.prepick_height
        self.send_pose(pre_pick_pose)

    def pre_place(self, pose):
        self.feedback.status = "Going to Preplace"
        self.server.publish_feedback(self.feedback)
        pre_place_pose = pose
        pre_place_pose.position.z = pose.position.z + self.prepick_height
        self.send_pose(pre_place_pose)

    def place(self, pose):
        self.feedback.status = "Going to Place"
        self.server.publish_feedback(self.feedback)
        self.send_pose(pose)

    def pick(self, pose):
        self.feedback.status = "Going to Pick"
        self.server.publish_feedback(self.feedback)
        self.send_pose(pose)
    
    def cam_pose(self):
        self.feedback.status = "Going to Cam_pos"
        self.server.publish_feedback(self.feedback)
        x.preform_pose('cam')

    def statemachines(self):
        types = raw_input("Do you want to go to 'pick', 'cam' or 'home'")
        if types == 'pick' :
            
            #x.preform_pose('cam')
            #self.led_pub.publish(True)
            #rospy.sleep(5.0)
            #self.led_pub.publish(False)
            #rospy.sleep(1.0)
            
            self.pre_pick(self.get_pose_from_yaml('pick').pose)
            #self.gripper_client(18.5)
            rospy.sleep(3.0)
            
            self.pick(self.get_pose_from_yaml('pick').pose)
            #self.gripper_client(3.5)
            rospy.sleep(3.0)
            
            self.pre_pick(self.get_pose_from_yaml('pick').pose)
            #x.preform_pose('cam')
            rospy.sleep(5.0)
            self.pre_place(self.get_pose_from_yaml('place').pose)
            self.place(self.get_pose_from_yaml('place').pose)
            #self.gripper_client(18.5)
            rospy.sleep(3.0)
            self.pre_place(self.get_pose_from_yaml('place').pose)
            self.preform_pose('home')
            
            
            
        elif types == 'cam':
            self.preform_pose('cam')
            self.led_pub.publish(True)
            rospy.sleep(5.0)
            self.led_pub.publish(False)
            
            
        elif types == 'home':
            x.preform_pose('home')

        else: 
            print "No states will be preformed because input was not correct"

        


    def execute(self, goal):
        # Do lots of awesome groundbreaking robot stuff 
        print goal
        self.thepickpose = goal.pose
        self.statemachines(goal.type)
        
        self.result.end_status = 'done'
        self.server.set_succeeded(self.result)
        print "Statemachine done"


if __name__ == '__main__':
    try:
        x = ArmStateMachine()
             
        
    except rospy.ROSInterruptException:
        pass
