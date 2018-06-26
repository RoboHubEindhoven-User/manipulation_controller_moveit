#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_modern_driver.srv import *

class SinglePose(object):
    def __init__(self):

        print "============ Starting"
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('single_pose_server', anonymous=True)
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory,
                                            queue_size=20)

       
        print self.robot.get_current_state()
        #print "============ Waiting for service"
        s = rospy.Service("single_pose", PoseService , self.set_pose)
        rospy.spin()

    def set_pose(self, pose):
        #print "============ Generating plan 1"
        self.group.set_pose_target(pose.wanted_pos)
        self.plan = self.group.plan()
        print "============ Waiting while RVIZ displays plan1..."
        self.execute_plan()
        return PoseServiceResponse("Done")

    def replay(self):
        #print "============ Visualizing plan"
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(self.plan)
        self.display_trajectory_publisher.publish(self.display_trajectory)
        self.use_plan()

    def check_valid(self, wanted_pose):
        #TODO: Check if pose is reachable
        print wanted_pose

    def execute_plan(self):
        # Uncomment below line when working with a real robot
        #self.group.go(wait=True)

        # Use execute instead if you would like the robot to follow
        # the plan that has already been computed
        self.group.set_planning_time(10)
        print "============ Executing plan! If robot connected then keep emergenty stop close"
        self.group.execute(self.plan)
        

        
    
        
if __name__ == '__main__':
    x = SinglePose()
    
   