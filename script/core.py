#! /usr/bin/env python

import roslib
roslib.load_manifest('manipulation_controller_moveit')
import rospy
import actionlib

from manipulation_controller_moveit.msg import MoveArmAction, MoveArmGoal

def feed(feedb):
    print feedb
def res(state ,result):
    print result

if __name__ == '__main__':
    rospy.init_node('core_faraday')
    client = actionlib.SimpleActionClient('move_arm', MoveArmAction)
    client.wait_for_server()

    goal = MoveArmGoal()
    goal.type = 'pick'
    goal.pose.position.x=-0.07926347985712519
    goal.pose.position.y=0.3748315539819346
    goal.pose.position.z=-0.06908381351545902
    goal.pose.orientation.x=-0.5145236374912043
    goal.pose.orientation.y=0.5304629420892573
    goal.pose.orientation.z=0.48502973305873515
    goal.pose.orientation.w= 0.46756887362377014


    # Fill in the goal here
    client.send_goal(goal,feedback_cb=feed, done_cb=res)
    client.wait_for_result(rospy.Duration.from_sec(60.0))
    

