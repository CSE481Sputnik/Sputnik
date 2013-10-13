#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
import rospy
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

class GripperClient():

    def __init__(self):
        #rospy.init_node('simplegui_gripper_node', anonymous=True)

        name_space_r = '/r_gripper_controller/gripper_action'
        self.r_gripper_client = SimpleActionClient(name_space_r, GripperCommandAction)

        name_space_l = '/l_gripper_controller/gripper_action'
        self.l_gripper_client = SimpleActionClient(name_space_l, GripperCommandAction)

        self.r_gripper_client.wait_for_server()
        self.l_gripper_client.wait_for_server()

    def command(self, left, close):
        gripper_client = self.l_gripper_client
        if not left:
            gripper_client = self.r_gripper_client

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 30.0
        if close:
            gripper_goal.command.position = 0.0
        else:
            gripper_goal.command.position = 1.0

        gripper_client.send_goal(gripper_goal)
        gripper_client.wait_for_result(rospy.Duration(10.0))
        if (gripper_client.get_state() != GoalStatus.SUCCEEDED):
	    rospy.logwarn('Gripper action unsuccessful.')
