#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('geometry_msgs')

import rospy
from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point

class HeadClient():

    def __init__(self):
        name_space = '/head_traj_controller/point_head_action'
        self.head_client = SimpleActionClient(name_space, PointHeadAction)
        self.head_client.wait_for_server()
        self.hor_pos = 0.0
        self.vert_pos = 0.0
    
    def move_head_vert (self, vert_val):
        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = 'torso_lift_link'
        head_goal.max_velocity = .3
        self.vert_pos = vert_val/50.0 - 1
        head_goal.target.point = Point(1.5, self.hor_pos, self.vert_pos)
        
        self.head_client.send_goal(head_goal)
        if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Head action unsuccessful.')

    def move_head_hor (self, hor_val):
        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = 'torso_lift_link'
        head_goal.max_velocity = .3
        self.hor_pos = -(hor_val/20.0 - 2.5)
        head_goal.target.point = Point(1.5, self.hor_pos, self.vert_pos)
        
        self.head_client.send_goal(head_goal)
        if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Head action unsuccessful.')
