#!/usr/bin/env python
from collections import deque

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

class HeadObjectTracking():

    # The number of previous data points to factor into this moving average
    DEFAULT_MAX_DATA = 10

    def __init__(self, max_data_points=DEFAULT_MAX_DATA):
        name_space = '/head_traj_controller/point_head_action'
        self.head_client = SimpleActionClient(name_space, PointHeadAction)
        self.head_client.wait_for_server()

        self.point_buffer = deque(maxlen=max_data_points)
        self.curr_tracking_point = Point(0,0,0)

    def new_tracking_data(self, data_x, data_y, data_z):
        """
        Adds a new tracking data point for the head.

        Points the head to a point taken as a moving average over some number of
        previous tracking data points.
        """
        # catch the last data point
        if len(self.point_buffer) > 0:
            last_data_point = self.point_buffer[-1]
        else:
            last_data_point = Point(0,0,0)

        # add a new data point
        self.point_buffer.append(Point(data_x, data_y, data_z))
        count = len(self.point_buffer)

        # calculate the moving average of the x, y, z positions
        tracking_point = self.curr_tracking_point
        avg_x = tracking_point.x - (last_data_point.x / count) + (data_x / count)
        avg_y = tracking_point.y - (last_data_point.y / count) + (data_y / count)
        avg_z = tracking_point.z - (last_data_point.z / count) + (data_z / count)

        # make a new averaged point to track and point the head there
        self.curr_tracking_point = Point(avg_x, avg_y, avg_z)
        self.point_head(avg_x, avg_y, avg_z)

    def point_head(self, x, y, z):
        """
        Point the head to the specified point
        """
        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = 'torso_lift_link'
        head_goal.max_velocity = .3
        head_goal.target.point = Point(x, y, z)
        
        self.head_client.send_goal(head_goal)
        if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Head action unsuccessful.')
