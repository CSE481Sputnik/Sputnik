#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

class BasePublisher():

    def __init__(self):
        topic_name = '/base_controller/command'
        self._base_publisher = rospy.Publisher(topic_name, Twist)

    def move_fwd_straight(self, speed):
        linear = Vector3(speed, 0.0, 0.0)
        self._move_tick(linear_vector=linear)

    def move_fwd_right(self, speed):
        linear = Vector3(speed, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, -1.0*speed)
        self._move_tick(linear, angular)

    def move_fwd_left(self, speed):
        linear = Vector3(speed, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, 1.0*speed)
        self._move_tick(linear, angular)

    def move_bkwd_straight(self, speed):
        linear = Vector3(-1.0*speed, 0.0, 0.0)
        self._move_tick(linear_vector=linear)

    def move_bkwd_right(self, speed):
        linear = Vector3(-1.0*speed, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, 1.0*speed)
        self._move_tick(linear, angular)

    def move_bkwd_left(self, speed):
        linear = Vector3(-1.0*speed, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, -1.0*speed)
        self._move_tick(linear, angular)

    def _move_tick(self, linear_vector=Vector3(0.0, 0.0, 0.0),
                         angular_vector=Vector3(0.0, 0.0, 0.0)):
        twist_msg = Twist()
        twist_msg.linear = linear_vector
        twist_msg.angular = angular_vector

        self._base_publisher.publish(twist_msg)
