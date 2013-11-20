#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
import tf
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from head_object_tracking import HeadObjectTracking
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from catch_me import srv
import threading

from core import State


class ScanState(State):
    def run(self):
        # TODO:Lars start scan with the head
        pass

    def next(self, pose_stamped):
        pass

class FollowState(State):
    def __init__(self):
        rospy.loginfo('Waiting for move_base action to come up')
        self.base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.base_client.wait_for_server()
        rospy.loginfo('Connected to move_base action server')

    def run(self):
        pass

    def next(self, pose_stamped):
        pass

class MoveState(State):
    def __init__(self):
        rospy.loginfo('Waiting for move_base action to come up')
        self.base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.base_client.wait_for_server()
        rospy.loginfo('Connected to move_base action server')

    def run(self):
        pass

    def next(self, pose_stamped):
        pass

class KillState(State):
    def run(self):
        pass

    def next(self, pose_stamped):
        pass


class CatchMe:
  MIN_GOAL_TIME_MS = 5000

  def __init__(self):
    self._last_move = 0

    rospy.wait_for_service('catch_me_destination_service')
    self.destination_service = rospy.ServiceProxy('catch_me_destination_service', srv.DestinationService)


    threading.Timer(0, self.move_to_marker).start()

  def move_to_marker(self):
    pose_stamped = self.destination_service()
    print pose_stamped
    
    if pose_stamped is None:
      rospy.loginfo('No marker position found')

    pos = pose_stamped.pose.pose.position
    pos_frame_id = pose_stamped.pose.header.frame_id
    rospy.loginfo(str(pos_frame_id) + '\n' + str(pos.x) + ',' + str(pos.y) + ',' + str(pos.z))

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pos.x - 0.8
    goal.target_pose.pose.position.y = pos.y
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo('Sending new goal')
    # Cancel old goal if it exists.
    #self.base_client.cancel_all_goals()
    self.base_client.send_goal(goal)
    #self.base_client.wait_for_result()
    #rospy.loginfo(str(self.base_client.get_result()))
    threading.Timer(MIN_GOAL_TIME_MS, self.move_to_marker).start()

if __name__=='__main__':
    rospy.init_node('catch_me_node')
    dest_pub = CatchMe()
    rospy.spin()
 
