#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from catch_me import srv
import threading
from catch_me.msg import *
from actionlib_msgs.msg import GoalStatus
import copy 

class CatchMe:
  FOLLOW_DIST = .5

  def __init__(self):
    self._last_move = 0
    self._last_pose = None

    self.tf = tf.TransformListener()

    rospy.loginfo('Waiting for catch_me_destination_service')
    rospy.wait_for_service('catch_me_destination_service')
    rospy.loginfo('Connected to catch_me_destination_service')
    self.destination_service = rospy.ServiceProxy('catch_me_destination_service', srv.DestinationService)

    rospy.loginfo('Waiting for move_base action to come up')
    self.base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.base_client.wait_for_server()
    rospy.loginfo('Connected to move_base action server')

    rospy.loginfo('Starting head-searching action')
    self.head_s_client = actionlib.SimpleActionClient('local_search', LocalSearchAction)
    self.head_s_client.wait_for_server()
    rospy.loginfo('Head-search action client started')

    threading.Timer(0, self.follow_marker).start()

  def follow_marker(self):
    pose_stamped = self.destination_service()
    
    if (pose_stamped is None or
          pose_stamped.pose.header.stamp.secs == 0 or
          pose_stamped.pose == self._last_pose):
      # Either, no marker known, or marker is old
      if pose_stamped.pose == self._last_pose:
        rospy.loginfo('Marker position unchanged')
      else:
        rospy.loginfo('No marker position found')

      if self.head_s_client.get_state() != GoalStatus.ACTIVE:
        goal = LocalSearchGoal(True)
        self.head_s_client.send_goal(goal)

    else:
      # New marker seen, try to move to it
      pos = pose_stamped.pose.pose.position
      orient = pose_stamped.pose.pose.orientation
      pos_frame_id = pose_stamped.pose.header.frame_id
      rospy.loginfo(str(pos_frame_id) + '\n' + str(pos.x) + ',' + str(pos.y) + ',' + str(pos.z))

      if pose_stamped.pose != self._last_pose or self.base_client.get_state() != GoalStatus.SUCCEEDED:
        rospy.loginfo('Marker position updated.  Moving towards marker.')
        self._last_pose = copy.deepcopy(pose_stamped.pose)

        self.move_to_marker(pose_stamped)

    goal_status = self.base_client.get_state()
    if goal_status == GoalStatus.PENDING or goal_status == GoalStatus.ACTIVE:
      # if we are currently moving/planning, slow update rate
      update_rate = 3
    else:
      # if we are currently a sitting duck, fast update rate
      update_rate = 0.5
    threading.Timer(update_rate, self.follow_marker).start()


  def move_to_marker(self, pose_stamped):
    pose = pose_stamped.pose
    pose.header.stamp = rospy.Time.now()

    goal = MoveBaseGoal()
    goal.target_pose = pose

    # The robot can't move in these directions. Clear them so it's not confused
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0


    # If the marker position is fresh, stay behind it so we don't collide.
    # Use pose_stamped to get original time
    if pose_stamped.pose.header.stamp.secs + 2 > rospy.Time.now().secs:
      goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - self.FOLLOW_DIST


    rospy.loginfo('Sending new goal. x: ' + str(goal.target_pose.pose.position.x) + ', y: ' + str(goal.target_pose.pose.position.y))
    # Cancel old goal if it exists.
    self.base_client.cancel_all_goals()
    self.base_client.send_goal(goal)
    #self.base_client.wait_for_result()
    #rospy.loginfo(str(self.base_client.get_result()))

if __name__=='__main__':
    rospy.init_node('catch_me_node')
    dest_pub = CatchMe()
    rospy.spin()
 
