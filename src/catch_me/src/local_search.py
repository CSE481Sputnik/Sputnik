#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('geometry_msgs')
import math
import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point
from ar_track_alvar.msg import AlvarMarkers
from catch_me.msg import *


#TODO: import point stamped

VISUAL_FIELD_SIZE = 55
MIN_HEAD_ANGLE = -140
MAX_HEAD_ANGLE = 140

class LocalSearch():
  _feedback = LocalSearchFeedback()
  _result = LocalSearchResult()

  def __init__(self):
    self._action_name = 'local_search'
    self.found_marker = False
    self.tracking_started = False
    
    #initialize head mover
    name_space = '/head_traj_controller/point_head_action'
    self.head_client = SimpleActionClient(name_space, PointHeadAction)
    self.head_client.wait_for_server()
    rospy.loginfo('%s: Action client for PointHeadAction running' % self._action_name)

    #initialize tracker mark
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_tracker)
    rospy.loginfo('%s: subscribed to AlvarMarkers' % self._action_name)
    
    #initialize this client
    self._as = SimpleActionServer(self._action_name, LocalSearchAction, execute_cb=self.run, auto_start=False)
    self._as.start()
    rospy.loginfo('%s: started' % self._action_name)
    
  def marker_tracker(self, pose_markers):
    if len(pose_markers.markers) == 0:
      return
    else:
      if self.tracking_started:
        self.found_marker = True  
        rospy.loginfo('%s: marker found' % self._action_name)
    
  def run(self, goal):
    r = rospy.Rate(.5)
    success = False;
    self.tracking_started = True
    self.found_marker = False

    rospy.loginfo('%s: Executing search for AR marker' % self._action_name)
    for cur_angle in xrange(0, 360, VISUAL_FIELD_SIZE):
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Premepted' % self._action_name)
        self._as.set_preempted()
	return
      
      head_goal = self.lookat_goal(cur_angle)
      self.head_client.send_goal(head_goal)
      self.head_client.wait_for_result(rospy.Duration.from_sec(5.0))
      if (self.head_client.get_state() != GoalStatus.SUCCEEDED):
        rospy.logwarn('Head could not move to specified location')
        break
      r.sleep()
      if (self.found_marker):
        success = True
        break

    if success:
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded()
    else:
      self._as.set_aborted()
    self.tracking_started = False
    return

  def lookat_goal(self, angle):
    angle = angle - 140
    if angle > 140:
	angle = 160

    head_goal = PointHeadGoal()
    head_goal.target.header.frame_id = '/torso_lift_link'
    head_goal.max_velocity = 1

    angle_in_radians = math.radians(angle)
    x = math.cos(angle_in_radians) * 5
    y = math.sin(angle_in_radians) * 5
    z = .4
    
    head_goal.target.point = Point(x, y, z)

    rospy.loginfo('%s: Head move goal for %s: %s produced' % (self._action_name, str(angle), str(head_goal)))
    return head_goal

if __name__=='__main__':
  rospy.init_node('catch_me_local_search')
  local_search = LocalSearch() 
  rospy.spin()
