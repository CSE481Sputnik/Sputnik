#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
import tf
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from ar_track_alvar.msg import AlvarMarkers
from head_object_tracking import HeadObjectTracking
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from catch_me import srv

class DestinationService:
  def __init__(self):
    # I need a better default value...
    self.pose = None
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_cb)
    dest_serv = rospy.Service('catch_me_destination_service', srv.DestinationService, self.service_cb)

  def marker_cb(self, pose_markers):
    if len(pose_markers.markers) == 0:
      return

    rospy.loginfo('AR Marker Pose updating')
    self.pose = pose_markers.markers[0].pose
    self.pose.header.stamp = rospy.Time.now()

  def service_cb(self, dummy):
    rospy.loginfo('Returning pose ' + str(self.pose))
    return srv.DestinationServiceResponse(self.pose)

if __name__=='__main__':
    rospy.init_node('catch_me_destination_service_node')
    s = DestinationService()
    rospy.spin()
 
