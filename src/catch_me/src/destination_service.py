#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
import tf
from ar_track_alvar.msg import AlvarMarker
from catch_me import srv

class DestinationService:
  def __init__(self):
    self.tf = tf.TransformListener()
    # I need a better default value...
    self.pose = None
    rospy.Subscriber('catch_me_destination_publisher', AlvarMarker, self.marker_cb)
    dest_serv = rospy.Service('catch_me_destination_service', srv.DestinationService, self.service_cb)

  def marker_cb(self, marker):
    rospy.loginfo('AR Marker Pose updating')    
    pose = marker.pose
    pose.header = marker.header # Marker has the valid header
    trans_pose = self.tf.transformPose('/map', pose)
    trans_pose.pose.orientation.x = -trans_pose.pose.orientation.x
    trans_pose.pose.orientation.y = -trans_pose.pose.orientation.y
    self.pose = trans_pose
#        common_time = self.tf.getLatestCommonTime('/ar_marker_1', '/map')
#        self.pose.header.stamp = common_time
#        self.pose.header.frame_id = '/ar_marker_1'
#        self.pose.pose = pose
#        self.pose = self.tf.transformPose('/map', self.pose)
         

  def service_cb(self, dummy):
    rospy.loginfo('Returning pose ' + str(self.pose))
    return srv.DestinationServiceResponse(self.pose)

if __name__=='__main__':
    rospy.init_node('catch_me_destination_service_node')
    s = DestinationService()
    rospy.spin()
 
