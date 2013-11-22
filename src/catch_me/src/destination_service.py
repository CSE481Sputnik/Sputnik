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
    self.tf = tf.TransformListener()
    # I need a better default value...
    self.pose = None
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_cb)
    dest_serv = rospy.Service('catch_me_destination_service', srv.DestinationService, self.service_cb)

  def marker_cb(self, pose_markers):
    for i in range(0, len(pose_markers.markers)):
      marker = pose_markers.markers[i]
      #get marker 1
      if marker.id == 1:
        rospy.loginfo('AR Marker Pose updating')    
#        #get the transform from torso_life_link to map
#        (trans, rot) = self.tf.lookupTransform('/torso_lift_link', '/map', rospy.Time(0))
        pose = marker.pose
        pose.header = marker.header # Marker has the valid header
        trans_pose = self.tf.transformPose('/map', pose)
#        orient = pose.pose.orientation
#        euler1 = tf.transformations.euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))
#        euler2 = tf.transformations.euler_from_quaternion(rot)
#        orient_trans = tf.transformations.quaternion_from_euler(euler1[0] - euler2[0], euler1[1] - euler2[1], euler1[2] - euler2[2])
        
        #make a new pose, which is the current ar markers pose mathed into a static position relative to the map
        '''self.pose = PoseStamped()
        self.pose.pose.position.x = pose.pose.position.x - trans[0]
        self.pose.pose.position.y = pose.pose.position.y - trans[1]
        self.pose.pose.position.z = pose.pose.position.z - trans[2]
        self.pose.pose.orientation = Quaternion(*orient_trans)
        self.pose.pose.orientation.x = -pose.pose.orientation.x * rot[0]
        self.pose.pose.orientation.y = -pose.pose.orientation.y * rot[1]
        self.pose.pose.orientation.z = pose.pose.orientation.z * rot[2]
        self.pose.pose.orientation.w = pose.pose.orientation.w * rot[3]
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = '/map'
        self.pose = PoseStamped()'''
        trans_pose.pose.orientation.x = -trans_pose.pose.orientation.x
        trans_pose.pose.orientation.y = -trans_pose.pose.orientation.y
#        trans_pose.pose.orientation.z = 0
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
 
