#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from ar_track_alvar.msg import AlvarMarkers

class DestinationPublisher:
  def __init__(self):
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_cb)
    self.pub = rospy.Publisher('catch_me_destination_pub', Pose)
    self._head_object_tracking = HeadObjectTracking()
    rospy.loginfo('Destination Publisher Started')

  def marker_cb(self, pose_markers):
    if len(pose_markers.markers) == 0:
      return
    rospy.loginfo('AR Marker Pose updating')


    pose = pose_markers.markers[0].pose.pose
    
    x_pos = pose.position.x
    y_pos = pose.position.y
    z_pos = pose.position.z
    self._head_object_tracking.new_tracking_data(x_pos, y_pos, z_pos)

    transform = DestinationPublisher.get_matrix_from_pose(pose)
    offset_array = [0.0, 0.0, 0.0]
    offset_transform = tf.transformations.translation_matrix(offset_array)
    hand_transform = tf.transformations.concatenate_matrices(transform,
                                                             offset_transform)
    trans_pose = DestinationPublisher.get_pose_from_transform(hand_transform)
    rospy.loginfo('Marker: \n' + str(pose) + ',\nDestination: \n' + str(trans_pose))
    self.pub.publish(trans_pose)

  @staticmethod
  def get_matrix_from_pose(pose):
    rotation = [pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]
    transformation = tf.transformations.quaternion_matrix(rotation)
    position = [pose.position.x, pose.position.y, pose.position.z]
    transformation[:3, 3] = position
    return transformation

  @staticmethod
  def get_pose_from_transform(transform):
    pos = transform[:3,3].copy()
    rot = tf.transformations.quaternion_from_matrix(transform)
    return Pose(Point(pos[0], pos[1], pos[2]),
                Quaternion(rot[0], rot[1], rot[2], rot[3]))

if __name__=='__main__':
    rospy.init_node('catch_me_destination_node')
    dest_pub = DestinationPublisher()
    rospy.spin()
 
