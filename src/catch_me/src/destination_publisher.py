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

class DestinationPublisher:
  def __init__(self):
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_cb)
    self.pub = rospy.Publisher('catch_me_destination_pub', PoseStamped)

    rospy.loginfo('Waiting for move_base action to come up')
    self.base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.base_client.wait_for_server()
    rospy.loginfo('Connected to move_base action server')

    self._head_object_tracking = HeadObjectTracking()
    rospy.loginfo('Destination Publisher Started')

  def marker_cb(self, pose_markers):
    if len(pose_markers.markers) == 0:
      return

    rospy.loginfo('AR Marker Pose updating')
    pose = pose_markers.markers[0].pose.pose
    pose_header = pose_markers.markers[0].header
    
    x_pos = pose.position.x
    y_pos = pose.position.y
    z_pos = pose.position.z
    self._head_object_tracking.new_tracking_data(x_pos, y_pos, z_pos)

    if self.base_client.get_state == actionlib.SimpleGoalState.PENDING or self.base_client.get_state == actionlib.SimpleGoalState.ACTIVE:
      return

    pos_frame_id = pose_header.frame_id
    rospy.loginfo(str(pos_frame_id) + '\n' + str(x_pos) + ',' + str(y_pos) + ',' + str(z_pos))

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_pos - 0.8
    goal.target_pose.pose.position.y = y_pos
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo('Sending new goal')
    self.base_client.send_goal(goal)
    self.base_client.wait_for_result()
    rospy.loginfo(str(self.base_client.get_result()))

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
 
