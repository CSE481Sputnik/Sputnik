#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_msgs.msg
import time

class DestinationGoalPublisher:
	def __init__(self):
		rospy.Subscriber('catch_me_destination_pub', Pose, self.dest_cb)
		self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped)
		rospy.loginfo('Destination Goal Publisher Started')

	def dest_cb(self, position):
		#force sleep or rate i think it's rospy.rate(hz), but might need to be done to publisher instead
			
		if not position:
			rospy.loginfo('publisher seems to publish bad data')
			return
		rospy.loginfo('Setting new Goal')

		#make header
		header = std_msgs.msg.Header()
		header.stamp = rospy.Time.now()

		# when picking a goal, pick slightly infront of the robot, until it stops
		# We can worry about pathing later.
		# If the robot picks a destination within the expanded_obstacles, it will not move
		x = 1.0 * ( 1 if position.position.x > 0 else -1)
		y = 0.0 * ( 1 if position.position.y > 0 else -1)
		z = 0.0

		qx = 0.0 
		qy = 0.0
		qz = 0.0 * (1 if position.orientation.z > 0 else -1)
		qw = 0.0 * (1 if position.orientation.w > 0 else -1)

		dest_goal = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
		rospy.loginfo('Header:\n' + str(header) + ',\nGoal:\n' + str(dest_goal))
		self.pub.publish(PoseStamped(header, dest_goal))

if __name__ == '__main__':
	rospy.init_node('catch_me_goal_node')
	dest_goal_pub = DestinationGoalPublisher()
	rospy.spin()
