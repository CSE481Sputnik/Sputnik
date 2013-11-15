#!/usr/bin/env python

import roslib
roslib.load_manifest('simple_navigation_goals')

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

if __name__=='__main__':
    rospy.init_node('simple_navigation_goals')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    #print "waiting for server"
    #client.wait_for_server()
    print "connected to server"
    rospy.loginfo('connected to move_base server')
    goal = MoveBaseGoal()
    # currently these are random numbers I found from 
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0
    print 'sending goal'
    rospy.loginfo('sending goal')
    client.send_goal(goal)
    #client.wait_for_result()
    print client.get_result()
