#! /usr/bin/env python

import roslib; roslib.load_manifest('rqt_simplegui')
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import SimplePostConstraint
from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.msg import OrientationConstraint

def pose_constraint_to_position_orientation_constraints(pose_constraint):
    position_constraint = PositionConstraint()
    orientation_constraint = OrientationConstraint()
    position_constraint.header = pose_constraint.header
    position_constraint.link_name = pose_constraint.link_name
    position_constraint.position = pose_constraint.pose.position

    position_constraint.constraint_region_shape.type = Shape.BOX
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
    position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)

    position_constraint.constraint_region_orientation.x = 0.0
    position_constraint.constraint_region_orientation.y = 0.0
    position_constraint.constraint_region_orientation.z = 0.0
    position_constraint.constraint_region_orientation.w = 1.0

    position_constraint.weight = 1.0

    orientation_constraint.header = pose_constraint.header
    orientation_constraint.link_name = pose_constraint.link_name
    orientation_constraint.orientation = pose_constraint.pose.orientation
    orientation_constraint.type = pose_constraint.orientation_constraint_type

    orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
    orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
    orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
    orientation_constraint.weight = 1.0

    return position_constraint, orientation_constraint

def add_goal_constraint_to_move_arm_goal(pose_constraint, move_arm_goal):
    position_constraint, orientation_constraint = pose_constraint_to_position_orientation_constraints(pose_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

if __name__ == '__main__':
    rospy.init_node('r_arm_move_goal', anonymous=True)
    move_arm_client = SimpleActionClient('move_right_arm', MoveArmAction)
    
    move_arm_client.wait_for_server()

    goal = MoveArmGoal()
    goal.planner_server_name = 'ompl_planning/plan_kinematic_path'
    goal.motion_plan_request.planner_id = ''
    goal.motion_plan_request.group_name = 'right_arm'
    goal.motion_plan_request.num_planning_attempts = 1
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
    
    desired_pose = SimplePoseConstraint()
    desired_pose.header.frame_id = 'torso_lift_link'
    desired_pose.link_name = 'r_wrist_roll_link'
    desired_pose.pose.position.x = 0.75
    desired_pose.pose.position.y = -0.188
    desired_pose.pose.position.z = 0

    desired_pose.pose.orientation.x = 0.0
    desired_pose.pose.orientation.y = 0.0
    desired_pose.pose.orientation.z = 0.0
    desired_pose.pose.orientation.w = 1.0

    desired_pose.absolute_position_tolerance.x = 0.02
    desired_pose.absolute_position_tolerance.y = 0.02
    desired_pose.absolute_position_tolerance.z = 0.02

    # We want to allow the arm to rotate to get to a position and 2pi > 6.00
    desired_pose.absolute_roll_tolerance = 6.00
    desired_pose.absolute_pitch_tolerance = 6.00
    desired_pose.absolute_yaw_tolerance = 6.00

    add_goal_constraint_to_move_arm_goal(desired_pose, goal)

    move_arm_client.send_goal(goal)
    finished = move_arm_client.wait_for_result(rospy.Duration(120.0))

    if not finished:
        rospy.loginfo('Timed out trying to move arm to goal')
    else:
        state = move_arm_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Action finished and was successful')
        else:
            rospy.loginfo('Action failed to move arm to goal')
