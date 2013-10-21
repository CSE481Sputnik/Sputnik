#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('actionlib')

from subprocess import call
from collections import defaultdict
import threading
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient

class ArmGUI(Plugin):

    joint_sig = Signal(JointState)

    def __init__(self, context):
        super(ArmGUI, self).__init__(context)
        self.setObjectName('ArmGUI')
        self._widget = QWidget()
        
        # Action/service/message clients or servers
        
        switch_srv_name = 'pr2_controller_manager/switch_controller'
        rospy.loginfo('Waiting for switch controller service...')
        rospy.wait_for_service(switch_srv_name)
        self.switch_service_client = rospy.ServiceProxy(switch_srv_name,
                                                 SwitchController)
                                                 
        self.r_joint_names = ['r_shoulder_pan_joint',
                              'r_shoulder_lift_joint',
                              'r_upper_arm_roll_joint',
                              'r_elbow_flex_joint',
                              'r_forearm_roll_joint',
                              'r_wrist_flex_joint',
                              'r_wrist_roll_joint']
        self.l_joint_names = ['l_shoulder_pan_joint',
                              'l_shoulder_lift_joint',
                              'l_upper_arm_roll_joint',
                              'l_elbow_flex_joint',
                              'l_forearm_roll_joint',
                              'l_wrist_flex_joint',
                              'l_wrist_roll_joint']

        self.all_joint_names = []
        self.all_joint_poses = []

        # saving our poses
        self.saved_r_arm_poses = defaultdict(lambda: list())
        self.saved_l_arm_poses = defaultdict(lambda: list())
        self.saved_pose_sets = set()

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)


        # Create a trajectory action client
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
        
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()
        
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.joint_sig.connect(self.joint_sig_cb)
        
        large_box = QtGui.QVBoxLayout()
        
        button_box1 = QtGui.QHBoxLayout()
        button_box1.addWidget(self.create_button('Relax arms'))
        button_box1.addWidget(self.create_button('Freeze arms'))
        button_box1.addStretch(1)
        large_box.addLayout(button_box1)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        button_box2 = QtGui.QHBoxLayout()
        self.pose_set_text = QtGui.QLineEdit(self._widget)
        button_box2.addWidget(self.pose_set_text)
        button_box2.addWidget(self.create_button('Add to Pose Set'))
        button_box2.addStretch(1)
        large_box.addLayout(button_box2)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        button_box3 = QtGui.QHBoxLayout()
        self.pose_selector = QtGui.QComboBox()
        self.pose_selector.setSizeAdjustPolicy(QtGui.QComboBox.AdjustToContents)
        self.pose_selector.currentIndexChanged[str].connect(self.update_pose_set_length)
        button_box3.addWidget(self.pose_selector)

        self.pose_set_length_label = QtGui.QLabel()
        button_box3.addWidget(self.pose_set_length_label)
        self.update_pose_set_length()

        button_box3.addWidget(self.create_button('Do Pose Set'))
        button_box3.addItem(QtGui.QSpacerItem(50,20))
        button_box3.addWidget(self.create_button('Delete Pose Set'))
        button_box3.addStretch(1)
        large_box.addLayout(button_box3)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        poses_box = QtGui.QVBoxLayout()
        self.status_message_label = QtGui.QLabel('No Saved Poses')
        poses_box.addWidget(self.status_message_label)
        large_box.addLayout(poses_box)

        large_box.addStretch(1)
        self._widget.setObjectName('ArmGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        rospy.loginfo('GUI initialization complete.')
        
        

    def create_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    # This is a really janky way of doing this, you should check
    # self._widget.sender() to figure out where the event originated.
    def command_cb(self):
        button_name = self._widget.sender().text()
        if (button_name == 'Relax arms'):
            self.relax_arm('r')
            self.relax_arm('l')
        elif (button_name == 'Freeze arms'):
            self.freeze_arm('r')
            self.freeze_arm('l')
        elif (button_name == 'Add to Pose Set'):
            self.save_pose()
        elif (button_name == 'Do Pose Set'):
            self.move_arm()
        elif (button_name == 'Delete Pose Set'):
            self.delete_pose()
        self.update_pose_set_length()


    def save_pose(self):
        pose_set = self.pose_set_text.text()
        if len(pose_set) is 0 or pose_set is None:
            self.status_message_label.setText("Invalid pose name")
            return

        # if not already saved, make a new pose set
        if pose_set not in self.saved_pose_sets:
            self.pose_selector.addItem(pose_set)
            self.saved_pose_sets.add(pose_set)
        # auto select the saved pose
        index = self.pose_selector.findText(pose_set)
        self.pose_selector.setCurrentIndex(index)


        r_joint_state = self.get_joint_state('r')
        l_joint_state = self.get_joint_state('l')

        self.saved_r_arm_poses[pose_set].append(r_joint_state)
        self.saved_l_arm_poses[pose_set].append(l_joint_state)

        self.saved_pose_sets.add(pose_set)

        self.status_message_label.setText('Pose saved!')


    def update_pose_set_length(self):
        pose_set = self.pose_selector.currentText()
        text = "(%d)" % len(self.saved_r_arm_poses[pose_set])
        self.pose_set_length_label.setText(text)


    def move_arm(self):
        pose_set = self.pose_selector.currentText()
        
        if not pose_set in self.saved_pose_sets:
            rospy.logerr("Target pose set does not exist, I cannot move my little arms!")
            self.status_message_label.setText("No pose set")
            return
        
        if pose_set in self.saved_r_arm_poses:
            self.freeze_arm('r')
            self.move_to_joints('r', self.saved_r_arm_poses[pose_set], 1.0)

        if pose_set in self.saved_l_arm_poses:
            self.freeze_arm('l')
            self.move_to_joints('l', self.saved_l_arm_poses[pose_set], 1.0)

        self.status_message_label.setText('Pose executing!')

    def delete_pose(self):
        pose_set = self.pose_selector.currentText()
        rospy.loginfo('Clearing pose set %s' % pose_set)

        # removing from the select box
        pose_set_len = len(self.saved_r_arm_poses[pose_set])
        index = self.pose_selector.findText(pose_set)
        self.pose_selector.removeItem(index)

        # removing from the pose set maps
        self.saved_r_arm_poses.pop(pose_set, None)
        self.saved_l_arm_poses.pop(pose_set, None)
        self.saved_pose_sets.remove(pose_set)

        self.status_message_label.setText('Pose deleted!')


    def move_to_joints(self, side_prefix, positions, time_to_joint):
        '''Moves the arm to the desired joints'''
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        time_move = time_to_joint
        print "using following positions %s" % positions
        for pose in positions:
            velocities = [0] * len(pose)
            traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=pose,
                            velocities=velocities, time_from_start=rospy.Duration(time_move)))
            time_move += time_to_joint
	
        if (side_prefix == 'r'):
            traj_goal.trajectory.joint_names = self.r_joint_names
            self.r_traj_action_client.send_goal(traj_goal)
        else:
            traj_goal.trajectory.joint_names = self.l_joint_names
            self.l_traj_action_client.send_goal(traj_goal)

    def relax_arm(self, side_prefix):
        controller_name = side_prefix + '_arm_controller'
        start_controllers = []
        stop_controllers = [controller_name]
        self.set_arm_mode(start_controllers, stop_controllers)

    def freeze_arm(self, side_prefix):
        controller_name = side_prefix + '_arm_controller'
        start_controllers = [controller_name]
        stop_controllers = []
        self.set_arm_mode(start_controllers, stop_controllers)

    def set_arm_mode(self, start_controllers, stop_controllers):
        try:
            self.switch_service_client(start_controllers, stop_controllers, 1)
        except rospy.ServiceException:
            rospy.logerr('Could not change arm mode.')

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
            joint_states message is received'''
        self.lock.acquire()
        self.all_joint_names = msg.name
        self.all_joint_poses = msg.position
        self.joint_sig.emit(msg)
        self.lock.release()

    def joint_sig_cb(self, msg):
        pass

    def get_joint_state(self, side_prefix):
        '''Returns position for arm joints on the requested side (r/l)'''
        if side_prefix == 'r':
            joint_names = self.r_joint_names
        else:
            joint_names = self.l_joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received yet!\n")
            return None
    
        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
                self.lock.release()
                return None

        self.lock.release()
        return positions


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # Leave both arm controllers on
        start_controllers = ['r_arm_controller', 'l_arm_controller']
        stop_controllers = []
        self.set_arm_mode(start_controllers, stop_controllers)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

