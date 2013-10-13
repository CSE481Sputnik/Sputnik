#!/usr/bin/env python

import roslib
roslib.load_manifest('sound_play')
roslib.load_manifest('rospy')

from subprocess import call
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from gripper_client import GripperClient
from base_publisher import BasePublisher


class SimpleGUI(Plugin):

    sound_sig = Signal(SoundRequest)

    def __init__(self, context):
        super(SimpleGUI, self).__init__(context)
        self.setObjectName('SimpleGUI')
        self._widget = QWidget()

        self._gripper_client = GripperClient()

        self._base_publisher = BasePublisher()
        
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.sound_sig.connect(self.sound_sig_cb)
        
        large_box = QtGui.QVBoxLayout()

        upper_box = QtGui.QHBoxLayout()
        large_box.addLayout(upper_box)
        
        lower_box = QtGui.QHBoxLayout()
        large_box.addLayout(lower_box)

        gripper_box = QtGui.QHBoxLayout()
        lower_box.addLayout(gripper_box)

        # GRIPPERS
        self.gripper_box_label = QtGui.QLabel('Grippers: ')
        gripper_box.addWidget(self.gripper_box_label)

#        self.gripper_left_label = QtGui.QLabel('L')
#        gripper_box.addWidget(self.gripper_left_label)

        gripper_left_btns = QtGui.QVBoxLayout()
        g_left_open_btn = QtGui.QPushButton('Open L', self._widget)
        g_left_open_btn.clicked.connect(self.gripper_cb)
        gripper_left_btns.addWidget(g_left_open_btn)
        g_left_close_btn = QtGui.QPushButton('Close L', self._widget)
        g_left_close_btn.clicked.connect(self.gripper_cb)
        gripper_left_btns.addWidget(g_left_close_btn)
        gripper_box.addLayout(gripper_left_btns)

#        self.gripper_right_label = QtGui.QLabel('R')
#        gripper_box.addWidget(self.gripper_right_label)

        gripper_right_btns = QtGui.QVBoxLayout()
        g_right_open_btn = QtGui.QPushButton('Open R', self._widget)
        g_right_open_btn.clicked.connect(self.gripper_cb)
        gripper_right_btns.addWidget(g_right_open_btn)
        g_right_close_btn = QtGui.QPushButton('Close R', self._widget)
        g_right_close_btn.clicked.connect(self.gripper_cb)
        gripper_right_btns.addWidget(g_right_close_btn)
        gripper_box.addLayout(gripper_right_btns)

        lower_box.addItem(QtGui.QSpacerItem(100,20))

        # SPEECH
        speech_box = QtGui.QHBoxLayout()
        
        self.speech_label = QtGui.QLabel('Speech: ')
        speech_box.addWidget(self.speech_label)

        self.speech_text = QtGui.QLineEdit(self._widget);
        speech_box.addWidget(self.speech_text);

        speech_box.addWidget(self.create_btn('Speak'))
        speech_box.addStretch(1)
        lower_box.addLayout(speech_box)

        large_box.addItem(QtGui.QSpacerItem(100,20))
        
#        speech_box = QtGui.QHBoxLayout()
#        self.speech_label = QtGui.QLabel('Robot has not spoken yet')
#        palette = QtGui.QPalette()
#        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
#        self.speech_label.setPalette(palette)
#        speech_box.addWidget(self.speech_label)

#        large_box.addLayout(speech_box)
        large_box.addStretch(1)


        # BASE MOVEMENT
        move_base_controls = QtGui.QVBoxLayout()

        move_fwd_controls = QtGui.QHBoxLayout()
        move_fwd_btn = self.create_toggle_btn("Forward")
        move_fwd_btn.clicked.connect(self.make_move_base_cd(0.5))
        move_fwd_controls.addWidget(move_fwd_btn)
        move_fwd_right_btn = self.create_toggle_btn("Forward Right")
        move_fwd_right_btn.clicked.connect(self.make_move_base_cd(0.5))
        move_fwd_controls.addWidget(move_fwd_right_btn)
        move_fwd_left_btn = self.create_toggle_btn("Forward Left")
        move_fwd_left_btn.clicked.connect(self.make_move_base_cd(0.5))
        move_fwd_controls.addWidget(move_fwd_left_btn)

        move_bkwd_controls = QtGui.QHBoxLayout()
        move_bkwd_btn = self.create_toggle_btn("Backward")
        move_bkwd_btn.clicked.connect(self.make_move_base_cd(0.5))
        move_bkwd_controls.addWidget(move_bkwd_btn)
        move_bkwd_right_btn = self.create_toggle_btn("Backward Right")
        move_bkwd_right_btn.clicked.connect(self.make_move_base_cd(0.5))
        move_bkwd_controls.addWidget(move_bkwd_right_btn)
        move_bkwd_left_btn = self.create_toggle_btn("Backward Left")
        move_bkwd_left_btn.clicked.connect(self.make_move_base_cd(0.5))
        move_bkwd_controls.addWidget(move_bkwd_left_btn)

        move_base_controls.addLayout(move_fwd_controls)
        move_base_controls.addLayout(move_bkwd_controls)

        upper_box.addLayout(move_base_controls)

        self._widget.setObjectName('SimpleGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        
    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    def create_btn(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def create_toggle_btn(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.setAutoRepeat(True)
        btn.setAutoRepeatInterval(10)
        return btn

    def sound_sig_cb(self, sound_request):
        qWarning('Received sound signal.')
        #if (sound_request.command == SoundRequest.SAY):
        qWarning('Robot said: ' + sound_request.arg)
        #self.speech_label.setText('Robot said: ' + sound_request.arg)

    def command_cb(self):
        btn_name = self._widget.sender().text()
        if (btn_name == 'Speak'):
            text = self.speech_text.text()
            qWarning('Robot will say: ' + text)
            self._sound_client.say(text)

    def gripper_cb(self):
        btn_name = self._widget.sender().text()
        if btn_name == 'Open L':
            self._gripper_client.command(True, False)
        elif btn_name == 'Open R':
            self._gripper_client.command(False, False)
        elif btn_name == 'Close L':
            self._gripper_client.command(True, True)
        elif btn_name == 'Close R':
            self._gripper_client.command(False, True)

    def make_move_base_cd(self, speed):
        def callback():
            self.move_base_cb(speed)
        return callback

    def move_base_cb(self, speed):
        qWarning('Received movement command.')
        btn_name = self._widget.sender().text()
        if btn_name == 'Forward':
            self._base_publisher.move_fwd_straight(speed)
        elif btn_name == 'Forward Right':
            self._base_publisher.move_fwd_right(speed)
        elif btn_name == 'Forward Left':
            self._base_publisher.move_fwd_left(speed)
        elif btn_name == 'Backward':
            self._base_publisher.move_bkwd_straight(speed)
        elif btn_name == 'Backward Right':
            self._base_publisher.move_bkwd_right(speed)
        elif btn_name == 'Backward Left':
            self._base_publisher.move_bkwd_left(speed)
            
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

