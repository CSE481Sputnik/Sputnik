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


class SimpleGUI(Plugin):

    sound_sig = Signal(SoundRequest)

    def __init__(self, context):
        super(SimpleGUI, self).__init__(context)
        self.setObjectName('SimpleGUI')
        self._widget = QWidget()

        self._gripper_client = GripperClient()
        
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.sound_sig.connect(self.sound_sig_cb)
        
        large_box = QtGui.QVBoxLayout()
        
        lower_box = QtGui.QHBoxLayout()
        large_box.addLayout(lower_box)

        gripper_box = QtGui.QHBoxLayout()
        lower_box.addLayout(gripper_box)

        self.gripper_box_label = QtGui.QLabel('Grippers: ')
        gripper_box.addWidget(self.gripper_box_label)

#        self.gripper_left_label = QtGui.QLabel('L')
#        gripper_box.addWidget(self.gripper_left_label)

        gripper_left_buttons = QtGui.QVBoxLayout()
        g_left_open_button = QtGui.QPushButton('Open L', self._widget)
        g_left_open_button.clicked.connect(self.gripper_cb)
        gripper_left_buttons.addWidget(g_left_open_button)
        g_left_close_button = QtGui.QPushButton('Close L', self._widget)
        g_left_close_button.clicked.connect(self.gripper_cb)
        gripper_left_buttons.addWidget(g_left_close_button)
        gripper_box.addLayout(gripper_left_buttons)

#        self.gripper_right_label = QtGui.QLabel('R')
#        gripper_box.addWidget(self.gripper_right_label)

        gripper_right_buttons = QtGui.QVBoxLayout()
        g_right_open_button = QtGui.QPushButton('Open R', self._widget)
        g_right_open_button.clicked.connect(self.gripper_cb)
        gripper_right_buttons.addWidget(g_right_open_button)
        g_right_close_button = QtGui.QPushButton('Close R', self._widget)
        g_right_close_button.clicked.connect(self.gripper_cb)
        gripper_right_buttons.addWidget(g_right_close_button)
        gripper_box.addLayout(gripper_right_buttons)

        lower_box.addItem(QtGui.QSpacerItem(100,20))

	speech_box = QtGui.QHBoxLayout()
        
        self.speech_label = QtGui.QLabel('Speech: ')
        speech_box.addWidget(self.speech_label)

        self.speech_text = QtGui.QLineEdit(self._widget);
        speech_box.addWidget(self.speech_text);

        speech_box.addWidget(self.create_button('Speak'))
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

        self._widget.setObjectName('SimpleGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        
    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    def create_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(self.command_cb)
        return btn

    def sound_sig_cb(self, sound_request):
        qWarning('Received sound signal.')
        #if (sound_request.command == SoundRequest.SAY):
        qWarning('Robot said: ' + sound_request.arg)
        self.speech_label.setText('Robot said: ' + sound_request.arg)

    def command_cb(self):
        button_name = self._widget.sender().text()
        if (button_name == 'Speak'):
            text = self.speech_text.text()
            qWarning('Robot will say: ' + text)
            self._sound_client.say(text)

    def gripper_cb(self):
        button_name = self._widget.sender().text()
        if button_name == 'Open L':
            self._gripper_client.command(True, False)
        elif button_name == 'Open R':
            self._gripper_client.command(False, False)
        elif button_name == 'Close L':
            self._gripper_client.command(True, True)
        elif button_name == 'Close R':
            self._gripper_client.command(False, True)
            
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

