#! /usr/bin/env python

# Copyright (c) 2010, Washington University in St. Louis
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Washington University in St. Louis nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy
import actionlib
import learning_actionlib.msg


class MapSumAction(object):
    # create messages that are used to publish feedback/result

    _feedback = learning_actionlib.msg.MapSumFeedback()
    _result = learning_actionlib.msg.MapSumResult()

    def __init__(self, name):
        self._action_name = name
        _as = actionlib.SimpleActionServer(self._action_name, learning_actionlib.msg.MapSumAction, execute_cb=self.execute_cb, auto_start=False)
        _as.start()
        self._as = _as
        rospy.loginfo('%s: Starting' % (self._action_name))

    def execute_cb(self, goal):
        success = True
        # every sum starts with 0
        self._feedback.res = 0
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, finding sum of array %s' % (self._action_name, goal.vals))
        # start executing the action
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        self._feedback.res = sum(goal.vals)
        self._as.publish_feedback(self._feedback)

        if success:
            self._result.res = self._feedback.res
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('mapSum')
    MapSumAction(rospy.get_name())
    rospy.spin()
