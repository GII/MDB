#!/usr/bin/env python

# Copyright 2018, GII / Universidad de la Coruna (UDC)
#
# Main contributor(s):
# * Luis Calvo, luis.calvo@udc.es
#
#  This file is also part of MDB.
#
# * MDB is free software: you can redistribute it and/or modify it under the
# * terms of the GNU Affero General Public License as published by the Free
# * Software Foundation, either version 3 of the License, or (at your option) any
# * later version.
# *
# * MDB is distributed in the hope that it will be useful, but WITHOUT ANY
# * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# * A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# * details.
# *
# * You should have received a copy of the GNU Affero General Public License
# * along with MDB. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *
from builtins import object
import rospy
from mdb_robots_policies.srv import GetJointsState
from sensor_msgs.msg import JointState


class get_joints_state(object):
    def __init__(self):
        rospy.init_node("get_joints_state_server")

        self.current_state = None

        self.joint_states_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_states_cb)
        self.getjs_srv = rospy.Service("get_joints_state", GetJointsState, self.handle_getjs)

    def joint_states_cb(self, state):
        self.current_state = state

    def handle_getjs(self, req):
        if req.request.data == True:
            print("Sending current joint states")
            while len(self.current_state.position) == 1:
                pass
            return self.current_state
        else:
            return None


def get_joints_state_server():
    get_joints_state()
    rospy.spin()


if __name__ == "__main__":
    get_joints_state_server()
