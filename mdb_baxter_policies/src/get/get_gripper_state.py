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

import rospy
from mdb_baxter_policies.srv import GetGripperState
from baxter_core_msgs.msg import EndEffectorState

class get_gripper_state():
	def __init__(self):
		rospy.init_node('get_gripper_state_server')
		
		self.left_current_state = None
		self.right_current_state = None

		self.lgrip_sb = rospy.Subscriber("/robot/end_effector/left_gripper/state", EndEffectorState, self.lgrip_cb)
		self.rgrip_sb = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, self.rgrip_cb)
		self.getgs_sv = rospy.Service('get_gripper_state', GetGripperState, self.handle_getgs)

	def lgrip_cb(self, state):
		self.left_current_state = state

	def rgrip_cb(self, state):
		self.right_current_state = state

	def handle_getgs(self, req):
		if (req.request.data == 'left'):
			print "Sending current larm gripper state"
			return self.left_current_state
		elif (req.request.data == 'right'):
			print "Sending current rarm gripper state"
			return self.right_current_state
		else:
			return None

def get_gripper_state_server():
	get_gripper_state()
	rospy.spin()

if __name__ == "__main__":
    get_gripper_state_server()
