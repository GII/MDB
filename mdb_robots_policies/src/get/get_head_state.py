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
from mdb_robots_policies.srv import GetHeadState
from baxter_core_msgs.msg import HeadState

class get_head_state():
	def __init__(self):
		rospy.init_node('get_head_state_server')
		
		self.current_state = None

		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.geths_sv = rospy.Service('/get_head_state', GetHeadState, self.handle_geths)

	def head_state_cb(self, state):
		self.current_state = state

	def handle_geths(self, req):
		if (req.request.data == True):
			print "Sending current joint states"
			#while len(self.current_state.position)==1:
				#pass
			return self.current_state
		else:
			return None

def get_head_state_server():
	get_head_state()
	rospy.spin()

if __name__ == "__main__":
    get_head_state_server()
