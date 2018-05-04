#!/usr/bin/env python

import rospy
from mdb_baxter_moveit.srv import GetHS
from baxter_core_msgs.msg import HeadState
from sensor_msgs.msg import JointState

class get_head_state():
	def __init__(self):
		rospy.init_node('get_joints_state_server')
		
		self.current_state = None

		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.geths_sv = rospy.Service('/get_head_state', GetHS, self.handle_geths)

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
