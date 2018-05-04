#!/usr/bin/env python

import rospy
from mdb_baxter_moveit.srv import GetJS
from sensor_msgs.msg import JointState

class get_joints_state():
	def __init__(self):
		rospy.init_node('get_joints_state_server')
		
		self.current_state = None

		self.joint_states_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_states_cb)
		self.getjs_srv = rospy.Service('get_joints_state', GetJS, self.handle_getjs)

	def joint_states_cb(self, state):
		self.current_state = state

	def handle_getjs(self, req):
		if (req.request.data == True):
			print "Sending current joint states"
			while len(self.current_state.position)==1:
				pass
			return self.current_state
		else:
			return None

def get_joints_state_server():
	get_joints_state()
	rospy.spin()

if __name__ == "__main__":
    get_joints_state_server()
