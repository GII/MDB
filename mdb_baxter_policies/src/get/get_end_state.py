#!/usr/bin/env python

import rospy
from mdb_baxter_moveit.srv import GetES
from baxter_core_msgs.msg import EndpointState

class get_end_state():
	def __init__(self):
		rospy.init_node('get_end_state_server')
		
		self.left_current_state = None
		self.right_current_state = None

		self.left_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.left_state_cb)
		self.right_state_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.right_state_cb)
		self.getes_srv = rospy.Service('get_end_state', GetES, self.handle_getes)

	def left_state_cb(self, state):
		self.left_current_state = state

	def right_state_cb(self, state):
		self.right_current_state = state

	def handle_getes(self, req):
		if (req.request.data == 'left'):
			print "Sending current larm endstate"
			return self.left_current_state
		elif (req.request.data == 'right'):
			print "Sending current rarm endstate"
			return self.right_current_state
		else:
			return None

def get_end_state_server():
	get_end_state()
	rospy.spin()

if __name__ == "__main__":
    get_end_state_server()
