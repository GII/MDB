#!/usr/bin/env python

import rospy
from mdb_baxter_policies.srv import GetGS
from baxter_core_msgs.msg import EndEffectorState

class get_gripper_state():
	def __init__(self):
		rospy.init_node('get_gripper_state_server')
		
		self.left_current_state = None
		self.right_current_state = None

		self.lgrip_sb = rospy.Subscriber("/robot/end_effector/left_gripper/state", EndEffectorState, self.lgrip_cb)
		self.rgrip_sb = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, self.rgrip_cb)
		self.getgs_sv = rospy.Service('get_gripper_state', GetGS, self.handle_getgs)

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
