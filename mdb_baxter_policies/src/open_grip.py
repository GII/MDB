#!/usr/bin/env python
import rospy
from mdb_baxter_policies.srv import OpenGrip
from mdb_common.msg import OpenGripReq
from sensor_msgs.msg import JointState
from baxter_interface.gripper import Gripper
from baxter_core_msgs.msg import EndEffectorCommand

from baxter_core_msgs.msg import EndpointState

class open_grip():
	def __init__(self):
		rospy.init_node('open_grip_server')		

		self.arm = rospy.get_param("~arm")
		self.gripper = Gripper(self.arm)

		self.angle = None
		self.arm_check = None

		self.state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.state_cb)
		self.grip_sub = rospy.Subscriber("/open_grip", OpenGripReq, self.open_grip_cb)

		## Pruebas de interpolacion ##
		#self.endpoint = None
		#self.left_point_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.left_state_cb)
		##############################
	def select_arm_angle(self, arg):
		options = {
			'left': 7,
			'right': 14,
		}
		return options[arg]	

	def state_cb(self, state):
		if self.angle != None and self.arm == self.arm_check  and self.arm+"_w1" in state.name[:] and state.position[self.select_arm_angle(self.arm)]<self.angle:
			print self.angle, self.arm, self.arm_check, state.position[self.select_arm_angle(self.arm)], self.angle
			print "opening grip "+self.arm
			self.gripper.open()
			#print self.endpoint.pose.position
			self.angle = None

	## Pruebas de interpolacion ##
	#def left_state_cb(self, state):
		#self.endpoint = state		
	##############################	

	def open_grip_cb(self, msg):
		self.angle = msg.angle
		self.arm_check = msg.arm.data

def open_grip_server():
	open_grip()
	rospy.spin()

if __name__ == "__main__":
    open_grip_server()
