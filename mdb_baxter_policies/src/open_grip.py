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
from mdb_common.msg import OpenGripReq
from sensor_msgs.msg import JointState
from baxter_interface.gripper import Gripper

class open_grip():
	def __init__(self):
		rospy.init_node('open_grip_server')		

		self.arm = rospy.get_param("~arm")
		self.gripper = Gripper(self.arm)

		self.angle = None
		self.arm_check = None

		self.state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.state_cb)
		self.grip_sub = rospy.Subscriber("/open_grip", OpenGripReq, self.open_grip_cb)

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
			self.angle = None

	def open_grip_cb(self, msg):
		self.angle = msg.angle
		self.arm_check = msg.arm.data

def open_grip_server():
	open_grip()
	rospy.spin()

if __name__ == "__main__":
    open_grip_server()
