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

import rospy, tf
from shape_msgs.msg import SolidPrimitive

class test_policies():
	def __init__(self, global_policies):
		self.global_policies = global_policies

	def test_code(self):
		#self.global_policies.baxter_arm.AddElements()
		#self.global_policies.baxter_arm.GraspCreation()
		#self.global_policies.baxter_arm.pickup("small_obj", self.global_policies.baxter_arm.grasps)

		#left_test_angles = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673] #moveit
		#left_test_angles_2 = [0.5565567001634442, 1.9541028505160654, 1.3091857254483799, -0.9385845547335236, -0.6373815959474021, 2.0937221905141428, 1.5304675427922119] #baxter

		#left_test_speed = [2.0, 6.0, 2.0, 4.0, 4.0, 0.0, 0.0]
		#left_test_acceleration = [0.3, 0.3, 0.3, 0.7, 0.7, 0.7, 0.7]
		#self.global_policies.baxter_arm.move_joints_raw_position(left_test_angles_2, left_test_speed, left_test_acceleration, 'baxter', 'left', True, 5)

		#self.global_policies.baxter_arm.move_joints_command(4, left_test_angles_2, 'left', 'baxter')
		#self.global_policies.baxter_arm.move_joints_interface('left', left_test_angles, 0.05, 'moveit')

	def test_position_constraint(self):
		position_constraints = []
		# print "End effector position: ", self.global_policies.baxter_arm.larm_group.get_current_pose('left_gripper')

		pos = self.global_policies.baxter_arm.larm_state.current_es.pose.position
		ori = self.global_policies.baxter_arm.larm_state.current_es.pose.orientation
		pcr = self.global_policies.baxter_arm.position_constraint_region(SolidPrimitive.BOX, [1.22 + 0.22 + 1.0, 2.442, 0.20])
		pcp = self.global_policies.baxter_arm.position_constraint_pose([1.0, 0.83, 0.0, -0.05 + 0.0095 + pos.z])
		# pcp = self.global_policies.baxter_arm.position_constraint_pose([ori.w, pos.x, pos.y, pos.z])
		print pos, ori

		pc = self.global_policies.baxter_arm.position_constraints([pcr],[pcp], 1.0,'left')
		self.global_policies.baxter_arm.add_position_constraints([pc], 'left')

		# print self.global_policies.baxter_arm.larm_group.get_current_joint_values()

		# self.global_policies.baxter_arm.move_joints_directly(pose, 'moveit', 'left', True, 1.0)
		self.global_policies.baxter_arm.larm_group.set_start_state_to_current_state()
		self.global_policies.baxter_arm.larm_group.set_planning_time(100.0)

	def test_orientation_constraint(self):
		ori = self.global_policies.baxter_arm.rarm_group.get_current_rpy()
		quat = tf.transformations.quaternion_from_euler(ori[0], ori[1], ori[2])

		oc = self.global_policies.baxter_arm.orientation_constraint([3.14, 3.14, 3.14], quat, 1.0, 'left')
		self.global_policies.baxter_arm.add_orientation_constraints([oc],'left')

		self.global_policies.baxter_arm.larm_group.set_planning_time(120.0)
