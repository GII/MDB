#!/usr/bin/env python
import rospy

class test_policies():
	def __init__(self, global_policies):
		self.global_policies = global_policies

		self.test_code()

	def test_code(self):
		#self.global_policies.baxter_arm.AddElements()
		#self.global_policies.baxter_arm.GraspCreation()
		#self.global_policies.baxter_arm.pickup("small_obj", self.global_policies.baxter_arm.grasps)

		#left_test_angles = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673] #moveit
		#left_test_angles_2 = [0.5565567001634442, 1.9541028505160654, 1.3091857254483799, -0.9385845547335236, -0.6373815959474021, 2.0937221905141428, 1.5304675427922119] #baxter

		#left_test_speed = [2.0, 6.0, 2.0, 4.0, 4.0, 0.0, 0.0]
		#left_test_acceleration = [0.3, 0.3, 0.3, 0.7, 0.7, 0.7, 0.7]
		#self.baxter_arm.move_joints_raw_position(left_test_angles_2, left_test_speed, left_test_acceleration, 'baxter', 'left', True, 5)

		#self.baxter_arm.move_joints_command(4, left_test_angles_2, 'left', 'baxter')
		#self.baxter_arm.move_joints_interface('left', left_test_angles, 0.05, 'moveit')
