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
from mdb_baxter_policies.srv import BaxThrow, BaxGrab, BaxPush, BaxGrabBoth, BaxDropBoth, GetSense, BaxCheckReach, BaxGetCompleteSense, JoystickControl
from mdb_baxter_policies.srv import BaxThrowRequest, BaxGrabRequest, BaxPushRequest, BaxGrabBothRequest, BaxDropBothRequest, BaxChangeFaceRequest, BaxCheckReachRequest, JoystickControlRequest
from std_msgs.msg import Bool, Float64, String, Int16
from mdb_common.srv import BaxChange, BaxChangeRequest
from mdb_common.msg import SensData

class policies_manager():
	def __init__(self, global_exp):		
		self.global_exp = global_exp

		self.fixed_height = rospy.get_param("~fixed_height")
		self.super_throw = rospy.get_param("~super_throw")

		self.velocity = 0.85 

		# Service Proxies
		self.bax_throw_clnt = rospy.ServiceProxy('/baxter/policy/throw', BaxThrow)  
		self.bax_grab_clnt = rospy.ServiceProxy('/baxter/policy/grab', BaxGrab) 
		self.bax_push_clnt = rospy.ServiceProxy('/baxter/policy/push', BaxPush) 
		self.bax_change_hands_clnt = rospy.ServiceProxy('/baxter/policy/change_hands', BaxChange) 
		self.bax_grab_both_clnt = rospy.ServiceProxy('/baxter/policy/grab_both', BaxGrabBoth) 
		self.bax_drop_both_clnt = rospy.ServiceProxy('/baxter/policy/drop_both', BaxDropBoth) 
		self.bax_ask_help_clnt = rospy.ServiceProxy('/baxter/policy/ask_for_help', BaxChange) 
		self.bax_joy_control_clnt = rospy.ServiceProxy('/baxter/policy/joystick', JoystickControl)
		self.bax_drop_clnt = rospy.ServiceProxy('/baxter/policy/drop', BaxGrab)

		self.bax_get_sense_clnt = rospy.ServiceProxy('/baxter/get_sense', GetSense)
		self.bax_get_complete_sense_clnt = rospy.ServiceProxy('/baxter/get_complete_sense', BaxGetCompleteSense)
		self.bax_check_far_reach_clnt = rospy.ServiceProxy('/baxter/check_far_reach', BaxCheckReach)

		# Publishers
		self.super_throwing_pub = rospy.Publisher("/baxter_throwing/command", Int16, queue_size = 1)

	def select_sweep_angle(self, arg):
		options = {
			'left': -0.3925-0.11,
			'right': 0.3925+0.11,
		}
		return options[arg]

	def choose_x_dimension(self, arg):
		options = {
			'exp_box':0.12,
			'exp_small_obj':0.03, 
			'exp_big_obj':0.07,
		}
		return options[arg]

	def choose_y_dimension(self, arg):
		options = {
			'exp_box':0.115,
			'exp_small_obj':0.025, 
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def choose_push_dist(self, arg):
		options = {
			'exp_box':0.075,
			'exp_small_obj':0.025, 
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def choose_policy_srv (self, arg):
		options = {
			'grasp_object':self.bax_grab_clnt,
			'grasp_with_two_hands':self.bax_grab_both_clnt, 
			'change_hands':self.bax_change_hands_clnt,
			'sweep_object':self.bax_push_clnt,
			'put_object_in_box':self.bax_grab_clnt,
			'put_object_in_robot':self.bax_grab_clnt,
			'throw':self.bax_throw_clnt,
			'ask_nicely':self.bax_ask_help_clnt,
			'joystick':self.bax_joy_control_clnt, 
			'drop_object':self.bd_clnt, 
		}
		return options[arg]

	def choose_policy_req (self, arg):
		options = {
			'grasp_object':BaxGrabRequest(),
			'grasp_with_two_hands':BaxGrabBothRequest(), 
			'change_hands':BaxChangeRequest(),
			'sweep_object':BaxPushRequest(),
			'put_object_in_box':BaxGrabRequest(),
			'put_object_in_robot':BaxGrabRequest(),
			'throw':BaxThrowRequest(),
			'ask_nicely':BaxChangeRequest(),
			'joystick':JoystickControlRequest(),
			'drop_object':BaxGrabRequest(),
		}
		return options[arg]

	def choose_arm (self, arg, left_grip, right_grip):
		if not left_grip and not right_grip:
			if arg>0.0:
				return "left"
			elif arg<0.0:
				return "right"
			else:
				return "left"
		elif left_grip and not right_grip:
			return "left"
		elif not left_grip and right_grip:
			return "right"
		elif left_grip and right_grip:
			return "both"

	def choose_throw_angle (self, arm, box_angle):
		if (arm=='left' and box_angle < 0.0) or (arm=='right' and box_angle > 0.0):
			return -1.0*box_angle
		else:
			return box_angle

	def choose_throw_distance (self, arm, box_angle, box_dist):
		if (arm=='left' and box_angle < 0.0) or (arm=='right' and box_angle > 0.0):
			return 1.25
		else:
			return box_dist

	def is_same_side (self, arm, box_angle):
		if (arm=='left' and box_angle < 0.0) or (arm=='right' and box_angle > 0.0):
			return False
		else:
			return True

	def control_orientation(self, global_s, policy_code):
		if policy_code == 'change_hands': 
			self.global_exp.pan_to('front', 0.1) 
		elif policy_code == 'ask_nicely' or (global_s.left_grip.data > 0.0 and global_s.right_grip.data > 0.0):
			self.global_exp.pan_to('front', 0.1) 
		elif global_s.left_grip.data > 0.0 or global_s.right_grip.data > 0.0: #Look to the box
			self.global_exp.pan_to_pos(global_s.box_sens.angle.data, 0.1)
		elif policy_code!='change_hands' and not global_s.left_grip.data > 0.0 and not global_s.right_grip.data > 0.0: #Look to the ball
			self.global_exp.pan_to_pos(global_s.obj_sens.angle.data, 0.1)

	def gripper_sense_data (self, global_s, arm):
		if arm == "left":
			return global_s.left_grip.data
		elif arm == "right":
			return global_s.right_grip.data	

	def choose_sweep_height(self, obj):
		options = {
			'exp_small_obj':0.03,
			'exp_big_obj':0.1,
		}
		return options[obj]

	def policy_grasp_object(self, policy_code, global_s, arm, srv):
		if self.global_exp.obj_type == "exp_small_obj" and self.global_exp.world == "gripper_and_low_friction":
			srv.object_position.const_dist = global_s.obj_sens.dist
			srv.object_position.angle = global_s.obj_sens.angle
			srv.object_position.height.data = self.fixed_height 
			srv.orientation.data = "current"
			srv.arm.data = arm
			srv.scale.data = self.velocity
			self.global_exp.adopt_expression("focus")
			if not self.gripper_sense_data(global_s, arm) > 0.0:
				resp = self.choose_policy_srv(policy_code)(srv).result.data
				return resp
		return False

	def policy_grasp_with_two_hands(self, policy_code, global_s, arm, srv):
		resp = False
		if (global_s.obj_sens.angle.data < 0.3925) and (global_s.obj_sens.angle.data > -0.3925) and (global_s.obj_sens.dist.data > 0.47) and (global_s.obj_sens.dist.data < 0.75) and (global_s.left_grip.data < 1.0 and global_s.right_grip.data < 1.0):
			if self.global_exp.obj_type == "exp_big_obj" and self.global_exp.world == "gripper_and_low_friction":
				srv.sensorization.const_dist = global_s.obj_sens.dist
				srv.sensorization.angle = global_s.obj_sens.angle
				srv.sensorization.height.data = self.fixed_height
				srv.size.data = 0.08
				self.global_exp.adopt_expression("focus")
				resp = self.choose_policy_srv(policy_code)(srv).result.data
			elif self.global_exp.world == "no_gripper_and_high_friction":
				srv.sensorization.const_dist = global_s.obj_sens.dist
				srv.sensorization.angle = global_s.obj_sens.angle
				srv.sensorization.height.data = self.fixed_height
				if self.global_exp.obj_type == "exp_big_obj":
					srv.size.data = 0.08
				else:
					srv.size.data = 0.06
				self.global_exp.adopt_expression("focus")
				resp = self.choose_policy_srv(policy_code)(srv).result.data
		return resp

	def policy_change_hands(self, policy_code, global_s, arm, srv):
		if self.global_exp.world == "gripper_and_low_friction":
			self.global_exp.pan_to('front', 0.1) 
			srv.request.data = True
			self.global_exp.adopt_expression("focus")
			resp = self.choose_policy_srv(policy_code)(srv).result.data
			return resp
		return False

	def policy_sweep_object(self, policy_code, global_s, arm, srv):
		resp = False
		srv.obj_sens.const_dist = global_s.obj_sens.dist
		srv.obj_sens.angle = global_s.obj_sens.angle
		srv.obj_sens.height.data = self.fixed_height+0.005+self.choose_sweep_height(self.global_exp.obj_type)
		srv.dest_sens.const_dist.data = 0.70
		if self.global_exp.obj_type == "exp_small_obj" and self.global_exp.world == "gripper_and_low_friction":
			srv.dest_sens.angle.data = self.select_sweep_angle(arm)
		else:
			srv.dest_sens.angle.data = 0.0
		srv.dest_sens.height.data = self.fixed_height+0.005+self.choose_sweep_height(self.global_exp.obj_type)
		srv.radius.data = self.choose_push_dist(self.global_exp.obj_type) + 0.02 
		srv.arm.data = arm
		srv.scale.data = self.velocity
		if self.global_exp.world == "gripper_and_low_friction":
			print "Do grip"
			srv.grip.data = True
		self.global_exp.adopt_expression("focus")
		if not arm == "both":
			resp = self.choose_policy_srv(policy_code)(srv).result.data
		if resp:
			rospy.set_param("/check_reward", True)
			self.global_exp.complete_pan_static()
			rospy.delete_param("/check_reward")
		return resp

	def policy_put_object_in (self, policy_code, global_s, arm, srv):
		resp = False
		if (global_s.left_grip.data < 1.0 or global_s.right_grip.data < 1.0) and self.global_exp.world == "gripper_and_low_friction": 
			if policy_code == 'put_object_in_box': #Destination = Box
				srv.object_position.const_dist = global_s.box_sens.dist
				srv.object_position.angle = global_s.box_sens.angle
			else: #Destination = Robot (predefined)
				srv.object_position.const_dist.data = 0.47+0.03
				srv.object_position.angle.data = 0.0
			srv.object_position.height.data = self.fixed_height+self.choose_sweep_height(self.global_exp.obj_type)
			srv.orientation.data = "current"
			srv.arm.data = arm
			srv.scale.data = self.velocity
			self.global_exp.adopt_expression("focus")
			if not (self.gripper_sense_data(global_s, arm) < 1.0 or (not self.is_same_side(arm, global_s.box_sens.angle.data) and policy_code == 'put_object_in_box')):
				resp = self.choose_policy_srv(policy_code)(srv).result.data
		elif (global_s.left_grip.data > 0.0 and global_s.right_grip.data > 0.0):
			srv = BaxDropBothRequest()
			if policy_code == 'put_object_in_box': #Destination = Box
				srv.destination.const_dist = global_s.box_sens.dist
				srv.destination.angle = global_s.box_sens.angle
			else: #Destination = Robot (predefined)
				srv.destination.const_dist.data = 0.47 + 0.08 
				srv.destination.angle.data = 0.0
			srv.destination.height.data = self.fixed_height+self.choose_sweep_height("exp_big_obj") 
			srv.size.data = 0.15
			self.global_exp.adopt_expression("focus")
			resp = self.bax_drop_both_clnt(srv).result.data
		if resp == True:
			rospy.set_param("/check_reward", True)
			self.global_exp.complete_pan_static()
			rospy.delete_param("/check_reward")
		return resp

	def policy_throw (self, policy_code, global_s, arm, srv):
		resp = False
		if self.super_throw:
			srv.arm.data = 'right'					
			self.global_exp.adopt_expression("focus")
			self.super_throwing_pub.publish(4)
			rospy.sleep(10)
			self.super_throwing_pub.publish(5)
			rospy.sleep(20)
			resp = self.choose_policy_srv(policy_code)(srv).result.data
			rospy.set_param("/check_reward", True)
			self.global_exp.complete_pan_static()
			rospy.delete_param("/check_reward")
		elif self.global_exp.obj_type == "exp_small_obj" and self.global_exp.world == "gripper_and_low_friction":
			srv.sensorization.const_dist.data = self.choose_throw_distance(arm, global_s.box_sens.angle.data, global_s.box_sens.dist.data)
			srv.sensorization.angle.data = self.choose_throw_angle(arm, global_s.box_sens.angle.data)
			srv.arm.data = arm
			self.global_exp.adopt_expression("focus")
			resp = self.choose_policy_srv(policy_code)(srv).result.data
			if resp == True:
				rospy.set_param("/check_reward", True)
				self.global_exp.complete_pan_static()
				rospy.delete_param("/check_reward")
		return resp

	def policy_ask_nicely (self, policy_code, global_s, arm, srv):
		resp = False
		self.global_exp.pan_to('front', 0.1) 
		if not ((self.gripper_sense_data(global_s,"left") > 0.0 or self.gripper_sense_data(global_s,"right") > 0.0)):
			srv.request.data = True
			self.choose_policy_srv(policy_code)(srv)
			rospy.sleep(1)
			self.global_exp.adopt_expression("normal")	
			self.global_exp.complete_pan_static()
			resp = True
		return resp

	def policy_joystick(self, policy_code, global_s, arm, srv):
		srv.joystick_pos.const_dist = 0.6
		srv.joystick_pos.angle = 0.785
		srv.joystick_pos.height.data = 0.09
		srv.joystick_angle.data = 0.0
		srv.time_to_control.data = 0.0
		srv.arm_to_move.data = arm
		srv.velocity_scale.data = 1.0
		self.global_exp.adopt_expression("focus")
		if not self.gripper_sense_data(global_s, arm) > 0.0:
			resp = self.choose_policy_srv(policy_code)(srv).result.data
			return resp
		return False

	def policy_drop_object (self, policy_code, global_s, arm, srv):
		resp = False
		srv.object_position.const_dist = global_s.box_sens.dist
		srv.object_position.angle = global_s.box_sens.angle
		srv.object_position.height.data = self.fixed_height+self.choose_sweep_height(self.obj_type)
		srv.orientation.data = "current"
		srv.arm.data = arm
		srv.scale.data = self.velocity
		self.adopt_expression("focus")
		if self.gripper_sense_data(global_s, arm) >= 1.0:
			resp = self.choose_policy(policy_code)(srv).result.data
		if resp == True:
			rospy.set_param("/check_reward", True)
			self.complete_pan()
			rospy.delete_param("/check_reward")
		return resp
		
	def choose_policy (self, arg):
		options = {
			'grasp_object':self.policy_grasp_object,
			'grasp_with_two_hands':self.policy_grasp_with_two_hands, 
			'change_hands':self.policy_change_hands,
			'sweep_object':self.policy_sweep_object,
			'put_object_in_box':self.policy_put_object_in,
			'put_object_in_robot':self.policy_put_object_in,
			'throw':self.policy_throw,
			'ask_nicely':self.policy_ask_nicely,
			'joystick':self.policy_joystick,
			'drop_object':self.policy_drop_object, 
		}
		return options[arg]		

	def execute_policy(self, policy_code):
		global_s = self.bax_get_sense_clnt(Bool(True))
		arm = self.choose_arm(global_s.obj_sens.angle.data, self.gripper_sense_data(global_s,"left") > 0.0, self.gripper_sense_data(global_s,"right") > 0.0 )
		srv = self.choose_policy_req(policy_code)
		resp =  None
		#self.control_orientation(global_s, policy_code)

		rospy.loginfo("Iteration %s > Executing policy %s with the %s arm", self.global_exp.exp_iteration, policy_code, arm)
		self.global_exp.exp_iteration+=1

		#Execute policy
		resp = self.choose_policy(policy_code)(policy_code, global_s, arm, srv)

		#Publish the next sensorization
		rospy.loginfo("Publishing sensorization")
		self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_x_dimension('exp_box')), Float64(self.choose_x_dimension(self.global_exp.obj_type)))

		#If it failed
		if not resp:
			self.global_exp.adopt_expression("confused")
			rospy.sleep(2)

		#Standard face
		self.global_exp.adopt_expression("normal")
		rospy.loginfo("Success? : %s", resp)
		return resp
