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

import rospy, cv2, re, math, vlc, rospkg
import numpy as np
from baxter_core_msgs.msg import HeadPanCommand, HeadState
from sensor_msgs.msg import Image
from mdb_baxter_policies.srv import BaxThrow, BaxPush, BaxGrabBoth, BaxDropBoth, BaxGrab, BaxRestoreArmPose, BaxChangeFace, GetSense, BaxCheckReach, BaxGetCompleteSense, ManagePlanScene
from mdb_baxter_policies.srv import BaxThrowRequest, BaxPushRequest, BaxGrabBothRequest, BaxDropBothRequest, BaxGrabRequest, BaxRestoreArmPoseRequest, BaxChangeFaceRequest, GetSenseRequest, BaxGetCompleteSenseRequest, BaxCheckReachRequest, ManagePlanSceneRequest
from std_msgs.msg import Bool, Float64, String, Int16
from mdb_common.srv import BaxChange, ExecPolicy, RefreshWorld, NewExperiment, BaxChangeRequest
from mdb_baxter_experiments.srv import SimMng, SimMngRequest
from mdb_common.msg import SensData, ControlMsg

class exp_ltm():
	def __init__(self):
		self.head_state = None
		self.rospack = rospkg.RosPack()
		self.afile = vlc.MediaPlayer(self.rospack.get_path('mdb_baxter_experiments')+"/audio/TaDa.mp3");
		self.afile_r = vlc.MediaPlayer(self.rospack.get_path('mdb_baxter_experiments')+"/audio/inceptionhorn.mp3");

		self.exp_iteration=1
		self.world = None

		self.ball_pos = None
		self.box_pos = None
		self.obj_type = None

		self.mode = rospy.get_param("~mode")
		self.exp_type = rospy.get_param("~exp_type")
		self.config_time = rospy.get_param("~config_time")
		self.fixed_height = rospy.get_param("~fixed_height")
		self.super_throw = rospy.get_param("~super_throw")

		self.velocity = self.select_velocity()

		### ROS Service Clients ###
		#policies#
		self.bax_throw_clnt = rospy.ServiceProxy('/baxter/policy/throw', BaxThrow)  
		self.bax_grab_clnt = rospy.ServiceProxy('/baxter/policy/grab', BaxGrab) 
		self.bax_push_clnt = rospy.ServiceProxy('/baxter/policy/push', BaxPush) 
		self.bax_change_hands_clnt = rospy.ServiceProxy('/baxter/policy/change_hands', BaxChange) 
		self.bax_grab_both_clnt = rospy.ServiceProxy('/baxter/policy/grab_both', BaxGrabBoth) 
		self.bax_drop_both_clnt = rospy.ServiceProxy('/baxter/policy/drop_both', BaxDropBoth) 
		self.bax_ask_help_clnt = rospy.ServiceProxy('/baxter/policy/ask_for_help', BaxChange) 
		self.bax_restore_arm_pose_clnt = rospy.ServiceProxy('/baxter/restore', BaxRestoreArmPose)
		self.bax_change_face_clnt = rospy.ServiceProxy('/baxter/change_face', BaxChangeFace)
		self.bax_operation_pose_clnt = rospy.ServiceProxy('/baxter/operation_pose', BaxChange)
		self.bax_open_arms_pose_clnt = rospy.ServiceProxy('/baxter/open_arms_pose', BaxChange)
		#sensorization#
		self.bax_get_sense_clnt = rospy.ServiceProxy('/baxter/get_sense', GetSense)
		self.bax_get_complete_sense_clnt = rospy.ServiceProxy('/baxter/get_complete_sense', BaxGetCompleteSense)
		#scene#
		self.scene_clnt = rospy.ServiceProxy('/mdb3/baxter/modify_planning_scene', ManagePlanScene)
		##SIM##
		#policy#
		self.bax_reset_grippers_clnt = rospy.ServiceProxy('/baxter/reset_gippers', BaxChange)
		#simulation_management#
		self.load_srv = rospy.ServiceProxy('/sim/create_obj', SimMng)
		self.modify_srv = rospy.ServiceProxy('/sim/modify_obj', SimMng)
		self.delete_srv = rospy.ServiceProxy('/sim/delete_obj', SimMng)
		#check#
		self.bax_check_close_reach_clnt = rospy.ServiceProxy('/baxter/check_close_reach', BaxCheckReach)
		self.bax_check_far_reach_clnt = rospy.ServiceProxy('/baxter/check_far_reach', BaxCheckReach)

		### ROS Service Servers ###
		if self.mode == 'real':
			self.new_exp_srver = rospy.Service("/mdb/baxter/new_experiment", NewExperiment, self.handle_new_exp_real)
			self.ref_world_srver = rospy.Service("/mdb/baxter/refresh_world", RefreshWorld, self.handle_ref_world_real)
		else:
			self.new_exp_srver = rospy.Service("/mdb/baxter/new_experiment", NewExperiment, self.handle_new_exp)
			self.ref_world_srver = rospy.Service("/mdb/baxter/refresh_world", RefreshWorld, self.handle_ref_world)

		self.exec_pol_srver = rospy.Service("/mdb/baxter/exec_policy", ExecPolicy, self.handle_exec_pol)
		rospy.sleep(1)

		self.scene_clnt(String('head_collision'),String('add'),Float64(0.0), Float64(0.0), Float64(0.0))
		self.scene_clnt(String('table'),String('remove'),Float64(0.0), Float64(0.0), Float64(0.0))

		### ROS Publishers ###
		self.pan_pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size = 1)
		self.super_throwing_pub = rospy.Publisher("/baxter_throwing/command", Int16, queue_size = 1)

		### ROS Subscribers ###
		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.control_sub = rospy.Subscriber("/mdb/baxter/control", ControlMsg, self.control_cb)
		#self.executed_policy_sub = rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_cb)
	
	def control_cb (self, control_msg):
		self.world = control_msg.world
		self.pan_to('front', 0.1)
		if control_msg.reward == True:
			self.reward_sound()
		else:
			self.afile_r.play()
			rospy.sleep(3)
			self.afile_r.stop()
			self.bax_restore_arm_pose_clnt(String('both'))
			self.bax_reset_grippers_clnt(Bool(True), Bool(True))
		self.refresh_real()
		try:
			self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
		except rospy.ServiceException, e:
			rospy.loginfo("Sense service call failed: {0}".format(e))

	def executed_policy_cb (self, policy):
		resp = self.execute_policy(policy.data)		

	def select_velocity(self):
		if self.mode == "sim":
			return 0.5
		else:
			return 0.85
		

	def select_angle(self, arg):
		options = {
			'left': -0.3925-0.11,
			'right': 0.3925+0.11,
		}
		return options[arg]

	def choose_xd(self, arg):
		options = {
			'exp_box':0.12,
			'exp_small_obj':0.03, 
			'exp_big_obj':0.07,
		}
		return options[arg]

	def choose_push_dist(self, arg):
		options = {
			'exp_box':0.075,
			'exp_small_obj':0.025, 
			'exp_big_obj':0.0675,
		}
		return options[arg]


	def choose_yd(self, arg):
		options = {
			'exp_box':0.115,
			'exp_small_obj':0.025, 
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def translate_c2p (self, x, y):
		dist = np.sqrt((x**2) + (y**2))
		angle = np.arctan(y/x)
		return dist, angle

	def translate_p2c (self, dist, angle):
		dx = dist*np.cos(angle)
		dy = dist*np.sin(angle)
		return dx, dy

	def randomize_obj (self):
		if self.exp_type == "big":
			return "exp_big_obj"		
		elif self.exp_type == "small":
			return "exp_small_obj"
		elif np.random.uniform()>0.3:
			return "exp_small_obj"
		else:
			return "exp_big_obj"

	def check_box_ball (self, obj, box_x, box_y, ball_x, ball_y):
		print "check_box_ball: ", box_x, box_y, ball_x, ball_y
		if box_x+(self.choose_xd("exp_box")*100) < ball_x-(self.choose_xd(obj)*100) or box_x-(self.choose_xd("exp_box")*100) > ball_x+(self.choose_xd(obj)*100) or box_y+(self.choose_yd("exp_box")*100) < ball_y-(self.choose_yd(obj)*100) or box_y-(self.choose_yd("exp_box")*100) > ball_y+(self.choose_yd(obj)*100):
			return True
		else:
			return False
		
	def randomize_positions(self, obj):
		table_dim_x_min = 0.22
		table_dim_x_max = 0.22 + 1.22
		table_dim_y_min = 0.0		
		table_dim_y_max = 2.442

		ball_x = None
		ball_y = None
		ball_valid = False
		while not ball_valid:
			ball_x = np.random.randint(low=(table_dim_x_min+self.choose_xd(obj))*100, high=(table_dim_x_max-self.choose_xd(obj))*100)
			ball_y = np.random.randint(low=(table_dim_y_min+self.choose_yd(obj))*100, high=(table_dim_y_max-self.choose_yd(obj))*100)
			dist, angle = self.translate_c2p(ball_x*0.01, (ball_y*0.01)-1.221)
			if self.bax_check_close_reach_clnt(BaxCheckReachRequest(Float64(dist), Float64(angle), String(obj))).response.data:
				ball_valid = True
				self.ball_pos= [dist, angle]
					

		box_valid = False
		while not box_valid:
			if obj == "exp_big_obj":
				angle = np.random.randint(low=-39, high=39)
				dist = np.random.randint(low=47, high=75)
				box_x, box_y = self.translate_p2c(dist*0.01, angle*0.01)
				print angle*0.01, dist*0.01, box_x, box_y
				if self.check_box_ball(obj, box_x*100, box_y*100, ball_x, ball_y):
					box_valid = True
					self.box_pos = [dist*0.01, angle*0.01]
			else:
				box_x = np.random.randint(low=(table_dim_x_min+self.choose_xd('exp_box'))*100, high=(table_dim_x_max-self.choose_xd('exp_box'))*100)
				box_y = np.random.randint(low=(table_dim_y_min+self.choose_yd('exp_box'))*100, high=(table_dim_y_max-self.choose_yd('exp_box'))*100)
				dist, angle = self.translate_c2p(box_x*0.01, (box_y*0.01)-1.221)
				if self.bax_check_close_reach_clnt(BaxCheckReachRequest(Float64(dist), Float64(angle), String(obj))).response.data and self.check_box_ball(obj, box_x, box_y, ball_x, ball_y):
					box_valid = True
					self.box_pos = [dist, angle]
		return True	

	def choose_world_image(self):
		if self.obj_type == "exp_small_obj" and self.world == "gripper_and_low_friction":
			self.adopt_expression("small_obj_grip")
		elif self.obj_type == "exp_small_obj" and self.world == "no_gripper_and_high_friction":
			self.adopt_expression("small_obj_nogrip") 	 						
		elif self.obj_type == "exp_big_obj" and self.world == "gripper_and_low_friction":
			self.adopt_expression("big_obj_grip") 	
		elif self.obj_type == "exp_big_obj" and self.world == "no_gripper_and_high_friction":
			self.adopt_expression("big_obj_nogrip") 	

	def refresh_real(self):
		if rospy.has_param("/baxter_sense"): rospy.delete_param("/baxter_sense")
		if rospy.has_param("/check_reward"): rospy.delete_param("/check_reward")
		#self.pan_to('front', 0.1)
		self.bax_reset_grippers_clnt(Bool(True), Bool(True))
		self.obj_type = self.randomize_obj()
		self.choose_world_image()
		#rospy.sleep(self.config_time)
		raw_input("Press Key after configuring the experiment")
		print "Starting experiment"
		self.bax_restore_arm_pose_clnt(String('both'))
		self.adopt_expression("normal") 
		#self.adopt_operation_pose()
		#self.pan_to('front', 0.1)
		self.complete_pan()

	def handle_new_exp_real(self, req):
		self.world = req.world.data
		self.refresh_real()
		try:
			self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Sense service call failed: {0}".format(e))
			return Bool(False)
				
	def handle_new_exp(self, req):
		self.world = req.world.data

		self.obj_type =  self.randomize_obj()
		self.randomize_positions(self.obj_type)

		try:
			self.load_srv(SimMngRequest(model_name=String('exp_table')))
			self.load_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=Float64(self.ball_pos[0]), angle=Float64(self.ball_pos[1]), height=Float64(0.0))))
			self.load_srv(SimMngRequest(model_name=String('exp_box'), sense=SensData(dist=Float64(self.box_pos[0]), angle=Float64(self.box_pos[1]), height=Float64(0.0))))
			self.bax_reset_grippers_clnt(Bool(True), Bool(True))
			self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Delete Model service call failed: {0}".format(e))
			return Bool(False)

	def refresh_world (self):
		try:
			self.delete_srv(SimMngRequest(model_name=String(self.obj_type)))
			self.delete_srv(SimMngRequest(model_name=String('exp_box')))
		except rospy.ServiceException, e:
			rospy.loginfo("Delete Model service call failed: {0}".format(e))
			return False

		self.obj_type =  self.randomize_obj()
		self.randomize_positions(self.obj_type)

		try:
			self.load_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=Float64(self.ball_pos[0]), angle=Float64(self.ball_pos[1]), height=Float64(0.0))))
			self.load_srv(SimMngRequest(model_name=String('exp_box'), sense=SensData(dist=Float64(self.box_pos[0]), angle=Float64(self.box_pos[1]), height=Float64(0.0))))
			self.bax_reset_grippers_clnt(Bool(True), Bool(True))
			return True
		except rospy.ServiceException, e:
			rospy.loginfo("Delete Model service call failed: {0}".format(e))	
			return False

	def handle_ref_world(self, req):
		self.world = req.world.data
		if self.refresh_world():
			self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			return Bool(True)
		else:
			return Bool(False)

	def handle_ref_world_real(self, req):
		self.world = req.world.data
		self.pan_to('front', 0.1)
		if req.reward.data == True:
			self.reward_sound()
		else:
			self.afile_r.play()
			rospy.sleep(3)
			self.afile_r.stop()
			self.bax_restore_arm_pose_clnt(String('both'))
			self.bax_reset_grippers_clnt(Bool(True), Bool(True))
		self.refresh_real()
		try:
			self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Sense service call failed: {0}".format(e))
			return Bool(False)

	def handle_exec_pol (self, req):
		resp = self.execute_policy(req.policy_code.data)
		return Bool(resp)

	def head_state_cb(self, state):
		self.head_state = state

	def choose_policy (self, arg):
		options = {
			'grasp_object':self.bax_grab_clnt,
			'grasp_with_two_hands':self.bax_grab_both_clnt, 
			'change_hands':self.bax_change_hands_clnt,
			'sweep_object':self.bax_push_clnt,
			'put_object_in_box':self.bax_grab_clnt,
			'put_object_in_robot':self.bax_grab_clnt,
			'throw':self.bax_throw_clnt,
			'ask_nicely':self.bax_ask_help_clnt,
			'flexsim':self.bax_grab_clnt, 
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
			'flexsim':BaxGrabRequest(),
		}
		return options[arg]

	'''def choose_arm (self, arg):
		if arg>0:
			return "left"
		elif arg<0:
			return "right"
		else:
			return "left" #TODO:Aqui necesitaria ver por el log realmente cual es el brazo a utilizar atendiendo al estado del gripper antes y despues, etc.'''

	def adapt_sens(self, sens):
		return [sens[0]/100.0, -math.radians(sens[1]), sens[2]]

	def complete_pan_head (self):
		self.adopt_open_pose()
		rospy.set_param("/baxter_sense", True)
		self.adopt_expression("normal")
		self.pan_to('front', 0.1) 
		#Move head to the left 
		'''self.adopt_expression("horizon")
		self.pan_to('left', 0.07) 
		self.adopt_expression("normal")
		rospy.sleep(1)
		#Move head to the right
		self.adopt_expression("horizon2")
		self.pan_to('right', 0.07)
		self.adopt_expression("normal")
		rospy.sleep(1)
		#Move head to the center
		self.adopt_expression("horizon")
		self.pan_to('front', 0.07)
		self.adopt_expression("normal")'''
		rospy.sleep(1)
		rospy.delete_param("/baxter_sense")
		rospy.sleep(3)

		#self.adopt_operation_pose()
		self.bax_restore_arm_pose_clnt(String('both'))

	def complete_pan (self):
		self.adopt_open_pose()
		rospy.set_param("/baxter_sense", True)
		self.adopt_expression("normal")
		#self.pan_to('front', 0.1) 
		#Move head to the left 
		self.adopt_expression("horizon")
		rospy.sleep(1)
		#Move head to the right
		self.adopt_expression("horizon2")
		rospy.sleep(1)
		#Move head to the center
		self.adopt_expression("horizon")
		rospy.sleep(1)
		rospy.delete_param("/baxter_sense")
		#Move head to the right
		self.adopt_expression("horizon2")
		rospy.sleep(1)
		#Move head to the center
		self.adopt_expression("horizon")
		#rospy.sleep(1)
		rospy.sleep(3)
		self.adopt_expression("normal")
		#self.adopt_operation_pose()
		self.bax_restore_arm_pose_clnt(String('both'))

	def adopt_operation_pose (self):
		req = BaxChangeRequest()
		req.request.data = True
		self.bax_operation_pose_clnt(req)

	def adopt_open_pose(self):
		req = BaxChangeRequest()
		req.request.data = True
		self.bax_open_arms_pose_clnt(req)

	def adopt_expression(self, expr):
		req = BaxChangeFaceRequest()
		req.expression.data = expr
		self.bax_change_face_clnt(req)

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

	def same_side (self, arm, box_angle):
		print "same_side: ", arm, box_angle
		if (arm=='left' and box_angle < 0.0) or (arm=='right' and box_angle > 0.0):
			return False
		else:
			return True

	def control_orientation(self, global_s, policy_code):
		if policy_code == 'change_hands': #Look to the front
			self.pan_to('front', 0.1) 
		elif policy_code == 'ask_nicely' or (global_s.left_grip.data > 0.0 and global_s.right_grip.data > 0.0):
			self.pan_to('front', 0.1) 
		elif global_s.left_grip.data > 0.0 or global_s.right_grip.data > 0.0: #Look to the box
			self.pan_to_sp(global_s.box_sens.angle.data, 0.1)
		elif policy_code!='change_hands' and not global_s.left_grip.data > 0.0 and not global_s.right_grip.data > 0.0: #Look to the ball
			self.pan_to_sp(global_s.obj_sens.angle.data, 0.1)

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

	#TODO: Improve the height management
	def execute_policy(self, policy_code):
		global_s = self.bax_get_sense_clnt(Bool(True))
		arm = self.choose_arm(global_s.obj_sens.angle.data, self.gripper_sense_data(global_s,"left") > 0.0, self.gripper_sense_data(global_s,"right") > 0.0 )
		srv = self.choose_policy_req(policy_code)
		resp =  None
		#self.control_orientation(global_s, policy_code)

		rospy.loginfo("Iteration %s > Executing policy %s with the %s arm", self.exp_iteration, policy_code, arm)
		self.exp_iteration+=1

		if policy_code == 'grasp_object':
			if self.obj_type == "exp_small_obj" and self.world == "gripper_and_low_friction":
				#Object sensorization 
				srv.object_position.const_dist = global_s.obj_sens.dist
				srv.object_position.angle = global_s.obj_sens.angle
				srv.object_position.height.data = self.fixed_height 
				#Rest of the service request
				srv.orientation.data = "current"
				srv.arm.data = arm
				srv.scale.data = self.velocity
				#Focused face
				self.adopt_expression("focus")
				#Execute policy
				if self.gripper_sense_data(global_s, arm) > 0.0:
					resp = False
				else:
					resp = self.choose_policy(policy_code)(srv).result.data
			else:
				resp = False

		if policy_code == 'grasp_with_two_hands':
			if (global_s.obj_sens.angle.data < 0.3925) and (global_s.obj_sens.angle.data > -0.3925) and (global_s.obj_sens.dist.data > 0.47) and (global_s.obj_sens.dist.data < 0.75) and (global_s.left_grip.data < 1.0 and global_s.right_grip.data < 1.0):
				if self.obj_type == "exp_big_obj" and self.world == "gripper_and_low_friction":
					#Object sensorization:
					srv.sensorization.const_dist = global_s.obj_sens.dist
					srv.sensorization.angle = global_s.obj_sens.angle
					srv.sensorization.height.data = self.fixed_height
					#Rest of the service request
					srv.size.data = 0.08
					#Focused face
					self.adopt_expression("focus")
					#Execute policy
					resp = self.choose_policy(policy_code)(srv).result.data
				elif self.world == "no_gripper_and_high_friction":
					#Object sensorization:
					srv.sensorization.const_dist = global_s.obj_sens.dist
					srv.sensorization.angle = global_s.obj_sens.angle
					srv.sensorization.height.data = self.fixed_height
					#Rest of the service request
					if self.obj_type == "exp_big_obj":
						srv.size.data = 0.08
					else:
						srv.size.data = 0.06
					#Focused face
					self.adopt_expression("focus")
					#Execute policy
					resp = self.choose_policy(policy_code)(srv).result.data
				else:
					resp = False
			else:
				resp = False

		if policy_code == 'change_hands':
			if self.world == "gripper_and_low_friction":
				#Look to the front
				self.pan_to('front', 0.1) 
				#Service request
				srv.request.data = True
				#Focused face
				self.adopt_expression("focus")
				#Execute policy
				resp = self.choose_policy(policy_code)(srv).result.data
			else:
				resp = False

		if policy_code == 'sweep_object':
			#Object sensorization
			srv.obj_sens.const_dist = global_s.obj_sens.dist
			srv.obj_sens.angle = global_s.obj_sens.angle
			srv.obj_sens.height.data = self.fixed_height+0.005+self.choose_sweep_height(self.obj_type)
			#Dest destination
			srv.dest_sens.const_dist.data = 0.70
			if self.obj_type == "exp_small_obj" and self.world == "gripper_and_low_friction":
				srv.dest_sens.angle.data = self.select_angle(arm)
			else:
				srv.dest_sens.angle.data = 0.0
			srv.dest_sens.height.data = self.fixed_height+0.005+self.choose_sweep_height(self.obj_type)
			#Rest of the service request
			srv.radius.data = self.choose_push_dist(self.obj_type) + 0.02 #global_s.obj_sens.radius.data
			srv.arm.data = arm
			srv.scale.data = self.velocity
			print self.mode
			if self.world == "gripper_and_low_friction":
				print "Do grip"
				srv.grip.data = True
			#Focused face
			self.adopt_expression("focus")
			#Execute policy
			if arm == "both":
				resp =  False
			else:
				resp = self.choose_policy(policy_code)(srv).result.data
			if self.mode == "sim" and resp:
				self.modify_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=srv.dest_sens.const_dist, angle=srv.dest_sens.angle, height=Float64(0.0))))
			if self.mode=="real" and resp:
				rospy.set_param("/check_reward", True)
				self.complete_pan()
				rospy.delete_param("/check_reward")

		if policy_code == 'put_object_in_box' or policy_code == 'put_object_in_robot': 
			if (global_s.left_grip.data < 1.0 or global_s.right_grip.data < 1.0) and self.world == "gripper_and_low_friction": ###Single
				if policy_code == 'put_object_in_box': #Destination = Box
					srv.object_position.const_dist = global_s.box_sens.dist
					srv.object_position.angle = global_s.box_sens.angle
				else: #Destination = front, predefined
					srv.object_position.const_dist.data = 0.47+0.03
					srv.object_position.angle.data = 0.0
				srv.object_position.height.data = self.fixed_height+self.choose_sweep_height(self.obj_type)

				#Rest of the service request
				srv.orientation.data = "current"
				srv.arm.data = arm
				srv.scale.data = self.velocity
				#Focused face
				self.adopt_expression("focus")

				#Execute policy
				if self.gripper_sense_data(global_s, arm) < 1.0 or (not self.same_side(arm, global_s.box_sens.angle.data) and policy_code == 'put_object_in_box'):
					resp = False
				else:
					resp = self.choose_policy(policy_code)(srv).result.data

			elif (global_s.left_grip.data > 0.0 and global_s.right_grip.data > 0.0): ###Double
				srv = BaxDropBothRequest()
				if policy_code == 'put_object_in_box': #Destination = Box
					srv.destination.const_dist = global_s.box_sens.dist
					srv.destination.angle = global_s.box_sens.angle
				else: #Destination = front, predefined
					srv.destination.const_dist.data = 0.47 + 0.08 #self.choose_xd(self.obj_type) #0.47+0.08
					srv.destination.angle.data = 0.0
				srv.destination.height.data = self.fixed_height+self.choose_sweep_height("exp_big_obj") #(self.obj_type)
				#Rest of the service request
				srv.size.data = 0.15
				#Focused face
				self.adopt_expression("focus")
				#Execute policy
				resp = self.bax_drop_both_clnt(srv).result.data
				
			else:
				resp = False
			if self.mode=="real" and resp == True:
				rospy.set_param("/check_reward", True)
				self.complete_pan()
				rospy.delete_param("/check_reward")

		if policy_code == 'throw': 
			if self.super_throw:
				srv.arm.data = 'right'					
				self.adopt_expression("focus")
				self.super_throwing_pub.publish(4)
				rospy.sleep(10)
				self.super_throwing_pub.publish(5)
				rospy.sleep(20)
				resp = self.choose_policy(policy_code)(srv).result.data
				rospy.set_param("/check_reward", True)
				self.complete_pan()
				rospy.delete_param("/check_reward")
			else:
				if self.obj_type == "exp_small_obj" and self.world == "gripper_and_low_friction":
					if self.mode == 'sim':
						if global_s.obj_sens.height.data > 0.0:
							if not self.bax_check_far_reach_clnt(BaxCheckReachRequest(global_s.box_sens.dist, global_s.box_sens.angle, String(self.obj_type))).response.data and self.same_side(arm, global_s.box_sens.angle.data):
								self.modify_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=global_s.box_sens.dist, angle=global_s.box_sens.angle, height=Float64(0.005))))
							else:
								self.modify_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=Float64(global_s.box_sens.dist.data+0.35), angle=Float64(self.choose_throw_angle(arm, global_s.box_sens.angle.data)), height=Float64(0.0))))
							rospy.sleep(1)
							self.bax_reset_grippers_clnt(Bool(True), Bool(True))
							resp = True
						else:
							resp = False
					else:
						#Destination
						srv.sensorization.const_dist.data = self.choose_throw_distance(arm, global_s.box_sens.angle.data, global_s.box_sens.dist.data)
						srv.sensorization.angle.data = self.choose_throw_angle(arm, global_s.box_sens.angle.data)
						#Rest of the service request
						srv.arm.data = arm
						#Focused face
						self.adopt_expression("focus")
						#Execute policy
						resp = self.choose_policy(policy_code)(srv).result.data
						if resp == True:
							rospy.set_param("/check_reward", True)
							self.complete_pan()
							rospy.delete_param("/check_reward")
				else:
					resp = False
			
		if policy_code == 'ask_nicely': 
			#Look to the front
			self.pan_to('front', 0.1) 
			if (self.gripper_sense_data(global_s,"left") > 0.0 or self.gripper_sense_data(global_s,"right") > 0.0):
				resp = False
			else:
				#Service request
				if self.mode=="real":
					srv.request.data = True
					#Execute policy
					self.choose_policy(policy_code)(srv)
				if self.mode=="sim":				
					self.modify_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=Float64(global_s.obj_sens.dist.data-0.1), angle=global_s.obj_sens.angle, height=Float64(0.0))))
					rospy.sleep(1)
				#Standard face
				self.adopt_expression("normal")	
				#Complete pan
				if self.mode=="real":
					self.complete_pan()
				resp = True

		if policy_code == 'flexsim':
			self.adopt_open_pose()
			self.bax_restore_arm_pose_clnt(String('left'))

			srv = self.choose_policy_req('grasp_object')
			srv.object_position.const_dist.data = 0.6
			srv.object_position.angle.data = 1.2
			srv.object_position.height.data = -0.02
			srv.orientation.data = "current"
			srv.arm.data = "left"
			srv.scale.data = self.velocity
			#Focused face
			self.adopt_expression("focus")
			#Execute policy
			resp = self.choose_policy(policy_code)(srv).result.data

			srv = self.choose_policy_req('put_object_in_robot')
			srv.object_position.const_dist = 0.7
			srv.object_position.angle.data = 0.0
			srv.object_position.height.data = -0.02
			srv.orientation.data = "current"
			srv.arm.data = 'left'
			srv.scale.data = self.velocity
			#Focused face
			self.adopt_expression("focus")
			resp = self.choose_policy(policy_code)(srv).result.data

		#Publish de next sensorization
		rospy.loginfo("Publishing sensorization")
		self.bax_get_complete_sense_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))

		#If it failed
		if not resp:
			self.adopt_expression("confused")
			rospy.sleep(2)

		#Standard face
		self.adopt_expression("normal")
		rospy.loginfo("Success? : %s", resp)
		return resp

	def pan_selection (self, arg):
		pan = {
			'left':0.785,
			'right':-0.785,
			'front':0.0,
		}
		return pan[arg]
		
	def pan_to (self, pos, speed):			
		self.pan_pub.publish(HeadPanCommand(self.pan_selection(pos), speed, 0.0))
		rospy.sleep(0.5)
		while self.head_state.isTurning:
			pass	

	def pan_to_sp (self, pos, speed):
		self.pan_pub.publish(HeadPanCommand(pos, speed, 0.0))
		rospy.sleep(0.5)
		while self.head_state.isTurning:
			pass	

	def reward_sound (self):
		self.afile.play()
		self.adopt_expression("rewardw")
		rospy.sleep(0.5)
		self.adopt_expression("rewardb")
		rospy.sleep(0.5)
		self.adopt_expression("rewardw")
		rospy.sleep(0.5)
		self.adopt_expression("rewardb")
		rospy.sleep(0.5)
		self.adopt_expression("rewardw")
		rospy.sleep(0.5)
		self.adopt_expression("rewardb")
		rospy.sleep(0.5)
		self.afile.stop()

def main():
	ltme = exp_ltm()
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("exp_ltm")
	main()	

