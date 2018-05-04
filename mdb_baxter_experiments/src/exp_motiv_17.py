#!/usr/bin/env python
import rospy, cv2, re, math, vlc, rospkg
import numpy as np
from baxter_core_msgs.msg import HeadPanCommand, HeadState
from sensor_msgs.msg import Image
from mdb_baxter_policies.srv import BaxThrow, BaxP, BaxChange, BaxGB, BaxDB, BaxG, PickAdj, BaxRAP, BaxCF, GetSense, BCheckR, BCheckRRequest, BaxSense
from mdb_baxter_policies.srv import BaxThrowRequest, BaxPRequest, BaxChangeRequest, BaxGBRequest, BaxDBRequest, BaxGRequest, PickAdjRequest, BaxRAPRequest, BaxCFRequest, GetSenseRequest, BaxSenseRequest
from std_msgs.msg import Bool, Float64, String
from exp_scene import *
from mdb_msgs.srv import ExecPolicy, NewExperiment, RefreshWorld
from mdb_baxter_experiments.srv import SimMng, SimMngRequest
from mdb_baxter_detection.msg import SensData


class exp_motiv_17():
	def __init__(self):
		self.head_state = None
		self.rospack = rospkg.RosPack()
		self.afile = vlc.MediaPlayer(self.rospack.get_path('mdb_baxter_experiments')+"/audio/TaDa.mp3");

		self.fixed_height = -0.03
		self.exp_iteration=1
		self.world = None

		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.pan_pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size = 1)

		self.scene = exp_scene()

		self.ball_pos = None
		self.box_pos = None
		self.obj_type = "exp_small_obj"

		self.mode = rospy.get_param("~mode")
		self.exp_type = rospy.get_param("~exp_type")
		self.config_time = rospy.get_param("~config_time")
		self.sense_fix_flag = rospy.get_param("~sense_fix")
		self.velocity = self.select_velocity()

		### ROS Service Clients

		#policies#
		self.bt_clnt = rospy.ServiceProxy('/baxter_throw', BaxThrow)  
		self.bp_clnt = rospy.ServiceProxy('/baxter_push', BaxP) 
		self.bc_clnt = rospy.ServiceProxy('/baxter_change', BaxChange) 
		self.bgb_clnt = rospy.ServiceProxy('/baxter_grab_both', BaxGB) 
		self.bdb_clnt = rospy.ServiceProxy('/baxter_drop_both', BaxDB) 
		self.bg_clnt = rospy.ServiceProxy('/baxter_grab', BaxG) 
		self.bgm_clnt = rospy.ServiceProxy('/baxter_giveme', BaxChange) 
		self.brap_clnt = rospy.ServiceProxy('/baxter_restore', BaxRAP)
		self.bcf_clnt = rospy.ServiceProxy('/baxter_changeface', BaxCF)
		self.bop_clnt = rospy.ServiceProxy('/baxter_op', BaxChange)
		self.boap_clnt = rospy.ServiceProxy('/baxter_oap', BaxChange)
		self.bolap_clnt = rospy.ServiceProxy('/baxter_olap', BaxChange)
		self.bd_clnt = rospy.ServiceProxy('/baxter_bd', BaxG)

		#sensorization#
		self.bs_clnt = rospy.ServiceProxy('/baxter_sense', GetSense)
		self.bes_clnt = rospy.ServiceProxy('/baxter_bes', BaxSense)

		##SIM##
		#policy#
		self.bsrg_clnt = rospy.ServiceProxy('/baxter_reset_gippers', BaxChange)
		#simulation_management#
		self.load_srv = rospy.ServiceProxy('/sim/create_obj', SimMng)
		self.modify_srv = rospy.ServiceProxy('/sim/modify_obj', SimMng)
		self.delete_srv = rospy.ServiceProxy('/sim/delete_obj', SimMng)
		#check#
		self.bcrc_clnt = rospy.ServiceProxy('/baxter_check_close_reach', BCheckR)
		self.bfrc_clnt = rospy.ServiceProxy('/baxter_check_far_reach', BCheckR)

		### ROS Service Servers
		if self.mode == 'real':
			self.new_exp_srver = rospy.Service("/mdb/baxter/new_experiment", NewExperiment, self.handle_new_exp_real)
			self.ref_world_srver = rospy.Service("/mdb/baxter/refresh_world", RefreshWorld, self.handle_ref_world_real)
		else:
			self.new_exp_srver = rospy.Service("/mdb/baxter/new_experiment", NewExperiment, self.handle_new_exp)
			self.ref_world_srver = rospy.Service("/mdb/baxter/refresh_world", RefreshWorld, self.handle_ref_world)

		self.exec_pol_srver = rospy.Service("/mdb/baxter/exec_policy", ExecPolicy, self.handle_exec_pol)

		rospy.sleep(1)

		#self.scene.add_table()

	def select_velocity(self):
		if self.mode == "sim":
			return 0.5
		else:
			return 0.85
		

	def select_angle(self, arg):
		options = {
			'left': -0.3925,
			'right': 0.3925,
		}
		return options[arg]

	def choose_xd(self, arg):
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
		#return "exp_small_obj"
		#return "exp_big_obj"

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
			if self.bcrc_clnt(BCheckRRequest(Float64(dist), Float64(angle), String(obj))).response.data:
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
				if self.bcrc_clnt(BCheckRRequest(Float64(dist), Float64(angle), String(obj))).response.data and self.check_box_ball(obj, box_x, box_y, ball_x, ball_y):
					box_valid = True
					self.box_pos = [dist, angle]
		return True	

	def refresh_real(self):
		if rospy.has_param("/baxter_sense"): rospy.delete_param("/baxter_sense")
		if rospy.has_param("/check_reward"): rospy.delete_param("/check_reward")
		if rospy.has_param("/box_sense"): rospy.delete_param("/box_sense")
		self.pan_to('front', 0.1) 
		self.obj_type = "exp_small_obj"
		self.adopt_expression("automatic") 
		#rospy.sleep(self.config_time)
		raw_input("Press Key to start experiment")
		#rospy.set_param("/baxter_sense", True)
		self.adopt_expression("normal") 
		self.pan_to('front', 0.1)
		self.complete_pan()
		self.adopt_open_left_pose()

	def handle_new_exp_real(self, req):
		self.world = req.world.data
		self.refresh_real()
		try:
			self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
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
			self.bsrg_clnt(Bool(True), Bool(True))
			self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
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
			self.bsrg_clnt(Bool(True), Bool(True))
			return True
		except rospy.ServiceException, e:
			rospy.loginfo("Delete Model service call failed: {0}".format(e))	
			return False

	def handle_ref_world(self, req):
		self.world = req.world.data
		if self.refresh_world():
			self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			return Bool(True)
		else:
			return Bool(False)

	def handle_ref_world_real(self, req):
		self.world = req.world.data
		self.pan_to('front', 0.1)
		if req.reward.data == True:
			self.reward_sound()
		self.refresh_real()
		try:
			self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
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
			'grasp_object':self.bg_clnt,
			'grasp_with_two_hands':self.bgb_clnt, 
			'change_hands':self.bc_clnt,
			'sweep_object':self.bp_clnt,
			'put_object_in_box':self.bg_clnt,
			'put_object_in_robot':self.bg_clnt,
			'throw':self.bt_clnt,
			'ask_nicely':self.bgm_clnt,
			'drop_object':self.bd_clnt, 
		}
		return options[arg]

	def choose_policy_req (self, arg):
		options = {
			'grasp_object':BaxGRequest(),
			'grasp_with_two_hands':BaxGBRequest(), 
			'change_hands':BaxChangeRequest(),
			'sweep_object':BaxPRequest(),
			'put_object_in_box':BaxGRequest(),
			'put_object_in_robot':BaxGRequest(),
			'throw':BaxThrowRequest(),
			'ask_nicely':BaxChangeRequest(),
			'drop_object':BaxGRequest(),
		}
		return options[arg]

	def choose_arm (self, arg):
		if arg>0:
			return "left"
		elif arg<0:
			return "right"
		else:
			return "left" #TODO:Aqui necesitaria ver por el log realmente cual es el brazo a utilizar atendiendo al estado del gripper antes y despues, etc.

	def adapt_sens(self, sens):
		return [sens[0]/100.0, -math.radians(sens[1]), sens[2]]

	def complete_pan_head (self):   ###### PELIGRO
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
		self.brap_clnt(String('both'))

	def complete_pan (self):
		self.adopt_open_pose()
		rospy.set_param("/baxter_sense", True)
		rospy.set_param("/box_sense", True)
		self.adopt_expression("normal")
		self.pan_to('front', 0.1) 
		#Move head to the left 
		self.adopt_expression("horizon")
		rospy.sleep(1)
		#Move head to the right
		self.adopt_expression("horizon2")
		rospy.sleep(1)
		#Move head to the center
		self.adopt_expression("horizon")
		rospy.sleep(1)
		#rospy.delete_param("/baxter_sense")
		#Move head to the right
		self.adopt_expression("horizon2")
		rospy.sleep(1)
		#Move head to the center
		self.adopt_expression("horizon")
		rospy.sleep(1)
		self.adopt_expression("normal")
		#self.adopt_operation_pose()
		rospy.delete_param("/box_sense")
		self.brap_clnt(String('both'))

	def adopt_operation_pose (self):
		req = BaxChangeRequest()
		req.request.data = True
		self.bop_clnt(req)

	def adopt_open_pose(self):
		req = BaxChangeRequest()
		req.request.data = True
		self.boap_clnt(req)

	def adopt_open_left_pose(self):
		req = BaxChangeRequest()
		req.request.data = True
		self.bolap_clnt(req)

	def adopt_expression(self, expr):
		req = BaxCFRequest()
		req.expression.data = expr
		self.bcf_clnt(req)

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


	def sense_fix(self, global_s, arm):
		if self.obj_type != "exp_big_obj" and self.mode == "real":
			if global_s.obj_sens.angle.data > 0.0 and global_s.obj_sens.dist.data < 1.0:
				global_s.obj_sens.angle.data -=0.01
				if global_s.obj_sens.dist.data < 0.75:
					global_s.obj_sens.angle.data -=0.01					
			elif global_s.obj_sens.angle.data < 0.0 and global_s.obj_sens.dist.data < 1.0:
				global_s.obj_sens.angle.data +=0.01
				if global_s.obj_sens.dist.data < 0.75:
					global_s.obj_sens.angle.data +=0.01
			if abs(global_s.obj_sens.angle.data)<0.9 and global_s.obj_sens.dist.data > 0.75 and arm=="right":
				global_s.obj_sens.dist.data +=0.025

			if global_s.box_sens.angle.data > 0.0 and global_s.box_sens.dist.data < 1.0:
				global_s.box_sens.angle.data -=0.01
				if global_s.box_sens.dist.data < 0.75:
					global_s.box_sens.angle.data -=0.01					
			elif global_s.box_sens.angle.data < 0.0 and global_s.box_sens.dist.data < 1.0:
				global_s.box_sens.angle.data +=0.01
				if global_s.box_sens.dist.data < 0.75:
					global_s.box_sens.angle.data +=0.01
			if abs(global_s.box_sens.angle.data)<0.9 and global_s.box_sens.dist.data > 0.75 and arm=="right":
				global_s.box_sens.dist.data +=0.025
		return global_s

	#TODO: Improve the height management
	def execute_policy(self, policy_code):
		global_s = self.bs_clnt(Bool(True))
		arm = "right"
		srv = self.choose_policy_req(policy_code)
		resp =  None
		#self.control_orientation(global_s, policy_code)

		if (self.sense_fix_flag): 
			print "\nsense_fix\n"
			global_s = self.sense_fix(global_s, arm)

		rospy.loginfo("Iteration %s > Executing policy %s with the %s arm", self.exp_iteration, policy_code, arm)
		self.exp_iteration+=1

		if policy_code == 'grasp_object':
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
				self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
				resp = False
			else:
				resp = self.choose_policy(policy_code)(srv).result.data
				self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			
		if policy_code == 'put_object_in_box':
			srv.object_position.const_dist = global_s.box_sens.dist
			srv.object_position.angle = global_s.box_sens.angle
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
				self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			else:
				resp = self.choose_policy(policy_code)(srv).result.data
			if self.mode=="real" and resp == True:
				rospy.set_param("/check_reward", True)
				self.complete_pan()
				rospy.delete_param("/check_reward")
			self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))

		if policy_code == 'drop_object':
			srv.object_position.const_dist = global_s.box_sens.dist
			srv.object_position.angle = global_s.box_sens.angle
			srv.object_position.height.data = self.fixed_height+self.choose_sweep_height(self.obj_type)

			#Rest of the service request
			srv.orientation.data = "current"
			srv.arm.data = arm
			srv.scale.data = self.velocity
			#Focused face
			self.adopt_expression("focus")

			#Execute policy
			if self.gripper_sense_data(global_s, arm) < 1.0:
				resp = False
				self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))
			else:
				resp = self.choose_policy(policy_code)(srv).result.data
				print self.gripper_sense_data(global_s, arm), resp
			if self.mode=="real" and resp == True:
				rospy.set_param("/check_reward", True)
				self.complete_pan()
				rospy.delete_param("/check_reward")
			self.bes_clnt(Bool(True), Float64(self.choose_xd('exp_box')), Float64(self.choose_xd(self.obj_type)))

		#If it failed
		if not resp:
			self.adopt_expression("confused")
			rospy.sleep(2)

		#Standard face
		self.adopt_expression("normal")
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
	ltme = exp_motiv_17()
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("exp_motiv_17")
	main()	

