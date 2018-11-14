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
import vlc
import rospkg
import numpy as np
from baxter_core_msgs.msg import HeadPanCommand, HeadState
from mdb_baxter_policies.srv import BaxRestoreArmPose, BaxChangeFace, BaxCheckReach, ManagePlanScene
from mdb_baxter_policies.srv import BaxRestoreArmPoseRequest, BaxChangeFaceRequest, BaxCheckReachRequest, ManagePlanSceneRequest
from std_msgs.msg import Bool, Float64, String
from mdb_common.srv import BaxChange, ExecPolicy, RefreshWorld, NewExperiment, BaxChangeRequest
from mdb_baxter_experiments.srv import SimMng, SimMngRequest
from mdb_common.msg import SensData, ControlMsg
from policies_manager import *

class exp_core():
	def __init__(self):
		rospy.init_node("exp_core")

		self.rospack = rospkg.RosPack()
		self.afile = vlc.MediaPlayer(self.rospack.get_path('mdb_baxter_experiments')+"/audio/TaDa.mp3");

		self.mode = rospy.get_param("~mode")
		self.exp_type = rospy.get_param("~exp_type")
		self.exp_rec = rospy.get_param("~exp_rec")
		self.exp_iteration=1

		self.head_state = None
		self.world = None
		self.obj_type = None			
		if self.exp_rec == 'mot':
			self.obj_type = "exp_small_obj"

		self.ball_pos = None
		self.box_pos = None

		self.policies_manager = policies_manager(self)

		# Service Proxies
		self.bax_restore_arm_pose_clnt = rospy.ServiceProxy('/baxter/restore', BaxRestoreArmPose)
		self.bax_change_face_clnt = rospy.ServiceProxy('/baxter/change_face', BaxChangeFace)
		self.bax_open_arms_pose_clnt = rospy.ServiceProxy('/baxter/open_arms_pose', BaxChange)
		self.scene_clnt = rospy.ServiceProxy('/mdb3/baxter/modify_planning_scene', ManagePlanScene)
		self.bax_reset_grippers_clnt = rospy.ServiceProxy('/baxter/reset_gippers', BaxChange)
		self.bax_check_close_reach_clnt = rospy.ServiceProxy('/baxter/check_close_reach', BaxCheckReach)
		self.load_srv = rospy.ServiceProxy('/sim/create_obj', SimMng)
		self.modify_srv = rospy.ServiceProxy('/sim/modify_obj', SimMng)
		self.delete_srv = rospy.ServiceProxy('/sim/delete_obj', SimMng)

		# Service Servers
		if self.mode == 'real':
			self.new_exp_srver = rospy.Service("/mdb/baxter/new_experiment", NewExperiment, self.handle_new_exp_real)
			self.ref_world_srver = rospy.Service("/mdb/baxter/refresh_world", RefreshWorld, self.handle_ref_world_real)
		else:
			self.new_exp_srver = rospy.Service("/mdb/baxter/new_experiment", NewExperiment, self.handle_new_exp)
			self.ref_world_srver = rospy.Service("/mdb/baxter/refresh_world", RefreshWorld, self.handle_ref_world)
		self.exec_pol_srver = rospy.Service("/mdb/baxter/exec_policy", ExecPolicy, self.handle_exec_pol)

		# Publishers
		self.pan_pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size = 1)

		# Subscribers
		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.control_sub = rospy.Subscriber("/mdb/baxter/control", ControlMsg, self.control_cb)
		self.executed_policy_sub = rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_cb)

		self.scene_clnt(String('head_collision'),String('add'),Float64(0.0), Float64(0.0), Float64(0.0))
		self.scene_clnt(String('table'),String('remove'),Float64(0.0), Float64(0.0), Float64(0.0))
	
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
			self.policies_manager.bax_get_complete_sense_clnt(Bool(True), Float64(self.policies_manager.choose_x_dimension('exp_box')), Float64(self.policies_manager.choose_x_dimension(self.obj_type)))
		except rospy.ServiceException, e:
			rospy.loginfo("Sense service call failed: {0}".format(e))

	def executed_policy_cb (self, policy):
		resp = self.policies_manager.execute_policy(policy.data)		

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

	def check_items_conflict (self, obj, box_x, box_y, ball_x, ball_y): #S#
		print "check_items_conflict: ", box_x, box_y, ball_x, ball_y
		if box_x+(self.policies_manager.choose_x_dimension("exp_box")*100) < ball_x-(self.policies_manager.choose_x_dimension(obj)*100) or box_x-(self.policies_manager.choose_x_dimension("exp_box")*100) > ball_x+(self.policies_manager.choose_x_dimension(obj)*100) or box_y+(self.policies_manager.choose_y_dimension("exp_box")*100) < ball_y-(self.policies_manager.choose_y_dimension(obj)*100) or box_y-(self.policies_manager.choose_y_dimension("exp_box")*100) > ball_y+(self.policies_manager.choose_y_dimension(obj)*100):
			return True
		else:
			return False
		
	def randomize_positions(self, obj): #S#
		table_dim_x_min = 0.22
		table_dim_x_max = 0.22 + 1.22
		table_dim_y_min = 0.0		
		table_dim_y_max = 2.442

		ball_x = None
		ball_y = None
		ball_valid = False
		while not ball_valid:
			ball_x = np.random.randint(low=(table_dim_x_min+self.policies_manager.choose_x_dimension(obj))*100, high=(table_dim_x_max-self.policies_manager.choose_x_dimension(obj))*100)
			ball_y = np.random.randint(low=(table_dim_y_min+self.policies_manager.choose_y_dimension(obj))*100, high=(table_dim_y_max-self.policies_manager.choose_y_dimension(obj))*100)
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
				if self.check_items_conflict(obj, box_x*100, box_y*100, ball_x, ball_y):
					box_valid = True
					self.box_pos = [dist*0.01, angle*0.01]
			else:
				box_x = np.random.randint(low=(table_dim_x_min+self.policies_manager.choose_x_dimension('exp_box'))*100, high=(table_dim_x_max-self.policies_manager.choose_x_dimension('exp_box'))*100)
				box_y = np.random.randint(low=(table_dim_y_min+self.policies_manager.choose_y_dimension('exp_box'))*100, high=(table_dim_y_max-self.policies_manager.choose_y_dimension('exp_box'))*100)
				dist, angle = self.translate_c2p(box_x*0.01, (box_y*0.01)-1.221)
				if self.bax_check_close_reach_clnt(BaxCheckReachRequest(Float64(dist), Float64(angle), String(obj))).response.data and self.check_items_conflict(obj, box_x, box_y, ball_x, ball_y):
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
		if self.exp_rec == 'mot' and rospy.has_param("/box_sense"): rospy.delete_param("/box_sense")
		self.bax_reset_grippers_clnt(Bool(True), Bool(True))
		self.obj_type = self.randomize_obj()
		self.choose_world_image()
		raw_input("Press Key after configuring the experiment")
		print "Starting experiment"
		self.bax_restore_arm_pose_clnt(String('both'))
		self.adopt_expression("normal") 
		self.complete_pan_static()
		if self.exp_rec == 'mot'
			self.adopt_open_left_pose()

	def handle_new_exp_real(self, req):
		self.world = req.world.data
		self.refresh_real()
		try:
			self.policies_manager.bax_get_complete_sense_clnt(Bool(True), Float64(self.policies_manager.choose_x_dimension('exp_box')), Float64(self.policies_manager.choose_x_dimension(self.obj_type)))
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Sense service call failed: {0}".format(e))
			return Bool(False)
				
	def handle_new_exp(self, req): #S#
		self.world = req.world.data
		self.obj_type =  self.randomize_obj()
		self.randomize_positions(self.obj_type)

		try:
			self.load_srv(SimMngRequest(model_name=String('exp_table')))
			self.load_srv(SimMngRequest(model_name=String(self.obj_type), sense=SensData(dist=Float64(self.ball_pos[0]), angle=Float64(self.ball_pos[1]), height=Float64(0.0))))
			self.load_srv(SimMngRequest(model_name=String('exp_box'), sense=SensData(dist=Float64(self.box_pos[0]), angle=Float64(self.box_pos[1]), height=Float64(0.0))))
			self.bax_reset_grippers_clnt(Bool(True), Bool(True))
			self.policies_manager.bax_get_complete_sense_clnt(Bool(True), Float64(self.policies_manager.choose_x_dimension('exp_box')), Float64(self.policies_manager.choose_x_dimension(self.obj_type)))
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Delete Model service call failed: {0}".format(e))
			return Bool(False)

	def refresh_world (self): #S#
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

	def handle_ref_world(self, req): #S#
		self.world = req.world.data
		if self.refresh_world():
			self.policies_manager.bax_get_complete_sense_clnt(Bool(True), Float64(self.policies_manager.choose_x_dimension('exp_box')), Float64(self.policies_manager.choose_x_dimension(self.obj_type)))
			return Bool(True)
		else:
			return Bool(False)

	def handle_ref_world_real(self, req):
		self.world = req.world.data
		self.pan_to('front', 0.1)
		if req.reward.data == True:
			self.reward_sound()
		else:
			self.bax_restore_arm_pose_clnt(String('both'))
			self.bax_reset_grippers_clnt(Bool(True), Bool(True))
		self.refresh_real()
		try:
			self.policies_manager.bax_get_complete_sense_clnt(Bool(True), Float64(self.policies_manager.choose_x_dimension('exp_box')), Float64(self.policies_manager.choose_x_dimension(self.obj_type)))
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Sense service call failed: {0}".format(e))
			return Bool(False)

	def handle_exec_pol (self, req):
		resp = self.policies_manager.execute_policy(req.policy_code.data)
		return Bool(resp)

	def head_state_cb(self, state):
		self.head_state = state

	def complete_pan_static (self):
		self.adopt_open_pose()
		rospy.set_param("/baxter_sense", True)
		self.adopt_expression("normal")
		self.adopt_expression("horizon")
		rospy.sleep(1)
		self.adopt_expression("horizon2")
		rospy.sleep(1)
		self.adopt_expression("horizon")
		rospy.sleep(1)
		rospy.delete_param("/baxter_sense")
		self.adopt_expression("horizon2")
		rospy.sleep(1)
		self.adopt_expression("horizon")
		rospy.sleep(3)
		self.adopt_expression("normal")
		self.bax_restore_arm_pose_clnt(String('both'))

	def adopt_open_pose(self):
		req = BaxChangeRequest()
		req.request.data = True
		self.bax_open_arms_pose_clnt(req)

	def adopt_expression(self, expr):
		req = BaxChangeFaceRequest()
		req.expression.data = expr
		self.bax_change_face_clnt(req)

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
	ltme = exp_core()
	rospy.spin()

if __name__ == '__main__':
	main()	

