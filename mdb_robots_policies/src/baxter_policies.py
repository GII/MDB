#! /usr/bin/env python

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
import tf
import rospkg
import yaml
import math
import numpy as np
from baxter_arm import *
from baxter_display import *
from exp_senses import *
from calibration_policies import calibration_policies
from robobo_policies import *
from dynamic_reconfigure import client
from std_msgs.msg import Bool, Float64, Int32
from geometry_msgs.msg import PointStamped
from mdb_common.msg import OpenGripReq, Candidates
from mdb_common.srv import CandAct, BaxMC, BaxChange, CandActResponse, BaxCart
from mdb_robots_policies.srv import BaxThrow, BaxPush, BaxGrabBoth, BaxDropBoth, BaxGrab, BaxRestoreArmPose, BaxChangeFace, BaxCheckReach, BaxGetCompleteSense, CheckActionValidity, ManagePlanScene, JoystickControl
from gazebo_msgs.srv import GetModelState

class baxter_policies():
	def __init__(self):
		rospy.init_node("baxter_policies_server")
		rospy.on_shutdown(node_shutdown)

		# Variables
		self.rospack = rospkg.RosPack()

		self.safe_operation_height = 0.2
		self.loop_tries = 5

		self.fixed_speed = 2.5
		self.failed_iterations = 0

		self.robobo_status = False

		self.speed_l = []
		self.grip_l = []
		self.impact_l = []
		self.g_angle_l = []
		self.g_low_d_l = []
		self.g_high_d_l = []
		self.g_angle_fat_l = []
		self.g_low_fat_d_l = []
		self.g_high_fat_d_l = []

		self.exp_rec = rospy.get_param("~exp_rec")
		self.super_throw = rospy.get_param("~super_throw")
		self.use_robobo = rospy.get_param("~use_robobo")

		# Baxter low_level movements
		self.baxter_arm = baxter_arm()

		# Baxter display
		self.baxter_display = display()

		# Experiment sensorization
		self.exp_senses = exp_senses(self)

		# Experiment calibration
		self.exp_calibration = calibration_policies(self)

		# Robobo control
		if self.use_robobo: 
			self.robobo_policies = robobo_policies(self)

		# Obtain the initial arms position and configuration
		self.baxter_arm.update_init_data()
		self.baxter_arm.update_data()

		# ROS Subscribers		
		self.rob_loc_sb = rospy.Subscriber("/aruco_single_head/tag_detection", Bool, self.rob_loc_cb)

		# ROS Publishers
		self.grip_pub = rospy.Publisher('/open_grip', OpenGripReq, queue_size = 1)

		# ROS Service Servers
		self.bax_throw_srver = rospy.Service('/baxter/policy/throw', BaxThrow, self.handle_baxter_throw)  
		self.bax_grab_srver = rospy.Service('/baxter/policy/grab', BaxGrab, self.handle_baxter_grab) 
		self.bax_push_srver = rospy.Service('/baxter/policy/push', BaxPush, self.handle_baxter_push) 
		self.bax_change_srver = rospy.Service('/baxter/policy/change_hands', BaxChange, self.handle_baxter_change_hands) 
		self.bax_grab_both_srver = rospy.Service('/baxter/policy/grab_both', BaxGrabBoth, self.handle_baxter_grab_both) 
		self.bax_drop_both_srver = rospy.Service('/baxter/policy/drop_both', BaxDropBoth, self.handle_baxter_drop_both) 
		self.bax_ask_srver = rospy.Service('/baxter/policy/ask_for_help', BaxChange, self.handle_baxter_ask_help)
		self.bax_joystick_control_srver = rospy.Service('/baxter/policy/joystick', JoystickControl, self.handle_joystick_control)
		self.bax_reset_grippers_srver = rospy.Service('/baxter/reset_grippers', BaxChange, self.handle_baxter_reset_grippers)
		self.bax_check_close_reach_srver = rospy.Service('/baxter/check_close_reach', BaxCheckReach, self.handle_baxter_check_close_reach)
		self.bax_check_far_reach_srver = rospy.Service('/baxter/check_far_reach', BaxCheckReach, self.handle_baxter_check_far_reach)

		# Common
		self.bax_restore_srver = rospy.Service('/baxter/restore', BaxRestoreArmPose, self.handle_baxter_restore_pose)
		self.bax_change_face_srver = rospy.Service('/baxter/change_face', BaxChangeFace, self.handle_baxter_change_face)
		self.bax_open_arms_pose_srver = rospy.Service('/baxter/open_arms_pose', BaxChange, self.handle_open_arms_pose)
		self.bax_open_larm_pose_srver = rospy.Service('/baxter/open_left_arm_pose', BaxChange, self.handle_open_left_arm_pose)
		self.bax_open_rarm_pose_srver = rospy.Service('/baxter/open_right_arm_pose', BaxChange, self.handle_open_right_arm_pose)
		self.bax_get_complete_sens_srver = rospy.Service('/baxter/get_complete_sense', BaxGetCompleteSense, self.handle_get_complete_sens)

		# Motivation
		self.bax_drop_srver = rospy.Service('/baxter/policy/drop', BaxGrab, self.handle_baxter_drop)
		self.bax_cart_mov_srver = rospy.Service('/baxter/cart_mov', BaxMC, self.handle_baxter_cartesian_mov)
		self.bax_candidate_actions_srver = rospy.Service ("/baxter/candidate_actions", CandAct, self.handle_cand_act)
		self.bax_check_action_validity_srver = rospy.Service("/baxter/check_action_val", CheckActionValidity, self.handle_check_action_validity)

		# Alex
		self.bax_change_l_gripper_srver = rospy.Service('/baxter/change_l_gripper_state', BaxChange, self.handle_change_l_gripper_state)
		self.bax_change_r_gripper_srver = rospy.Service('/baxter/change_r_gripper_state', BaxChange, self.handle_change_r_gripper_state)
		rospy.Service('/baxter/move_xyz', BaxCart, self.handle_move_xyz)
		rospy.Service('/baxter/move_polar', BaxMC, self.handle_move_polar)

		# ROS Service Clients
		try:
			self.scene_clnt = rospy.ServiceProxy('/mdb3/baxter/modify_planning_scene', ManagePlanScene) ###Used in calibration_policies
		except rospy.ServiceException, e:
			print "Service exception", str(e)
			exit(1)

		# Dynamic reconfigure client instance
		self.client = client.Client ("rsdk_position_w_id_joint_trajectory_action_server")
		self.default_conf = self.client.get_configuration()
		self.update_rconf()

		# Read throw configuration
		self.readtcfile()
		# Obtain throw polinomial model
		self.t_poly = self.obtain_poly(self.grip_l, self.impact_l, 4)

		# Read grab configuration
		self.readgcfile()
		# Obtain grab low polinomial model for the small object
		self.g_lowpoly = self.obtain_poly(self.g_angle_l, self.g_low_d_l, 3)
		# Obtain grab high polinomial model for the small object
		self.g_highpoly = self.obtain_poly(self.g_angle_l, self.g_high_d_l, 3)
		# Obtain grab low polinomial model for the big object
		self.g_lowfatpoly = self.obtain_poly(self.g_angle_fat_l, self.g_low_fat_d_l, 2)
		# Obtain grab high polinomial model for the big object
		self.g_highfatpoly = self.obtain_poly(self.g_angle_fat_l, self.g_high_fat_d_l, 2)

		self.ra_angle_l = []
		self.ra_distance_l = []
		self.ra_area_l = []
		self.readmcfile()
		self.right_arm_motiven_poly = self.obtain_poly(self.ra_angle_l, self.ra_distance_l, 3)

		self.initialize_wrist("right")
		self.initialize_wrist("left")
		self.baxter_arm.update_init_data()
		self.baxter_arm.update_data()
		
				
	##########################
    ##		CALLBACKS		##
	##########################

	def rob_loc_cb(self, loc):
		self.robobo_status = loc.data

	####################
	##     COMMON     ##
	####################
	def update_rconf(self):
		self.update_parameter('stopped_velocity_tolerance', 1.0)

		self.update_parameter('left_s0_trajectory', 1.0)
		self.update_parameter('left_s1_trajectory', 1.0)
		self.update_parameter('left_e0_trajectory', 1.0)
		self.update_parameter('left_e1_trajectory', 1.0)
		self.update_parameter('left_w0_trajectory', 1.0)
		self.update_parameter('left_w1_trajectory', 1.0)

		self.update_parameter('right_s0_trajectory', 1.0)
		self.update_parameter('right_s1_trajectory', 1.0)
		self.update_parameter('right_e0_trajectory', 1.0)
		self.update_parameter('right_e1_trajectory', 1.0)
		self.update_parameter('right_w0_trajectory', 1.0)
		self.update_parameter('right_w1_trajectory', 1.0)

	def update_parameter(self, name, value):
		self.client.update_configuration({name:value})

	def select_opposite(self, arg):
		options = {
			'left':'right',
			'right':'left',
		}
		return options[arg]

	def select_sign(self, arg):
		options = {
			'left': 1.0,
			'right': -1.0,
			'front': 1.0,
		}
		return options[arg]

	def select_offset(self, arg):
		options = {
			'left': -0.75,
			'right': 0.75,
		}
		return options[arg]

	def polar_to_cartesian (self, angle, dist):
		dx = dist*np.cos(angle)
		dy = dist*np.sin(angle)
		return dx, dy

	def cartesian_to_polar(self, co, cc):
		angle = np.arctan(co/cc)
		dist = self.obtain_dist(co, cc)
		return angle, dist

	def obtain_dist(self, x, y):
		return np.sqrt((x**2)+(y**2))

	def obtain_wrist_offset(self, side):
		self.baxter_arm.update_data()
		ori = self.baxter_arm.choose_arm_state(side).current_es.pose.orientation
		quat_in = (ori.x,ori.y,ori.z,ori.w)
		(r, p, y) = tf.transformations.euler_from_quaternion(quat_in)

		front = None
		if y > 0.0:
			front = 3.14
		else:
			front = -3.14

		angle_inc = front - y
		return angle_inc

	def obtain_poly (self, x, y, g):
		return np.poly1d(np.polyfit(x, y, g))

	####################
	##      Throw     ##
	####################

	def readtcfile(self):
		custom_configuration_file = self.rospack.get_path('mdb_robots_policies')+"/config/"+rospy.get_param("~throw_param_file")
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'speed':
				for speed in config[k]:
					self.speed_l.append(speed)
			if k == 'grip':
				for grip in config[k]:
					self.grip_l.append(grip)
			if k == 'impact':
				for matrix in config[k]:
					self.impact_l.append(self.obtain_dist(matrix[0], matrix[1]))

	def search_candidate(self, y_l, c_l, d):
		for i in range(0, len(y_l)):
			if np.around(y_l[i],2)==np.around(d, 2):
				return c_l[i]
		return False

	def obtain_grip(self, d):
		p = np.poly1d(self.t_poly)
		cand_c = (p - d).roots

		cand_f = []
		for ite in range(0, len(cand_c)):
			if cand_c[ite].real+cand_c[ite].imag not in cand_f and 0.5 <= cand_c[ite].real+cand_c[ite].imag <= 1.50:
				cand_f.append(cand_c[ite].real+cand_c[ite].imag)
			if cand_c[ite].real-cand_c[ite].imag not in cand_f and 0.5 <= cand_c[ite].real-cand_c[ite].imag <= 1.50:
				cand_f.append(cand_c[ite].real-cand_c[ite].imag)

		yprime = self.t_poly(cand_f)
		return self.search_candidate(yprime, cand_f, d)

	def pose_du (self, s0, sign, arm):
		angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		angles = [self.select_offset(arm)+s0, -0.9, sign*0.0, 1.0, sign*0.0, 1.27, sign*0.0]
		self.baxter_arm.move_joints_directly(angles, 'moveit', arm, True, 1.0)

	def retract_du (self, scale, arm):
		angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		angles[3] = 2.0
		angles[5] += 0.785
		self.baxter_arm.move_joints_directly(angles, 'moveit', arm, True, scale)

	def launch_du (self, scale, grip, arm):
		grip_req = OpenGripReq()
		angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		grip_req.arm.data = arm
		grip_req.angle = angles[5]-grip
		self.grip_pub.publish(grip_req)
		angles[3] -= 1.57
		angles[5] -= 1.57
		self.baxter_arm.move_joints_directly(angles, 'moveit', arm, True, scale)

	def handle_baxter_throw(self, srv):
		if not self.super_throw:
			if (-1.5<=srv.sensorization.angle.data<=1.5):
				rospy.sleep(0.5)
				if self.baxter_arm.gripper_instate_close(srv.arm.data):
					sign = self.select_sign(srv.arm.data)
					rospy.sleep(0.5)
					shoulder_pos = (0.064, sign*0.259)
					dx, dy = self.polar_to_cartesian(srv.sensorization.angle.data, srv.sensorization.const_dist.data)
					(angle, _) = self.cartesian_to_polar(shoulder_pos[1]-dy, shoulder_pos[0]-dx)
					grip = self.obtain_grip(srv.sensorization.const_dist.data)
					self.pose_du(angle, sign, srv.arm.data)
					self.retract_du(self.fixed_speed, srv.arm.data)
					self.launch_du(self.fixed_speed, grip, srv.arm.data)
					rospy.sleep(1)
					if self.baxter_arm.gripper_instate_open(srv.arm.data):
						self.baxter_arm.restore_arm_pose(srv.arm.data)
						self.baxter_arm.gripper_manager(srv.arm.data)
						self.exp_senses.assign_gripper_sense(srv.arm.data, 0.0)
						return Bool(True)
					else:
						self.baxter_arm.gripper_manager(srv.arm.data)
						self.baxter_arm.restore_arm_pose(srv.arm.data)
			return Bool(False)
		else:
			if self.baxter_arm.gripper_instate_open(srv.arm.data):
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				self.baxter_arm.gripper_manager(srv.arm.data)
				self.exp_senses.assign_gripper_sense(srv.arm.data, 0.0)
				return Bool(True)
			else:
				self.baxter_arm.gripper_manager(srv.arm.data)	
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				return Bool(False)

	####################
	##      Push      ##
	####################

	def fix_w2 (self, angle):
		w2 = angle
		if (angle > 3.059):
			w2 = 3.05
		if (angle < -3.059):
			w2 = -3.05
		return w2

	def translate_into_threshold (self, angle):
		new_angle = angle
		if (angle<-3.14):
			new_angle = angle + 6.28
		if (angle>3.14):
			new_angle = angle - 6.38

		new_angle = self.fix_w2(new_angle)
		return new_angle

	def push_loop(self, srv, x, y, dx, dy):
		if self.loop_tries > 0:
			if self.adjust_w1(srv.arm.data, x, y):
				if self.baxter_arm.move_xyz(x, y, srv.obj_sens.height.data+self.safe_operation_height, False, 'current', srv.arm.data, srv.scale.data, 1.0):
					if self.baxter_arm.move_xyz(x, y, srv.obj_sens.height.data, False, 'current', srv.arm.data, srv.scale.data, 1.0):
						self.baxter_arm.update_data()
						if self.baxter_arm.move_xyz(x+dx, y+dy, srv.obj_sens.height.data, False, "current", srv.arm.data, srv.scale.data, 1.0):
							rospy.sleep(0.5)
							self.baxter_arm.move_xyz(x+dx, y+dy, srv.obj_sens.height.data+0.12, False, "current", srv.arm.data, srv.scale.data, 0.75)
							self.loop_tries = 5
							self.adopt_oap()
							rospy.set_param("/baxter_sense", True)
							rospy.sleep(2)
							rospy.delete_param("/baxter_sense")
							rospy.sleep(2)
							self.baxter_arm.restore_arm_pose(srv.arm.data)
							return True
					return False
				else:
					print "Adjusting"
					self.loop_tries -= 1
					self.push_loop(srv, x, y, dx, dy)
		return False

	def cartesian_to_push (self, d_angle, d_dist, o_angle, o_dist):
		(dx, dy) = self.polar_to_cartesian(d_angle, d_dist)
		(ox, oy) = self.polar_to_cartesian(o_angle, o_dist)
		return dx - ox, dy - oy

	def polar_to_push (self, co, cc):
		(angle, dist) = self.cartesian_to_polar(co, cc)
		if cc < 0.0 and co < 0.0:
			angle += -3.1416
		if cc < 0.0 and co > 0.0:
			angle += 3.1416
		return angle, dist

	def regular_push (self, fx, fy, dx, dy, srv, d_angle):
		if self.baxter_arm.move_xyz(fx, fy, srv.obj_sens.height.data+self.safe_operation_height, False, "current", srv.arm.data, srv.scale.data, 0.95):
			self.orient_gripper(srv.arm.data, d_angle)
			self.baxter_arm.update_data()

			if self.baxter_arm.move_xyz(fx, fy, srv.obj_sens.height.data, False, "current", srv.arm.data, srv.scale.data, 1.0):
				if self.baxter_arm.move_xyz(fx+dx, fy+dy, srv.dest_sens.height.data, False, "current", srv.arm.data, srv.scale.data, 1.0):
					self.baxter_arm.move_xyz(fx+dx, fy+dy, srv.dest_sens.height.data+self.safe_operation_height, False, "current", srv.arm.data, srv.scale.data, 1.0)
					return True
		return False

	def first_push (self, far, srv, obx, oby):
		dist = far - srv.radius.data - 0.04

		new_angle = srv.obj_sens.angle.data
		if 0.35 > new_angle > -0.35:
			new_angle = np.sign(new_angle)*0.35

		fx, fy = self.cartesian_to_push(new_angle, dist, srv.obj_sens.angle.data, srv.obj_sens.const_dist.data)
		f_angle, f_dist = self.polar_to_push(fy, fx)

		odx, ody = self.polar_to_cartesian(f_angle, -srv.radius.data)
		self.pose_grab_far(srv.arm.data, obx + odx, oby + ody)

		self.orient_gripper(srv.arm.data, f_angle)
		self.baxter_arm.update_data()

		if self.push_loop(srv, obx + odx, oby + ody, fx, fy):
			self.baxter_arm.restore_arm_pose(srv.arm.data)
			self.baxter_arm.update_data()
			if self.second_push(srv, dist-0.01, new_angle):
				return True
		return False

	def second_push(self, srv, o_dist, new_angle):
		(obx, oby) = self.polar_to_cartesian(new_angle, o_dist)
		(dx, dy) = self.cartesian_to_push(srv.dest_sens.angle.data, srv.dest_sens.const_dist.data, self.exp_senses.obj_sense.angle.data, self.exp_senses.obj_sense.dist.data)
		(d_angle, d_dist) = self.polar_to_push(dy, dx)
		(odx, ody) = self.polar_to_cartesian(d_angle, -srv.radius.data-0.02)

 		if self.regular_push (obx+odx, oby+ody, dx, dy, srv, d_angle):
			return True
		return False

	def handle_baxter_push(self, srv):
		self.baxter_arm.update_data()

		if self.baxter_arm.gripper_instate_close("left") or self.baxter_arm.gripper_instate_close("right"):
			return Bool(False)

		#Close the gripper
		if self.baxter_arm.gripper_instate_open(srv.arm.data) and srv.grip.data:
			self.baxter_arm.gripper_manager(srv.arm.data)
			rospy.sleep(0.5)

		#Move the opposite arm away from the scene
		op = self.select_opposite(srv.arm.data)
		current_angles = self.baxter_arm.choose_arm_group(op).get_current_joint_values()
		current_angles[0] = np.sign(self.select_sign(op))*0.55
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', op, True, 1.0)
		self.baxter_arm.update_data()

		(obx, oby) = self.polar_to_cartesian(srv.obj_sens.angle.data, srv.obj_sens.const_dist.data)
		(dx, dy) = self.cartesian_to_push(srv.dest_sens.angle.data, srv.dest_sens.const_dist.data, srv.obj_sens.angle.data, srv.obj_sens.const_dist.data)
		(d_angle, d_dist) = self.polar_to_push(dy, dx)

		far = self.g_highpoly(abs(srv.obj_sens.angle.data))
		if srv.obj_sens.const_dist.data < far:
			print "push close"
			odx, ody = self.polar_to_cartesian(d_angle, -srv.radius.data)
			if self.regular_push (obx+odx, oby+ody, dx, dy, srv, d_angle):
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(False)
		else:
			print "push far"
			if self.first_push(far, srv, obx, oby):
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				self.loop_tries = 5
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				self.loop_tries = 5
				return Bool(False)

	##################
	##     Grab     ##
	##################

	def readgcfile(self):
		custom_configuration_file = self.rospack.get_path('mdb_robots_policies')+"/config/"+rospy.get_param("~grab_param_file")
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'angles':
				for angle in config[k]:
					self.g_angle_l.append(angle)
			if k == 'low':
				for low_d in config[k]:
					self.g_low_d_l.append(low_d)
			if k == 'high':
				for high_d in config[k]:
					self.g_high_d_l.append(high_d)
			if k == 'angles_fat':
				for angle_fat in config[k]:
					self.g_angle_fat_l.append(angle_fat)
			if k == 'low_fat':
				for low_fat_d in config[k]:
					self.g_low_fat_d_l.append(low_fat_d)
			if k == 'high_fat':
				for high_fat_d in config[k]:
					self.g_high_fat_d_l.append(high_fat_d)

	def adjust_w1(self, arm, x, y):
		current_p = self.baxter_arm.choose_arm_group(arm).get_current_pose().pose.position
		#2D distance between object and effector
		(_, dist_oe) = self.cartesian_to_polar(y-current_p.y, x-current_p.x)
		#Angle increment
		(angle_inc, _) = self.cartesian_to_polar(dist_oe, current_p.z-(-0.03))

		#Adjust w1 joint
		current_ang = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		current_ang[5] += -angle_inc

		if current_ang[5] > -1.5:
			###Check if angles within limits
			current_ang[6] = self.fix_w2(current_ang[6])
			self.baxter_arm.move_joints_directly(current_ang, 'moveit', arm, True, 2.5)
			self.baxter_arm.update_data()
			return True
		else:
			return False

	def grab_loop(self, srv, x, y, first):
		if self.loop_tries > 0:
			if self.adjust_w1(srv.arm.data, x, y):
				if self.baxter_arm.move_xyz(x, y, srv.object_position.height.data + self.safe_operation_height, False, 'current', srv.arm.data, srv.scale.data, 1.0):
					if self.baxter_arm.move_xyz(x, y, srv.object_position.height.data, True, 'current', srv.arm.data, srv.scale.data, 1.0):
						if self.baxter_arm.move_xyz(x, y, srv.object_position.height.data + self.safe_operation_height, False, 'current', srv.arm.data, srv.scale.data, 1.0):
							self.loop_tries = 5
							return True
					return False
				else:
					print "Adjusting"
					self.loop_tries -= 1
					self.grab_loop(srv, x, y, first)
		return False

	def normal_reach_grab (self, dx, dy, srv, first):
		result = False
		if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
			if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data, True, srv.orientation.data, srv.arm.data, srv.scale.data, 0.5):
				if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
					result = True
		return result

	def pose_grab_far (self, arm, x, y):
		sign = self.select_sign(arm)
		shoulder_pos = (0.064, sign*0.259)
		(s0, _) = self.cartesian_to_polar(shoulder_pos[1]-y, shoulder_pos[0]-x)
		angles = [self.select_offset(arm)+s0, -0.9, sign*0.0, 1.0, sign*0.0, 1.27, sign*0.0]
		self.baxter_arm.move_joints_directly(angles, 'moveit', arm, True, 1.0)
		self.baxter_arm.update_data()

	def far_reach_grab (self, dx, dy, srv, sens, far, first):
		result = False
		self.pose_grab_far(srv.arm.data, dx, dy)
		if self.grab_loop(srv, dx, dy, first):
			if self.baxter_arm.choose_gripper_state(srv.arm.data) and self.exp_rec == "ltm":
				new_angle = sens.angle.data
				if 0.35 > new_angle > -0.35:
					new_angle = np.sign(new_angle)*0.35
				xf, yf = self.polar_to_cartesian(new_angle, far-0.01)
				xf_r, yf_r = self.polar_to_cartesian(new_angle, far+0.10)
				if self.baxter_arm.move_xyz(xf, yf, srv.object_position.height.data, True, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
					if self.baxter_arm.move_xyz(xf_r, yf_r, srv.object_position.height.data + 0.18, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):

						self.adopt_oap()
						rospy.set_param("/baxter_sense", True)
						rospy.sleep(2)
						if self.exp_rec == "ltm":
							rospy.delete_param("/baxter_sense")
							rospy.sleep(2)
						self.baxter_arm.restore_arm_pose('both')

						xf, yf = self.polar_to_cartesian(self.exp_senses.obj_sense.angle.data, self.exp_senses.obj_sense.dist.data)
						if self.normal_reach_grab (xf, yf, srv, first):
							print "far->normal_grab"
							result = True
			else:
				result = True
		return result

	def far_reach_second_grab (self, dx, dy, srv, first, pose):
		result = False
		if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
			self.initialize_wrist(srv.arm.data)
			rospy.sleep(0.5)
			self.baxter_arm.move_to_pose_goal(pose, srv.arm.data, True, 1.0)
			self.baxter_arm.gripper_manager(srv.arm.data)
			rospy.sleep(0.5)
			if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
				result = True
		return result

	def assign_grab_grip (self, side, first):
		if first:
			self.exp_senses.assign_gripper_sense(side, 0.0)
		else:
			self.exp_senses.assign_gripper_sense(side, 1.0)

	def handle_baxter_grab(self, srv):
		self.baxter_arm.update_data()

		sens = srv.object_position
		dx, dy = self.polar_to_cartesian(sens.angle.data, sens.const_dist.data)
		far = self.g_highpoly(abs(sens.angle.data))
		first = self.baxter_arm.choose_gripper_state(srv.arm.data)
		current_angles = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()

		if sens.const_dist.data < far:
			print "Normal reach"
			if self.normal_reach_grab(dx, dy, srv, first):
				if self.exp_rec == "ltm":
					self.baxter_arm.restore_arm_pose(srv.arm.data)
				self.assign_grab_grip(srv.arm.data, first)
				if self.exp_rec == "mot":
					self.baxter_arm.move_joints_directly(current_angles, 'moveit', 'right', True, 1.0)
					rospy.sleep(1)
					self.exp_senses.rob_grip = 0.0
					rospy.set_param("/baxter_sense", True)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(False)

		elif sens.const_dist.data > far:
			print "Far reach"
			if self.far_reach_grab(dx, dy, srv, sens, far-0.05, first):
				if self.exp_rec == "ltm":
					self.baxter_arm.restore_arm_pose(srv.arm.data)
				self.assign_grab_grip(srv.arm.data, first)
				self.loop_tries = 5
				if self.exp_rec == "mot":
					self.baxter_arm.move_joints_directly(current_angles, 'moveit', 'right', True, 1.0)
					rospy.sleep(1)
					self.exp_senses.rob_grip = 0.0
					rospy.set_param("/baxter_sense", True)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				self.loop_tries = 5
				return Bool(False)

	####################
	##     Change     ##
	####################

	def first_pose_both_arms (self, pos):
		angles = [0.2401, -0.9491, -0.6957, 2.074, -1.022, 1.4841, 0.9211, -0.2401, -0.9491, +0.6957, 2.074, +1.022, 1.4841, -0.9211]
		angles[pos]+=1.57
		self.baxter_arm.move_joints_directly(angles, 'moveit', 'both', True, 1.0)

	def approach_one_arm (self, side, sign):
		self.baxter_arm.update_data()
		pos = self.baxter_arm.choose_arm_state(side).current_es.pose.position
		if self.baxter_arm.move_xyz(pos.x, pos.y+sign, pos.z, False, "current", side, 0.75, 1.0):
			return True
		else:
			return False

	def approach_both_arms (self):
		self.baxter_arm.update_data()
		left = self.baxter_arm.choose_arm_init_state('left').current_es.pose.position
		right = self.baxter_arm.choose_arm_init_state('right').current_es.pose.position

		inc = [0.0, -0.1, 0.0, 0.0, 0.0, 0.0]
		points = [left.x,left.y,left.z,right.x,right.y,right.z]
		points_f = [sum(x) for x in zip(inc, points)]
		self.baxter_arm.move_xyz_execute(points_f, False, 'current', 1.0, 1.0)

	def select_order(self):
		order = None
		if ((self.baxter_arm.choose_gripper_state("left")==True) and (self.baxter_arm.choose_gripper_state("right")==False)):
			order = ('left', 'right')
		elif ((self.baxter_arm.choose_gripper_state("right")==True) and (self.baxter_arm.choose_gripper_state("left")==False)):
			order = ('right', 'left')
		return order

	def select_wrist_angle(self, arg):
		options = {
			'left': 13,
			'right': 6,
		}
		return options[arg]

	#NOTE: Removed the message's success variable management
	def handle_baxter_change_hands(self, srv):
		if (srv.request.data == True):
			approach_dist = 0.22
			if self.baxter_arm.gripper_instate_close("left") or self.baxter_arm.gripper_instate_close("right"):
				order = self.select_order()
				if order:
					self.first_pose_both_arms(self.select_wrist_angle(order[0]))
					if self.approach_one_arm(order[1], self.select_sign(order[0])*approach_dist):
						self.baxter_arm.gripper_manager(order[1])
						rospy.sleep(0.4)
						self.baxter_arm.gripper_manager(order[0])
						rospy.sleep(0.25)
						#if self.baxter_arm.gripper_is_grip(order[0]) and not self.baxter_arm.gripper_is_grip(order[1]):
							#self.baxter_arm.restore_arm_pose("both")
							#return Bool(False)
						if self.approach_one_arm(order[1], self.select_sign(order[1])*approach_dist):
							self.baxter_arm.restore_arm_pose("both")
							self.exp_senses.assign_gripper_sense(order[0], 0.0)
							self.exp_senses.assign_gripper_sense(order[1], 1.0)
							### Mirror the object angle sensorization
							self.exp_senses.obj_sense.angle.data = -self.exp_senses.obj_sense.angle.data
							return Bool(True)
						else:
							pos = self.baxter_arm.choose_arm_state(order[1]).current_es.pose.position
							if self.baxter_arm.move_xyz(pos.x+0.10, pos.y, pos.z, False, "current", order[1], 0.75, 1.0):
								self.baxter_arm.restore_arm_pose("both")
								self.exp_senses.assign_gripper_sense(order[0], 0.0)
								self.exp_senses.assign_gripper_sense(order[1], 1.0)
								self.exp_senses.obj_sense.angle.data = -self.exp_senses.obj_sense.angle.data
								return Bool(True)
							else:
								self.baxter_arm.restore_arm_pose("both")
							return Bool(False)
					else:
						self.baxter_arm.restore_arm_pose("both")
		return Bool(False)

	#################################
	##     Grab with both arms     ##
	#################################

	def check_dual_grab (self):
		self.baxter_arm.update_data()
		if self.baxter_arm.choose_arm_state('left').current_es.wrench.force.y < -4.5 and self.baxter_arm.choose_arm_state('right').current_es.wrench.force.y > 4.5:
			return True
		else:
			return False

	def adquire_dual_grab_configuration(self):
		joints_fst = [-0.4019029664259784, -0.58603403895642884, -1.0662234035487642, 1.0085923680346596, 0.8658399083517928, 1.5422865673839378, 1.0785729867024924, 0.4019029664259784, -0.58603403895642884, 1.0662234035487642, 1.0085923680346596, -0.8658399083517928, 1.5422865673839378, -1.0785729867024924]
		self.baxter_arm.move_joints_directly(joints_fst, 'moveit', 'both', True, 1.0)
		self.baxter_arm.update_data()

	def dual_grab_first_step(self, x, y, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points = [x, y+(size/2.0)+0.075, cles.z, x, y-(size/2.0)-0.075, cles.z]
		if self.baxter_arm.move_to_position_goal_both(points, True, 1.0):
			return True
		return False

	def dual_grab_second_step(self, x, y, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points = [cles.x, cles.y, z, cres.x, cres.y, z]
		if self.baxter_arm.move_to_position_goal_both(points, True, 1.0):
			return True
		return False


	def adjust_dual_grab_orientation (self):
		ori_l_act = self.baxter_arm.choose_arm_state('left').current_es.pose.orientation
		ori_r_act = self.baxter_arm.choose_arm_state('right').current_es.pose.orientation

		quat_l_act = (ori_l_act.x, ori_l_act.y, ori_l_act.z, ori_l_act.w)
		quat_r_act = (ori_r_act.x, ori_r_act.y, ori_r_act.z, ori_r_act.w)

		(rl, pl, yl) = tf.transformations.euler_from_quaternion(quat_l_act)
		(rr, pr, yr) = tf.transformations.euler_from_quaternion(quat_r_act)

		pl += 0.3925
		pr += 0.3925

		xfl, yfl, zfl, wfl = tf.transformations.quaternion_from_euler(rl, pl, yl)
		xfr, yfr, zfr, wfr = tf.transformations.quaternion_from_euler(rr, pr, yr)

		if self.baxter_arm.move_to_ori_goal_both([xfl, yfl, zfl, wfl, xfr, yfr, zfr, wfr], True, 1.0):
			return True
		return False

	def both_arms_cartesian_move (self, inc):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points = [cles.x,cles.y,cles.z,cres.x,cres.y,cres.z]
		points_f = [sum(x) for x in zip(inc, points)]
		if self.baxter_arm.move_xyz_execute(points_f, False, 'current', 1.0, 1.0):
			return True
		return False

	def handle_baxter_grab_both(self, srv):
		if not (self.baxter_arm.choose_gripper_state("left") and self.baxter_arm.choose_gripper_state("right")) and not self.check_dual_grab():
			obx, oby = self.polar_to_cartesian(srv.sensorization.angle.data, srv.sensorization.const_dist.data)
			obz = srv.sensorization.height.data
			self.adquire_dual_grab_configuration()
			self.baxter_arm.update_data()
			if self.dual_grab_first_step(obx, oby, obz, srv.size.data):
				if self.dual_grab_second_step(obx, oby, obz, srv.size.data):
					if self.adjust_dual_grab_orientation():
						#self.baxter_arm.gripper_manager("left")
						#self.baxter_arm.gripper_manager("right")
						if self.both_arms_cartesian_move([0.0, -0.125, +0.01, 0.0, +0.125, +0.01]):
							while not self.check_dual_grab():
								self.both_arms_cartesian_move([0.0, -0.02, 0.0, 0.0, +0.02, 0.0])
							if self.both_arms_cartesian_move([0.0, 0.0, 0.10, 0.0, 0.0, 0.10]):
								#if self.check_dual_grab():
								self.exp_senses.assign_gripper_sense('both', 1.0)
								return Bool(True)
			self.baxter_arm.restore_arm_pose("both")
		return Bool(False)

	#################################
	##     Drop with both arms     ##
	#################################

	def adquire_dual_drop_configuration (self, dx, dy, z, size):
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position

		div = 0.05

		initx = (cles.x+cres.x)/2
		inity = (cles.y+cres.y)/2

		distance = self.obtain_dist(dx-initx, dy-inity)
		iterations = distance/div

		incx = (dx-initx)/iterations
		incy = (dy-inity)/iterations

		cposx = initx+incx
		cposy = inity+incy

		keep = True

		print "distance: ", distance
		while (iterations > 1.0) and keep:
			print "iterations: ", iterations
			if self.move_object(cposx, cposy, z, size):
				keep=True
				cposx+=incx
				cposy+=incy
				iterations-=1.0
				self.baxter_arm.update_data()
			else:
				keep=False

		if keep and self.move_object(dx, dy, z, size):
			print "last_step"
			self.baxter_arm.update_data()
			return True
		else:
			return False

	def dual_drop_first_step(self, dx, dy, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points_f = [cles.x, cles.y, z, cres.x, cres.y, z]
		if self.baxter_arm.move_xyz_execute(points_f, False, 'current', 1.0, 1.0):
			return True
		return False

	def handle_baxter_drop_both(self, srv):
		#if self.check_dual_grab():
		dx, dy = self.polar_to_cartesian(srv.destination.angle.data, srv.destination.const_dist.data)
		if self.adquire_dual_drop_configuration(dx, dy, srv.destination.height.data, srv.size.data):
			print "adquire_dual_drop_configuration"
			if self.dual_drop_first_step(dx, dy, srv.destination.height.data, srv.size.data):
				print "dual_drop_first_step"
				if self.both_arms_cartesian_move([0.0, 0.10, 0.0, 0.0, -0.10, 0.0]):
					print "separate arms"
					if self.both_arms_cartesian_move([0.0, 0.0, 0.2, 0.0, 0.0, 0.2]):
						print "up arms"
						if self.move_object(dx, 0.0, 0.25, srv.size.data):
							if not self.check_dual_grab():
								self.baxter_arm.restore_arm_pose('both')
								self.exp_senses.assign_gripper_sense('both', 0.0)
				return Bool(True)
		self.baxter_arm.restore_arm_pose('both')
		return Bool(False)

	##################
	##     Ask      ##
	##################

	def handle_baxter_ask_help(self, srv):
		if (srv.request.data == True):
			self.baxter_display.changeDisplay("giveme")
			rospy.sleep(10)
			return Bool(True)
		return Bool(False)

	########################
	##	Joystick Control  ##
	########################

	def handle_joystick_control (self, srv):
		(joystick_x, joystick_y) = self.polar_to_cartesian(srv.joystick_pos.angle.data, srv.joystick_pos.const_dist.data)
		if self.baxter_arm.move_xyz(joystick_x, joystick_y, srv.joystick_pos.height.data + self.safe_operation_height, False, 'current', srv.arm_to_move.data, srv.velocity_scale.data, 1.0):
			self.orient_gripper(srv.arm_to_move.data, srv.joystick_angle.data + 1.57)
			if self.baxter_arm.move_xyz(joystick_x, joystick_y, srv.joystick_pos.height.data, True, 'current', srv.arm_to_move.data, srv.velocity_scale.data, 1.0):
				(dx, dy) = self.polar_to_cartesian(srv.joystick_angle.data, 0.03)
				if self.baxter_arm.move_xyz(joystick_x+dx, joystick_y+dy, srv.joystick_pos.height.data, False, 'joystick', srv.arm_to_move.data, srv.velocity_scale.data, 1.0):
					#self.robobo_policies.joystick_rob_move(dx, dy, srv.time_to_control.data)
					rospy.sleep(srv.time_to_control.data)
					if self.baxter_arm.move_xyz(joystick_x, joystick_y, srv.joystick_pos.height.data, True, 'current', srv.arm_to_move.data, srv.velocity_scale.data, 1.0):
						if self.baxter_arm.move_xyz(joystick_x, joystick_y, srv.joystick_pos.height.data + self.safe_operation_height, False, 'current', srv.arm_to_move.data, srv.velocity_scale.data, 1.0):
							self.baxter_arm.restore_arm_pose(srv.arm_to_move.data)
							return Bool(True)
		return Bool(False)
	


	##############################
	##     Restore position     ##
	##############################
	def handle_baxter_restore_pose(self, srv):
		self.baxter_arm.restore_arm_pose(srv.arm.data)
		return Bool(True)

	#######################
	##    Change face    ##
	#######################
	def handle_baxter_change_face(self, srv):
		self.baxter_display.changeDisplay(srv.expression.data)
		return Bool(True)

	##########################
	##    Open arms pose    ##
    ##########################
	def handle_open_arms_pose(self, srv):
		if srv.request.data:
			self.adopt_oap()
			return Bool(True)
		return Bool(False)

	def adopt_oap(self):
		pose = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673, -1.7000342081740099, -0.9986214929134043, 1.1876846250202815, 1.9378012302962488, -0.6680486331240977, 1.0339030510347689, 0.49777676566881673]
		self.baxter_arm.move_joints_directly(pose, 'moveit', 'both', True, 1.0)
		self.baxter_arm.update_data()

	def handle_open_left_arm_pose(self, srv):
		pose = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673]
		if srv.request.data:
			self.baxter_arm.move_joints_directly(pose, 'moveit', 'left', True, 1.0)
			self.baxter_arm.update_data()
			return Bool(True)
		return Bool(False)

	def handle_open_right_arm_pose(self, srv):
		pose = [-1.7000342081740099, -0.9986214929134043, 1.1876846250202815, 1.9378012302962488, -0.6680486331240977, 1.0339030510347689, 0.49777676566881673]
		if srv.request.data:
			self.baxter_arm.move_joints_directly(pose, 'moveit', 'right', True, 1.0)
			self.baxter_arm.update_data()
			return Bool(True)
		return Bool(False)

	##########################
	##    Reset Grippers	##
	##########################

	def handle_baxter_reset_grippers(self, srv):
		if (srv.request.data == True):
			self.baxter_arm.lgripper.open()
			self.baxter_arm.rgripper.open()
			self.baxter_arm.lgripper_state = False
			self.baxter_arm.rgripper_state = False
			self.exp_senses.assign_gripper_sense('both', 0.0)
			return Bool(True)
		return Bool(False)

	def handle_change_l_gripper_state(self, srv):
		if (srv.request.data == True):
			self.baxter_arm.gripper_manager('left')
			return Bool(True)
		return Bool(False)

	def handle_change_r_gripper_state(self, srv):
		if (srv.request.data == True):
			self.baxter_arm.gripper_manager('right')
			return Bool(True)
		return Bool(False)

	##########################
	##    Check close reach	##
	##########################
	def handle_baxter_check_close_reach (self, srv):
		dist = 0.0
		if srv.type.data == 'exp_small_obj':
			dist = self.g_lowpoly(srv.ang.data)
		elif srv.type.data == 'exp_big_obj':
			dist = self.g_lowfatpoly(srv.ang.data)
		if srv.dist.data > dist:
			return Bool(True)
		else:
			return Bool(False)

	##########################
	##    Check far reach	##
	##########################
	def handle_baxter_check_far_reach (self, srv):
		dist = 0.0
		if srv.type.data == 'exp_small_obj':
			dist = self.g_highpoly(srv.ang.data)
		elif srv.type.data == 'exp_big_obj':
			dist = self.g_highfatpoly(srv.ang.data)
		if srv.dist.data < dist:
			return Bool(True)
		else:
			return Bool(False)

	##########################
	##		   Sense		##
	##########################
	def handle_get_complete_sens(self, srv):
		if (srv.request.data == True):
			self.exp_senses.publish_current_senses(srv.box_rad.data, srv.obj_rad.data)
			return Bool(True)
		return Bool(False)

	######################
	##	Initialization	##
	######################

	def initialize_wrist (self, side):
		angle_correction = self.obtain_wrist_offset(side)
		current_angles = self.baxter_arm.choose_arm_group(side).get_current_joint_values()
		current_angles[6] -= angle_correction
		current_angles[6] = self.translate_into_threshold(current_angles[6])
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', side, True, 2.0)
		self.baxter_arm.update_data()

	##########################
	##	Cartesian Movement  ##
	##########################
	def orient_gripper (self, arm, f_angle):
		angle_correction = self.obtain_wrist_offset(arm)
		current_angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		angle = -f_angle+current_angles[6]-angle_correction
		final_angle = self.translate_into_threshold(angle)
		current_angles[6] = final_angle
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', arm, True, 2.0)

	def angle_fix (self, angle):
		new_angle = angle
		if (angle<-3.14):
			new_angle = angle + 6.28
		if (angle>3.14):
			new_angle = angle - 6.28
		return new_angle

	def aim_gripper (self, arm, angle):
		#Orient the gripper towards the destination
		angle_correction = self.obtain_wrist_offset(arm)
		current_angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		current_angles[6]-= angle
		current_angles[6] = self.translate_into_threshold(current_angles[6])
		print "Aim gripper towards the angle ", -(angle_correction - angle)
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', arm, True, 2.0)
		self.baxter_arm.update_data()

	def update_gripper_orientation (self, arm='right'):
		self.exp_senses.rgrip_ori = -self.obtain_wrist_offset(arm)
		print "Current gripper orientation: ", self.exp_senses.rgrip_ori

	def handle_move_xyz(self, srv):

		if srv.valid.data == True:
			#Update the current gripper orientation
			self.update_gripper_orientation(srv.arm.data)

			#Move the gripper towards the destination
			pos = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose.position
			print "pos ini robot", pos.x, pos.y, pos.z
			self.exp_senses.rgrip_ori = self.angle_fix(self.exp_senses.rgrip_ori)

			result = False

			if self.baxter_arm.move_xyz(srv.dest.x.data, srv.dest.y.data, srv.dest.z.data, srv.grip.data, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
				self.baxter_arm.update_data()
				#Update the current gripper orientation
				self.update_gripper_orientation()
				result = True

			return Bool(result)
		return Bool(True)

	def handle_move_polar(self, srv):
		if srv.valid.data == True:
			# Update the current gripper orientation
			self.update_gripper_orientation(srv.arm.data)

			(x, y) = self.polar_to_cartesian(srv.dest.angle.data, srv.dest.const_dist.data)

			z = srv.dest.height.data
			result = False

			if self.baxter_arm.move_xyz(x, y, z, False, srv.orientation.data, srv.arm.data,
										srv.scale.data, 1.0):
				self.baxter_arm.update_data()
				# Update the current gripper orientation
				self.update_gripper_orientation()
				result = True

			return Bool(result)
		return Bool(True)

	def handle_baxter_cartesian_mov(self, srv):
		if srv.valid.data == True:
			#Update the current gripper orientation
			self.update_gripper_orientation(srv.arm.data)

			#Orient the gripper towards the destination
			self.aim_gripper(srv.arm.data, srv.dest.angle.data)

			#Update the current gripper orientation
			self.update_gripper_orientation(srv.arm.data)

			#Move the gripper towards the destination
			pos = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose.position
			self.exp_senses.rgrip_ori = self.angle_fix(self.exp_senses.rgrip_ori)

			(x, y) = self.polar_to_cartesian(self.exp_senses.rgrip_ori, srv.dest.const_dist.data)
			z = srv.dest.height.data
			result = False

			if self.baxter_arm.move_xyz(pos.x+x, pos.y+y, pos.z+z, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
				self.baxter_arm.update_data()
				#Update the current gripper orientation
				self.update_gripper_orientation()
				result = True

			return Bool(result)
		return Bool(True)

	##################################
	###		    CANDIDATES         ###
	##################################

	def check_baxter_validity(self, baxter_angle, arm="right", distance=0.07):
		angle_to_check = self.exp_senses.rgrip_ori
		angle_to_check += math.radians(baxter_angle)
		angle_to_check = self.angle_fix(angle_to_check)

		pos = self.baxter_arm.choose_arm_state(arm).current_es.pose.position
		x,y = self.polar_to_cartesian(angle_to_check, distance)
		resp = self.baxter_arm.move_xyz_plan(pos.x+x, pos.y+y, pos.z, "current", arm, 1.0)

		if ((pos.x+x > self.ra_area_l[0]) and (pos.x+x<self.ra_area_l[1]) and (pos.y+y<self.ra_area_l[2]) and (pos.y+y>self.ra_area_l[3])) and resp != False:
			return True
		else:
			return False

	def baxter_candidates(self, srv, arm="right", distance=0.07):
		baxter_l_angle = np.random.uniform(srv.limit.data, -srv.limit.data)
		if self.check_baxter_validity(baxter_l_angle, arm, distance):
			return Int32(baxter_l_angle), Bool(True)
		else:
			return Int32(baxter_l_angle), Bool(False)

	def handle_cand_act (self, srv):
		candidate_n = 0
		current_action_number = 0
		action_max_number = 10*srv.action_number.data
		valid_action_number = 0

		response = CandActResponse()
		while candidate_n < srv.action_number.data:
			candidate = Candidates()
			(candidate.baxter_action, candidate.baxter_valid) = self.baxter_candidates(srv)
			if self.self.exp_rec!="ltm": (candidate.robobo_action, candidate.robobo_valid) = self.robobo_policies.candidate_actions(srv)
			if (candidate.baxter_valid.data == True and candidate.robobo_valid.data == True):
				response.candidates.append(candidate)
				candidate_n+=1
				valid_action_number+=1
			elif valid_action_number > 4 and current_action_number > action_max_number:
				response.candidates.append(candidate)
				candidate_n+=1
			current_action_number+=1

		return response

	def handle_check_action_validity(self, srv):
		bax_is_valid = False
		rob_is_valid = False

		bax_is_valid = self.check_baxter_validity(srv.baxter_angle.data)
		if self.self.exp_rec!="ltm": rob_is_valid = self.robobo_policies.check_robobo_validity (srv.robobo_angle.data)
		return Bool(bax_is_valid), Bool(rob_is_valid)

	##################################
	###			Drop Object		   ###
	##################################

	def readmcfile(self):
		custom_configuration_file = self.rospack.get_path('mdb_robots_policies')+"/config/"+rospy.get_param("~motiven_param_file")
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'right_arm_angle':
				for angle in config[k]:
					self.ra_angle_l.append(angle)
			if k == 'right_arm_distance':
				for distance in config[k]:
					self.ra_distance_l.append(distance)
			if k == 'right_arm_area_limit':
				for limit in config[k]:
					self.ra_area_l.append(limit)

	def normal_reach_drop (self, dx, dy, srv):
		result = False
		if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
			self.baxter_arm.rgripper.open()
			self.baxter_arm.rgripper_state = False
			self.exp_senses.assign_gripper_sense(srv.arm.data, 0.0)
			result = True
		return result

	def far_reach_drop (self, dx, dy, srv):
		result = False
		self.pose_grab_far(srv.arm.data, dx, dy)
		if self.normal_reach_drop(dx, dy, srv):
			return True
		return result

	def handle_baxter_drop(self, srv):
		self.baxter_arm.update_data()
		sens = srv.object_position
		dx, dy = self.polar_to_cartesian(sens.angle.data, sens.const_dist.data)
		far = self.right_arm_motiven_poly(abs(sens.angle.data))

		first = self.baxter_arm.choose_gripper_state(srv.arm.data)

		if sens.const_dist.data < far:
			if self.normal_reach_drop(dx, dy, srv):
				self.assign_grab_grip(srv.arm.data, first)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(False)

		elif sens.const_dist.data > far:
			if self.far_reach_drop(dx, dy, srv):
				self.assign_grab_grip(srv.arm.data, first)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(False)

def node_shutdown():
	moveit_commander.roscpp_shutdown()

def baxter_policies_server():
	baxter_policies()
	rospy.spin()

if __name__ == '__main__':
	baxter_policies_server()
