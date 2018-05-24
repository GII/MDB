#! /usr/bin/env python

import tf, rospkg, yaml, math
from baxter_arm import *
from baxter_display import *
from exp_senses import *
from robobo_policies import robobo_policies
from dynamic_reconfigure import client
import numpy as np
from std_msgs.msg import Bool, Float64, Int32
from geometry_msgs.msg import PointStamped
from baxter_core_msgs.msg import HeadPanCommand, HeadState
from moveit_msgs.msg import OrientationConstraint, Constraints
from mdb_common.msg import OpenGripReq, Candidates, ObjDet
from mdb_common.srv import CandAct, BaxMC, BaxChange, CandActResponse
from mdb_baxter_policies.srv import BaxThrow, BaxP, BaxGB, BaxDB, BaxG, PickAdj, BaxRAP, BaxCF, BCheckR, BaxSense, CheckAct, Calib, BaxFMCM, GridCalib, PlanMng
from dynamic_reconfigure.srv import Reconfigure
from gazebo_msgs.srv import GetModelState

class baxter_policies():
	def __init__(self):
		rospy.init_node("baxter_policies_server")
		rospy.on_shutdown(node_shutdown)

		# Variables
		self.head_state = None
		self.rospack = rospkg.RosPack()

		self.safe_operation_height = 0.2
		self.loop_tries = 5

		self.fixed_speed = 2.5
		self.grab_total_inc = 0.0
		self.failed_iterations = 0

		self.robobo_status = False
		self.aruco_cent = None
		self.calib_obj_flag = False

		self.speed_l = []
		self.grip_l = []
		self.impact_l = []
		self.g_angle_l = []
		self.g_low_d_l = []
		self.g_high_d_l = []
		self.g_angle_fat_l = []
		self.g_low_fat_d_l = []
		self.g_high_fat_d_l = []

		self.mode = rospy.get_param("~mode")
		self.exp_rec = rospy.get_param("~exp_rec")

		# Baxter low_level movements
		self.baxter_arm = baxter_arm()

		# Baxter display
		self.baxter_display = display()

		# Experiment sensorization
		self.exp_senses = exp_senses()

		# Robobo control
		if self.exp_rec!="ltm": self.robobo_policies = robobo_policies(self)

		# Obtain the initial arms position and configuration
		self.baxter_arm.update_init_data()
		self.baxter_arm.update_data()

		# ROS Subscribers
		self.rob_loc_sb = rospy.Subscriber("/aruco_single_head/tag_detection", Bool, self.rob_loc_cb)
		self.aruco_cent_sb = rospy.Subscriber("/aruco_single_head/pixel", PointStamped, self.aruco_cent_cb)
		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.track_sub = rospy.Subscriber("/tracking/ball", ObjDet, self.track_cb)
		self.calib_obj_flag_sub = rospy.Subscriber("/tracking/ball_flag", Bool, self.calib_obj_flag_cb)

		# ROS Publishers
		self.grip_pub = rospy.Publisher('/open_grip', OpenGripReq, queue_size = 1)
		self.pan_pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size = 1)

		# ROS Service Servers
		self.bt_srver = rospy.Service('/baxter_throw', BaxThrow, self.handle_bt)  
		self.bp_srver = rospy.Service('/baxter_push', BaxP, self.handle_bp) 
		self.bc_srver = rospy.Service('/baxter_change', BaxChange, self.handle_bc) 
		self.bgb_srver = rospy.Service('/baxter_grab_both', BaxGB, self.handle_bgb) 
		self.bdb_srver = rospy.Service('/baxter_drop_both', BaxDB, self.handle_bdb) 
		self.bg_srver = rospy.Service('/baxter_grab', BaxG, self.handle_bg_prev) 
		self.bgm_srver = rospy.Service('/baxter_giveme', BaxChange, self.handle_bgm)
		self.brap_srver = rospy.Service('/baxter_restore', BaxRAP, self.handle_brap)
		self.bcf_srver = rospy.Service('/baxter_changeface', BaxCF, self.handle_bcf)
		self.bop_srver = rospy.Service('/baxter_op', BaxChange, self.handle_op)
		self.boap_srver = rospy.Service('/baxter_oap', BaxChange, self.handle_oap)
		self.bolap_srver = rospy.Service('/baxter_olap', BaxChange, self.handle_olap)
		self.borap_srver = rospy.Service('/baxter_orap', BaxChange, self.handle_orap)
		self.bes_srver = rospy.Service('/baxter_bes', BaxSense, self.handle_bes)
		self.bd_srver = rospy.Service('/baxter_bd', BaxG, self.handle_bd)

		self.bg_srver = rospy.Service('/baxter_grab_reach', BaxG, self.handle_bg_reach)

		self.baxter_fm_cart_mov_srver = rospy.Service('/baxter_fm_cart_mov', BaxFMCM, self.handle_baxter_cartesian_fm_mov)
		self.baxter_cart_mov_srver = rospy.Service('/baxter_cart_mov', BaxMC, self.handle_bcm)
		self.candidate_actions_srver = rospy.Service ("/candidate_actions", CandAct, self.handle_cand_act)
		self.check_action_validity_srver = rospy.Service("/check_action_val", CheckAct, self.handle_check_action_validity)
		self.baxter_sa_srver = rospy.Service('/baxter_sa', BaxChange, self.handle_sa)

		self.baxter_calib_srver = rospy.Service('/baxter_calib', Calib, self.matrix_movement)
		self.baxter_int_calib_srver = rospy.Service('/baxter_int_calib', GridCalib, self.interpolation_calibration)

		# ROS Simulation Service Servers
		self.bsrg_srver = rospy.Service('/baxter_reset_gippers', BaxChange, self.handle_bsrg)
		self.bcrc_srver = rospy.Service('/baxter_check_close_reach', BCheckR, self.handle_bcrc)
		self.bfrc_srver = rospy.Service('/baxter_check_far_reach', BCheckR, self.handle_bfrc)

		# ROS Simulation Clients
		self.get_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

		# ROS Service Clients
		try:
			self.pickadj_clnt = rospy.ServiceProxy('/pickup_adjustment', PickAdj)
			self.scene_clnt = rospy.ServiceProxy('/mdb3/baxter/modify_planning_scene', PlanMng)
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
		self.t_poly = self.obtain_t_poly()

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

		###################
		###		TEST	###
		###################

		'''self.baxter_arm.AddElements()
		self.baxter_arm.GraspCreation()
		self.baxter_arm.pickup("small_obj", self.baxter_arm.grasps)'''

		left_test_angles = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673] #moveit
		left_test_angles_2 = [0.5565567001634442, 1.9541028505160654, 1.3091857254483799, -0.9385845547335236, -0.6373815959474021, 2.0937221905141428, 1.5304675427922119] #baxter

		#left_test_speed = [2.0, 6.0, 2.0, 4.0, 4.0, 0.0, 0.0]
		#left_test_acceleration = [0.3, 0.3, 0.3, 0.7, 0.7, 0.7, 0.7]
		#self.baxter_arm.move_joints_raw_position(left_test_angles_2, left_test_speed, left_test_acceleration, 'baxter', 'left', True, 5)

		#self.baxter_arm.move_joints_command(4, left_test_angles_2, 'left', 'baxter')
		#self.baxter_arm.move_joints_interface('left', left_test_angles, 0.05, 'moveit')
		
				
	##########################
    ##		CALLBACKS		##
	##########################

	def rob_loc_cb(self, loc):
		self.robobo_status = loc.data

	def aruco_cent_cb(self, Pose):
		if self.robobo_status:
			self.aruco_cent = [Pose.point.x, Pose.point.y]
		else:
			self.aruco_cent = 'None'

	def track_cb(self, objsens):
		if self.calib_obj_flag:
			self.aruco_cent = [objsens.u.data, objsens.v.data]

	def head_state_cb(self, state):
		self.head_state = state

	def calib_obj_flag_cb(self, flag):
		self.calib_obj_flag = flag.data


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

	def translate_pos (self, angle, dist):
		dx = dist*np.cos(angle)
		dy = dist*np.sin(angle)
		return dx, dy

	def obtain_wrist_correction(self, side):
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

		#print angle_inc, " ", front, " ", y
		return angle_inc

	####################
	##      Throw     ##
	####################

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

	def obtain_dist(self, x, y):
		return np.sqrt((x**2)+(y**2))

	def readtcfile(self):
		custom_configuration_file = self.rospack.get_path('mdb_baxter_policies')+"/config/"+rospy.get_param("~throw_param_file")
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

	def obtain_t_poly (self):
		return np.poly1d(np.polyfit(self.grip_l, self.impact_l, 4))

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

	def obtain_angle(self, pos, x, y):
		cc = pos[0] - x
		co = pos[1] - y
		#print pos[0], x, cc, pos[1], y, co
		angle_ori = np.arctan(co/cc)
		return angle_ori

	#TODO: Adapt the system to allow negative angles (right arm)
	def handle_bt(self, srv):
		#: s0 values -0.75, 0.75
		if (-1.5<=srv.sensorization.angle.data<=1.5): #(0.0<=srv.scale.data<=4.0) and (0.5<=srv.grip.data<=1.5) and
			rospy.sleep(0.5)
			#########################################
			### Needed to test the grab by itself ###
			#rospy.sleep(4)
			#self.baxter_arm.gripper_manager(srv.arm.data)
			#rospy.sleep(1)
			#########################################
			if self.baxter_arm.gripper_instate_close(srv.arm.data):
				sign = self.select_sign(srv.arm.data)
				rospy.sleep(0.5)
				shoulder_pos = (0.064, sign*0.259)
				dx, dy = self.translate_pos(srv.sensorization.angle.data, srv.sensorization.const_dist.data)
				angle = self.obtain_angle(shoulder_pos, dx, dy)
				grip = self.obtain_grip(srv.sensorization.const_dist.data)
				self.pose_du(angle, sign, srv.arm.data)
				self.retract_du(self.fixed_speed, srv.arm.data)
				self.launch_du(self.fixed_speed, grip, srv.arm.data)
				rospy.sleep(1)
				if self.baxter_arm.gripper_instate_open(srv.arm.data):
					self.baxter_arm.restore_arm_pose(srv.arm.data)
					self.baxter_arm.gripper_manager(srv.arm.data)
					self.exp_senses.assign_gripper_sense(srv.arm.data, 0.0)
					#self.exp_senses.publish_current_senses()
					return Bool(True)
				else:
					self.baxter_arm.gripper_manager(srv.arm.data)
					self.baxter_arm.restore_arm_pose(srv.arm.data)
					#self.exp_senses.publish_current_senses()
					return Bool(False)
			else:
				#self.exp_senses.publish_current_senses()
				return Bool(False)
		else:
			#self.exp_senses.publish_current_senses()
			return Bool(False)

	####################
	##      Push      ##
	####################

	def limit_correction (self, angle):
		new_angle = angle
		if (angle<-3.14):
			#print "wipe-"
			new_angle = angle + 6.28
		if (angle>3.14):
			#print "wipe+"
			new_angle = angle - 6.38

		if (new_angle>3.059):
			new_angle = 3.05
		if (new_angle < -3.059):
			new_angle = -3.05
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
							self.grab_total_inc = 0.0

							current_angles = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()
							current_angles[0] = np.sign(self.select_sign(srv.arm.data))*0.35
							self.baxter_arm.move_joints_directly(current_angles, 'moveit', srv.arm.data, True, 1.0)

							self.baxter_arm.update_data()

							rospy.set_param("/baxter_sense", True)
							rospy.sleep(2)
							rospy.delete_param("/baxter_sense")
							rospy.sleep(2)

							return True
						else:
							return False
					else:
						return False
				else:
					print "Adjusting"
					self.loop_tries -= 1
					self.push_loop(srv, x, y, dx, dy)
			else:
				return False
		else:
			return False


	def cartesian_to_push (self, d_angle, d_dist, o_angle, o_dist):
		dx = d_dist*np.cos(d_angle)
		dy = d_dist*np.sin(d_angle)
		ox = o_dist*np.cos(o_angle)
		oy = o_dist*np.sin(o_angle)

		return dx - ox, dy - oy

	def polar_to_push (self, co, cc):
		angle = np.arctan(co/cc)
		dist = np.sqrt((co**2)+(cc**2))

		if cc < 0.0 and co < 0.0:
			angle += -3.1416
		if cc < 0.0 and co > 0.0:
			angle += 3.1416
		return angle, dist

	def second_push(self, srv, o_dist, new_angle):
		obx, oby = self.translate_pos(new_angle, o_dist)
		dx, dy = self.cartesian_to_push(srv.dest_sens.angle.data, srv.dest_sens.const_dist.data, self.exp_senses.obj_sense.angle.data, self.exp_senses.obj_sense.dist.data)
		d_angle, d_dist = self.polar_to_push(dy, dx)
		far = self.g_highpoly(abs(new_angle))
		odx, ody = self.translate_pos(d_angle, -srv.radius.data-0.02)

 		if self.regular_push (obx+odx, oby+ody, dx, dy, srv, d_angle):
			return True
		return False

	def regular_push (self, fx, fy, dx, dy, srv, d_angle):
		if self.baxter_arm.move_xyz(fx, fy, srv.obj_sens.height.data+self.safe_operation_height, False, "current", srv.arm.data, srv.scale.data, 0.95):
			#Orient the gripper towards the object
			angle_correction = self.obtain_wrist_correction(srv.arm.data)
			current_angles = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()
			angle = -d_angle+current_angles[6]-angle_correction
			final_angle = self.limit_correction(angle)
			current_angles[6] = final_angle
			self.baxter_arm.move_joints_directly(current_angles, 'moveit', srv.arm.data, True, 2.0)
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

		odx, ody = self.translate_pos(f_angle, -srv.radius.data)
		self.pose_grab_far(srv.arm.data, obx + odx, oby + ody)

		#Orient the gripper towards the object
		angle_correction = self.obtain_wrist_correction(srv.arm.data)
		current_angles = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()
		angle = -f_angle+current_angles[6]-angle_correction
		final_angle = self.limit_correction(angle)
		print "final_angle: ", final_angle
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', srv.arm.data, True, 2.0)
		self.baxter_arm.update_data()

		if self.push_loop(srv, obx + odx, oby + ody, fx, fy):
			self.baxter_arm.restore_arm_pose(srv.arm.data)
			self.baxter_arm.update_data()
			if self.second_push(srv, dist-0.01, new_angle):
				return True
		return False

	def handle_bp(self, srv):
		self.baxter_arm.update_data()

		print "radius: ", srv.radius.data

		if self.baxter_arm.gripper_instate_close("left") or self.baxter_arm.gripper_instate_close("right"):
			#self.exp_senses.publish_current_senses()
			return Bool(False)

		#Close the gripper
		print "grip_data:", srv.grip.data
		if self.baxter_arm.gripper_instate_open(srv.arm.data) and srv.grip.data:
			self.baxter_arm.gripper_manager(srv.arm.data)
			rospy.sleep(0.5)

		obx, oby = self.translate_pos(srv.obj_sens.angle.data, srv.obj_sens.const_dist.data)
		dx, dy = self.cartesian_to_push(srv.dest_sens.angle.data, srv.dest_sens.const_dist.data, srv.obj_sens.angle.data, srv.obj_sens.const_dist.data)
		d_angle, d_dist = self.polar_to_push(dy, dx)
		far = self.g_highpoly(abs(srv.obj_sens.angle.data))

		op = self.select_opposite(srv.arm.data)
		current_angles = self.baxter_arm.choose_arm_group(op).get_current_joint_values()
		current_angles[0] = np.sign(self.select_sign(op))*0.55
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', op, True, 1.0)
		self.baxter_arm.update_data()

		if srv.obj_sens.const_dist.data < far:
			print "push close"
			odx, ody = self.translate_pos(d_angle, -srv.radius.data)
			if self.regular_push (obx+odx, oby+ody, dx, dy, srv, d_angle):
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				#self.exp_senses.publish_current_senses()
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				#self.exp_senses.publish_current_senses()
				return Bool(False)
		else:
			print "push far"
			if self.first_push(far, srv, obx, oby):
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				self.loop_tries = 5
				#self.exp_senses.publish_current_senses()
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose('both')
				if srv.grip.data:
					self.baxter_arm.gripper_manager(srv.arm.data)
				self.loop_tries = 5
				#self.exp_senses.publish_current_senses()
				return Bool(False)


	##################
	##     Grab     ##
	##################

	def readgcfile(self):
		custom_configuration_file = self.rospack.get_path('mdb_baxter_policies')+"/config/"+rospy.get_param("~grab_param_file")
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

	def obtain_poly (self, x, y, g):
		return np.poly1d(np.polyfit(x, y, g))

	def obtain_object_orientation(self, srv, x, y):
		pos = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose.position
		cc = x - pos.x
		co = y - pos.y
		angle_ori = np.arctan(co/cc)
		return angle_ori

	def adjust_w2(self, srv, x, y):
		angle_correction = self.obtain_wrist_correction(srv.arm.data)
		current_angles = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()
		angle = current_angles[6]-angle_correction
		current_angles[6] = angle
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', srv.arm.data, True, 1.0)

	def adjust_w1(self, arm, x, y):
		#Stretch arm through w1
		current_p = self.baxter_arm.choose_arm_group(arm).get_current_pose().pose.position

		#2D distance between object and effector
		dist_oe = np.sqrt((x-current_p.x)**2 + (y-current_p.y)**2)

		#Angle increment
		angle_inc = np.arctan(dist_oe/(current_p.z-(-0.03)))

		#Adjust w1 joint
		current_ang = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()

		#print "angle_increment: ", angle_inc
		current_ang[5] += -angle_inc

		###Check if angles within limits
		#TODO: Check this when changing the angle initially
		if current_ang[5] > -1.5:
			if current_ang[6] < -3.059:
				current_ang[6] = -3.05
			elif current_ang[6] > 3.059:
				current_ang[6] = 3.05

			self.baxter_arm.move_joints_directly(current_ang, 'moveit', arm, True, 2.5)
			self.baxter_arm.update_data()
			return True
		else:
			return False

	def adjust_s0(self, srv, x, y):
		shoulder_pos = (0.064, self.select_sign(srv.arm.data)*0.259)
		pos = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose.position
		co_grip = pos.x - shoulder_pos[0]
		cc_grip = pos.y - shoulder_pos[1]
		co_obj = x - shoulder_pos[0]
		cc_obj = y - shoulder_pos[1]
		grip_ang = np.arctan(co_grip/cc_grip)
		obj_ang = np.arctan(co_obj/cc_obj)

		#Adjust s0 joint
		current_ang = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()
		current_ang[0] += grip_ang-obj_ang
		self.baxter_arm.move_joints_directly(current_ang, 'moveit', srv.arm.data, True, 2.0)

	def grab_loop(self, srv, x, y, first):
		if self.loop_tries > 0:
			if self.adjust_w1(srv.arm.data, x, y):
				if self.baxter_arm.move_xyz(x, y, srv.object_position.height.data + self.safe_operation_height, False, 'current', srv.arm.data, srv.scale.data, 1.0):
					if self.baxter_arm.move_xyz(x, y, srv.object_position.height.data, True, 'current', srv.arm.data, srv.scale.data, 1.0):
						if self.baxter_arm.move_xyz(x, y, srv.object_position.height.data + self.safe_operation_height, False, 'current', srv.arm.data, srv.scale.data, 1.0):
							self.loop_tries = 5
							self.grab_total_inc = 0.0
							return True
						else:
							return False
					else:
						return False
				else:
					print "Adjusting"
					self.loop_tries -= 1
					self.grab_loop(srv, x, y, first)
			else:
				return False
		else:
			return False

	def pose_grab_far (self, arm, x, y):
		sign = self.select_sign(arm)
		shoulder_pos = (0.064, sign*0.259)
		s0 = self.obtain_angle(shoulder_pos, x, y)
		angles = [self.select_offset(arm)+s0, -0.9, sign*0.0, 1.0, sign*0.0, 1.27, sign*0.0]
		self.baxter_arm.move_joints_directly(angles, 'moveit', arm, True, 1.0)
		self.baxter_arm.update_data()

	def normal_reach_grab (self, dx, dy, srv, first):
		result = False
		if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
			if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data, True, srv.orientation.data, srv.arm.data, srv.scale.data, 0.5):
				if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
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

	def far_reach_grab (self, dx, dy, srv, sens, far, first):
		result = False
		self.pose_grab_far(srv.arm.data, dx, dy)
		if self.grab_loop(srv, dx, dy, first):
			if self.baxter_arm.choose_gripper_state(srv.arm.data) and self.exp_rec == "ltm":
				new_angle = sens.angle.data
				if 0.35 > new_angle > -0.35:
					new_angle = np.sign(new_angle)*0.35
				xf, yf = self.translate_pos(new_angle, far-0.01)
				xf_r, yf_r = self.translate_pos(new_angle, far+0.10)
				if self.baxter_arm.move_xyz(xf, yf, srv.object_position.height.data, True, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
					pose = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose
					(rl, pl, yl) = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
					(xo, yo, zo, wo) = tf.transformations.quaternion_from_euler(rl, 0.0, yl)
					pose.orientation.x = xo
					pose.orientation.y = yo
					pose.orientation.z = zo
					pose.orientation.w = wo

					if self.baxter_arm.move_xyz(xf_r, yf_r, srv.object_position.height.data + 0.18, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):

						current_angles = self.baxter_arm.choose_arm_group(srv.arm.data).get_current_joint_values()
						current_angles[0] = np.sign(self.select_sign(srv.arm.data))*0.35
						self.baxter_arm.move_joints_directly(current_angles, 'moveit', srv.arm.data, True, 1.0)

						rospy.set_param("/baxter_sense", True)
						rospy.sleep(2)
						if self.exp_rec == "ltm":
							rospy.delete_param("/baxter_sense")
							rospy.sleep(2)
						self.baxter_arm.restore_arm_pose(srv.arm.data)
						#normal_reach_grab

						xf, yf = self.translate_pos(self.exp_senses.obj_sense.angle.data, self.exp_senses.obj_sense.dist.data)
						if self.normal_reach_grab (xf, yf, srv, first):
							print "far->normal_grab"
							result = True
			else:
				result = True
		return result

	def assign_grab_grip (self, side, first):
		if first:
			self.exp_senses.assign_gripper_sense(side, 0.0)
		else:
			self.exp_senses.assign_gripper_sense(side, 1.0)

	def handle_bg_prev(self, srv):
		#self.initialize_wrist(srv.arm.data)
		self.baxter_arm.update_data()
		sens = srv.object_position
		head_state = self.baxter_arm.get_hs(Bool(True))
		dx, dy = self.translate_pos(sens.angle.data, sens.const_dist.data)
		far = self.g_highpoly(abs(sens.angle.data))

		first = self.baxter_arm.choose_gripper_state(srv.arm.data)
		print "first: ", first

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
				#self.exp_senses.publish_current_senses()
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				#self.exp_senses.publish_current_senses()
				return Bool(False)

		elif sens.const_dist.data > far:
			print "Far reach"
			if self.far_reach_grab(dx, dy, srv, sens, far-0.05, first):
				if self.exp_rec == "ltm":
					self.baxter_arm.restore_arm_pose(srv.arm.data)
				self.assign_grab_grip(srv.arm.data, first)
				#self.exp_senses.publish_current_senses()
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
				#self.exp_senses.publish_current_senses()
				self.loop_tries = 5
				return Bool(False)

	def normal_reach_grab_measurement (self, dx, dy, srv, first):
		result = False
		if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
			if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data, False, srv.orientation.data, srv.arm.data, srv.scale.data, 0.5):
				if self.baxter_arm.move_xyz(dx, dy, srv.object_position.height.data + self.safe_operation_height, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
					result = True
		return result

	def handle_bg_reach(self, srv):
		self.baxter_arm.update_data()
		sens = srv.object_position
		dx, dy = self.translate_pos(sens.angle.data, sens.const_dist.data)
		first = self.baxter_arm.choose_gripper_state(srv.arm.data)

		if self.normal_reach_grab_measurement (dx, dy, srv, first):
			self.baxter_arm.restore_arm_pose(srv.arm.data)
			self.assign_grab_grip(srv.arm.data, first)
			return Bool(True)
		else:
			self.baxter_arm.restore_arm_pose(srv.arm.data)
			if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
				self.baxter_arm.gripper_manager(srv.arm.data)
			return Bool(False)

	####################
	##     Change     ##
	####################

	def first_pose_both_arms (self, pos):
		#print pos
		angles = [0.2401, -0.9491, -0.6957, 2.074, -1.022, 1.4841, 0.9211, -0.2401, -0.9491, +0.6957, 2.074, +1.022, 1.4841, -0.9211]
		angles[pos]+=1.57
		#angles[pos]-=1.57
		self.baxter_arm.move_joints_directly(angles, 'moveit', 'both', True, 1.0)

	def approach_arm (self, side, sign):
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
	def handle_bc(self, srv):
		if (srv.request.data == True):
			approach_dist = 0.22 #0.235

			### Testing code ###
			#rospy.sleep(4.0)
			#self.baxter_arm.gripper_manager("left")
			#rospy.sleep(0.5)
			####################

			if self.baxter_arm.gripper_instate_close("left") or self.baxter_arm.gripper_instate_close("right"):
				order = self.select_order()
				if order:
					self.first_pose_both_arms(self.select_wrist_angle(order[0]))
					if self.approach_arm(order[1], self.select_sign(order[0])*approach_dist):
						self.baxter_arm.gripper_manager(order[1])
						rospy.sleep(0.4)
						self.baxter_arm.gripper_manager(order[0])
						rospy.sleep(0.25)
						#if self.mode == "real" and self.baxter_arm.gripper_is_grip(order[0]) and not self.baxter_arm.gripper_is_grip(order[1]):
							#self.baxter_arm.restore_arm_pose("both")
							#self.exp_senses.publish_current_senses()
							#return Bool(False)
						if self.approach_arm(order[1], self.select_sign(order[1])*approach_dist):
							self.baxter_arm.restore_arm_pose("both")
							self.exp_senses.assign_gripper_sense(order[0], 0.0)
							self.exp_senses.assign_gripper_sense(order[1], 1.0)
							#self.exp_senses.publish_current_senses()
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
							#self.exp_senses.publish_current_senses()
							return Bool(False)
					else:
						self.baxter_arm.restore_arm_pose("both")
						#self.exp_senses.publish_current_senses()
						return Bool(False)
				else:
					#self.exp_senses.publish_current_senses()
					return Bool(False)
			else:
				#self.exp_senses.publish_current_senses()
				return Bool(False)
		else:
			#self.exp_senses.publish_current_senses()
			return Bool(False)

	#################################
	##     Grab with both arms     ##
	#################################

	def choose_dual_checker(self, arg):
		options = {
			'sim':self.check_dual_grab_sim,
            'real':self.check_dual_grab,
		}
		return options[arg]

	def check_dual_grab_sim (self):
		try:
			state = self.get_state_srv('exp_big_obj', 'table')
			if state.pose.position.z > 0.02:
				return True
			else:
				return False
		except rospy.ServiceException, e:
			rospy.loginfo("Get Model State service call failed: {0}".format(e))


	def check_dual_grab (self):
		self.baxter_arm.update_data()
		if self.baxter_arm.choose_arm_state('left').current_es.wrench.force.y < -4.5 and self.baxter_arm.choose_arm_state('right').current_es.wrench.force.y > 4.5:
			return True
		else:
			return False

	def grab_configuration(self):
		#joints_fst = [-0.4019029664259784, -0.58603403895642884, -0.7662234035487642, 1.0085923680346596, 0.6658399083517928, 1.3422865673839378, 1.2785729867024924, 0.4019029664259784, -0.58603403895642884, 0.7662234035487642, 1.0085923680346596, -0.6658399083517928, 1.3422865673839378, -1.2785729867024924]
		joints_fst = [-0.4019029664259784, -0.58603403895642884, -1.0662234035487642, 1.0085923680346596, 0.8658399083517928, 1.5422865673839378, 1.0785729867024924, 0.4019029664259784, -0.58603403895642884, 1.0662234035487642, 1.0085923680346596, -0.8658399083517928, 1.5422865673839378, -1.0785729867024924]
		self.baxter_arm.move_joints_directly(joints_fst, 'moveit', 'both', True, 1.0)
		self.baxter_arm.update_data()

	def adjust_position2(self, x, y, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points = [cles.x, cles.y, z, cres.x, cres.y, z]
		if self.baxter_arm.move_to_position_goal_both(points, True, 1.0):
			return True
		return False

	def adjust_position1(self, x, y, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points = [x, y+(size/2.0)+0.075, cles.z, x, y-(size/2.0)-0.075, cles.z]
		if self.baxter_arm.move_to_position_goal_both(points, True, 1.0):
			return True
		return False

	def adjust_orientation (self):
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

	def move_both_cartesian (self, inc):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points = [cles.x,cles.y,cles.z,cres.x,cres.y,cres.z]
		points_f = [sum(x) for x in zip(inc, points)]
		if self.baxter_arm.move_xyz_execute(points_f, False, 'current', 1.0, 1.0):
			return True
		return False

	#NOTE: Removed the message's success variable management
	def handle_bgb(self, srv):
		if not (self.baxter_arm.choose_gripper_state("left") and self.baxter_arm.choose_gripper_state("right")) and not self.choose_dual_checker(self.mode)():
			obx, oby = self.translate_pos(srv.sensorization.angle.data, srv.sensorization.const_dist.data)
			obz = srv.sensorization.height.data
			self.grab_configuration()
			self.baxter_arm.update_data()
			if self.adjust_position1(obx, oby, obz, srv.size.data):
				if self.adjust_position2(obx, oby, obz, srv.size.data):
					if self.adjust_orientation():
						#self.baxter_arm.gripper_manager("left")
						#self.baxter_arm.gripper_manager("right")
						if self.move_both_cartesian([0.0, -0.125, 0.0, 0.0, +0.125, 0.0]):
							#print self.choose_dual_checker(self.mode)()
							while not self.choose_dual_checker(self.mode)():
								self.move_both_cartesian([0.0, -0.02, 0.0, 0.0, +0.02, 0.0])
							if self.move_both_cartesian([0.0, 0.0, 0.10, 0.0, 0.0, 0.10]):
								#while not self.choose_dual_checker(self.mode)():
									#self.move_both_cartesian([0.0, -0.02, 0.0, 0.0, +0.02, 0.0])
								#if self.choose_dual_checker(self.mode)():
								self.exp_senses.assign_gripper_sense('both', 1.0)
									#self.exp_senses.publish_current_senses()
								return Bool(True)
			self.baxter_arm.restore_arm_pose("both")
			#self.exp_senses.publish_current_senses()
			return Bool(False)
		else:
			#self.exp_senses.publish_current_senses()
			return Bool(False)

	#################################
	##     Drop with both arms     ##
	#################################

	def adquire_drop_pos (self, dx, dy, z, size):
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position

		div = 0.05

		initx = (cles.x+cres.x)/2
		inity = (cles.y+cres.y)/2

		distance = self.obtain_dist(dx-initx, dy-inity)
		iterations = distance/div

		incx = (dx-initx)/iterations
		incy = (dy-inity)/iterations

		#remx = (dx-initx)%iterations
		#remy = (dy-inity)%iterations

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


	def move_object(self, dx, dy, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		d = cles.y - cres.y
		points_f = [dx, dy+(d/2.0), cles.z, dx, dy-(d/2.0), cres.z]
		if self.baxter_arm.move_to_position_goal_both(points_f, True, 1.0):
			return True
		return False

	def drop_object(self, dx, dy, z, size):
		self.baxter_arm.update_data()
		cles = self.baxter_arm.choose_arm_state('left').current_es.pose.position
		cres = self.baxter_arm.choose_arm_state('right').current_es.pose.position
		points_f = [cles.x, cles.y, z, cres.x, cres.y, z]
		if self.baxter_arm.move_xyz_execute(points_f, False, 'current', 1.0, 1.0):
			return True
		return False

	def handle_bdb(self, srv):
		#if self.choose_dual_checker(self.mode)():
		dx, dy = self.translate_pos(srv.destination.angle.data, srv.destination.const_dist.data)
		#if self.move_object(dx, dy, srv.destination.height.data, srv.size.data):
		if self.adquire_drop_pos(dx, dy, srv.destination.height.data, srv.size.data):
			print "adquire_drop_pos"
			if self.drop_object(dx, dy, srv.destination.height.data, srv.size.data):
				print "drop_object"
				if self.move_both_cartesian([0.0, 0.10, 0.0, 0.0, -0.10, 0.0]):
					print "separate arms"
					if self.move_both_cartesian([0.0, 0.0, 0.2, 0.0, 0.0, 0.2]):
						print "up arms"
						if self.move_object(dx, 0.0, 0.25, srv.size.data):
							if not self.choose_dual_checker(self.mode)():
								self.baxter_arm.restore_arm_pose('both')
								self.exp_senses.assign_gripper_sense('both', 0.0)
								#self.exp_senses.publish_current_senses()
				return Bool(True)

		self.baxter_arm.restore_arm_pose('both')
		#self.exp_senses.publish_current_senses()
		return Bool(False)
		#else:
			#self.exp_senses.publish_current_senses()
			#return Bool(False)

	##################
	##     Give     ##
	##################

	def handle_bgm(self, srv):
		if (srv.request.data == True):
			self.baxter_display.changeDisplay("giveme")
			rospy.sleep(10)
			#self.exp_senses.publish_current_senses()
			return Bool(True)
		return Bool(False)

	##############################
	##     Restore position     ##
	##############################
	def handle_brap(self, srv):
		self.baxter_arm.restore_arm_pose(srv.arm.data)
		return Bool(True)

	#######################
	##    Change face    ##
	#######################
	def handle_bcf(self, srv):
		self.baxter_display.changeDisplay(srv.expression.data)
		return Bool(True)

	##########################
	##    Open arms pose    ##
    ##########################
	def adopt_oap(self):
		pose = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673, -1.7000342081740099, -0.9986214929134043, 1.1876846250202815, 1.9378012302962488, -0.6680486331240977, 1.0339030510347689, 0.49777676566881673]
		self.baxter_arm.move_joints_directly(pose, 'moveit', 'both', True, 1.0)
		self.baxter_arm.update_data()

	def handle_oap(self, srv):
		if srv.request.data:
			self.adopt_oap()
			#self.baxter_arm.both_group.clear_path_constraints()
			return Bool(True)
		return Bool(False)

	def test_position_constraint(self):
		position_constraints = []
		# print "End effector position: ", self.baxter_arm.larm_group.get_current_pose('left_gripper')

		pos = self.baxter_arm.larm_state.current_es.pose.position
		ori = self.baxter_arm.larm_state.current_es.pose.orientation
		pcr = self.baxter_arm.position_constraint_region(SolidPrimitive.BOX, [1.22 + 0.22 + 1.0, 2.442, 0.20])
		pcp = self.baxter_arm.position_constraint_pose([1.0, 0.83, 0.0, -0.05 + 0.0095 + pos.z])
		# pcp = self.baxter_arm.position_constraint_pose([ori.w, pos.x, pos.y, pos.z])
		print pos, ori

		pc = self.baxter_arm.position_constraints([pcr],[pcp], 1.0,'left')
		self.baxter_arm.add_position_constraints([pc], 'left')

		# print self.baxter_arm.larm_group.get_current_joint_values()

		# self.baxter_arm.move_joints_directly(pose, 'moveit', 'left', True, 1.0)
		self.baxter_arm.larm_group.set_start_state_to_current_state()
		self.baxter_arm.larm_group.set_planning_time(100.0)

	def test_orientation_constraint(self):
		ori = self.baxter_arm.rarm_group.get_current_rpy()
		quat = tf.transformations.quaternion_from_euler(ori[0], ori[1], ori[2])

		oc = self.baxter_arm.orientation_constraint([3.14, 3.14, 3.14], quat, 1.0, 'left')
		self.baxter_arm.add_orientation_constraints([oc],'left')

		self.baxter_arm.larm_group.set_planning_time(120.0)

	def handle_olap(self, srv):
		pose = [1.7000342081740099, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673]
		if srv.request.data:
			#self.test_orientation_constraint()
			self.baxter_arm.move_joints_directly(pose, 'moveit', 'left', True, 1.0)
			self.baxter_arm.update_data()
			#self.baxter_arm.remove_path_constraints('left')

			return Bool(True)
		return Bool(False)

	def handle_orap(self, srv):
		pose = [-1.7000342081740099, -0.9986214929134043, 1.1876846250202815, 1.9378012302962488, -0.6680486331240977, 1.0339030510347689, 0.49777676566881673]
		if srv.request.data:
			self.baxter_arm.move_joints_directly(pose, 'moveit', 'right', True, 1.0)
			self.baxter_arm.update_data()
			return Bool(True)
		return Bool(False)

	##########################
	##    Operation pose    ##
    ##########################
	def handle_op(self, srv):
		#pose = [0.16336895390979655, -0.9986214929134043, -1.1876846250202815, 1.9378012302962488, 0.6680486331240977, 1.0339030510347689, -0.49777676566881673, -0.16336895390979655, -0.9986214929134043, 1.1876846250202815, 1.9378012302962488, -0.6680486331240977, 1.0339030510347689, 0.49777676566881673]
		pose = [0.46336895390979655, -0.9986214929134043, -1.4876846250202815, 1.9378012302962488, 0.5680486331240977, 1.2339030510347689, -0.49777676566881673, -0.46336895390979655, -0.9986214929134043, 1.4876846250202815, 1.9378012302962488, -0.5680486331240977, 1.2339030510347689, 0.49777676566881673]
		if srv.request.data:
			self.baxter_arm.move_joints_directly(pose, 'moveit', 'both', True, 1.0)
			self.baxter_arm.update_data()
			self.baxter_arm.update_init_data()
			return Bool(True)
		return Bool(False)

	##########################
	##    Reset Grippers	##
	##########################

	def handle_bsrg(self, srv):
		if (srv.request.data == True):
			self.baxter_arm.lgripper.open()
			self.baxter_arm.rgripper.open()
			self.baxter_arm.lgripper_state = False
			self.baxter_arm.rgripper_state = False
			self.exp_senses.assign_gripper_sense('both', 0.0)
			return Bool(True)
		return Bool(False)

	##########################
	##    Check close reach	##
	##########################
	def handle_bcrc (self, srv):
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
	def handle_bfrc (self, srv):
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
	def handle_bes(self, srv):
		if (srv.request.data == True):
			self.exp_senses.publish_current_senses(srv.box_rad.data, srv.obj_rad.data)
			return Bool(True)
		return Bool(False)

	######################
	##	Initialization	##
	######################
	def initialize_wrist (self, side):
		angle_correction = self.obtain_wrist_correction(side)
		current_angles = self.baxter_arm.choose_arm_group(side).get_current_joint_values()
		current_angles[6] -= angle_correction
		current_angles[6] = self.limit_correction(current_angles[6])
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', side, True, 2.0)
		self.baxter_arm.update_data()

	##########################
	##	Cartesian Movement  ##
	##########################
	def orient_gripper (self, arm, f_angle):
		angle_correction = self.obtain_wrist_correction(arm)
		current_angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()
		angle = -f_angle+current_angles[6]-angle_correction
		final_angle = self.limit_correction(angle)
		current_angles[6] = final_angle
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', arm, True, 2.0)

	def translate_request (self, req, ang):
		cx = req.const_dist.data*np.cos(ang)
		cy = req.const_dist.data*np.sin(ang)
		cz = req.height.data
		return cx, cy, cz

	def angle_fix (self, angle):
		new_angle = angle
		if (angle<-3.14):
			new_angle = angle + 6.28
		if (angle>3.14):
			new_angle = angle - 6.28
		return new_angle

	def aim_gripper (self, arm, angle):
		#Orient the gripper towards the destination
		angle_correction = self.obtain_wrist_correction(arm)
		current_angles = self.baxter_arm.choose_arm_group(arm).get_current_joint_values()

		current_angles[6]-= angle
		current_angles[6] = self.limit_correction(current_angles[6])
		print "Aim gripper towards the angle ", -(angle_correction - angle)
		self.baxter_arm.move_joints_directly(current_angles, 'moveit', arm, True, 2.0)
		self.baxter_arm.update_data()

	def update_gripper_orientation (self, arm='right'):
		self.exp_senses.rgrip_ori = -self.obtain_wrist_correction(arm)
		print "Current gripper orientation: ", self.exp_senses.rgrip_ori

	def handle_bcm(self, srv):
		if srv.valid.data == True:
			#Update the current gripper orientation
			self.update_gripper_orientation()

			#Orient the gripper towards the destination
			self.aim_gripper(srv.arm.data, srv.dest.angle.data)

			#Update the current gripper orientation
			self.update_gripper_orientation()

			print "Baxter prev angle: ", self.exp_senses.rgrip_ori,

			#Move the gripper towards the destination
			pos = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose.position
			#self.exp_senses.rgrip_ori += srv.dest.angle.data
			self.exp_senses.rgrip_ori = self.angle_fix(self.exp_senses.rgrip_ori)

			print "Baxter action orientation: ", self.exp_senses.rgrip_ori
			#self.rgrip_ori = self.limit_correction(self.rgrip_ori)
			x,y,z = self.translate_request(srv.dest, self.exp_senses.rgrip_ori)
			#print self.rgrip_ori, x, y, z

			result = False
			if self.baxter_arm.move_xyz(pos.x+x, pos.y+y, pos.z+z, False, srv.orientation.data, srv.arm.data, srv.scale.data, 1.0):
				self.baxter_arm.update_data()
				#Update the current gripper orientation
				self.update_gripper_orientation()
				result = True

			return Bool(result)
		return Bool(True)

	def handle_sa(self, srv):
		self.baxter_arm.update_data()
		current_angles_prev = self.baxter_arm.choose_arm_group('right').get_current_joint_values()
		orap = [-1.7000342081740099, -0.9986214929134043, 1.1876846250202815, 1.9378012302962488, -0.6680486331240977, 1.0339030510347689, 0.49777676566881673]
		self.baxter_arm.move_joints_directly(orap, 'moveit', 'right', True, 1.0)
		rospy.sleep(2)
		rospy.delete_param("/baxter_sense")
		self.baxter_arm.move_joints_directly(current_angles_prev, 'moveit', 'right', True, 1.0)
		rospy.sleep(2)
		self.baxter_arm.update_data()
		return Bool(True)

	def handle_baxter_cartesian_fm_mov (self, srv):
		self.baxter_arm.update_data()
		dx, dy = self.translate_pos(srv.angle.data, srv.dist.data)
		pos = self.baxter_arm.choose_arm_state(srv.arm.data).current_es.pose.position
		if self.baxter_arm.move_xyz(pos.x+dx, pos.y+dy, 0.1, False, "init", srv.arm.data, 1.0, 1.0):
			self.baxter_arm.update_data()
			return Bool(True)
		return Bool(False)

	##################################
	###		    CANDIDATES         ###
	##################################

	def translate_req (self, dist, ang):
		cx = req.const_dist.data*np.cos(ang)
		cy = req.const_dist.data*np.sin(ang)
		cz = req.height.data
		return cx, cy, cz

	def baxter_candidates(self, srv, arm="right", distance=0.07):
		baxter_l_angle = np.random.uniform(srv.limit.data, -srv.limit.data)
		if self.check_baxter_validity(baxter_l_angle, arm, distance):
			return Int32(baxter_l_angle), Bool(True)
		else:
			return Int32(baxter_l_angle), Bool(False)

	def check_baxter_validity(self, baxter_angle, arm="right", distance=0.07):
		angle_to_check = self.exp_senses.rgrip_ori
		angle_to_check += math.radians(baxter_angle)
		angle_to_check = self.angle_fix(angle_to_check)

		pos = self.baxter_arm.choose_arm_state(arm).current_es.pose.position
		x,y = self.translate_pos(angle_to_check, distance)
		resp = self.baxter_arm.move_xyz_plan(pos.x+x, pos.y+y, pos.z, "current", arm, 1.0)

		if ((pos.x+x > self.ra_area_l[0]) and (pos.x+x<self.ra_area_l[1]) and (pos.y+y<self.ra_area_l[2]) and (pos.y+y>self.ra_area_l[3])) and resp != False:
			return True
		else:
			return False

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

			print "Action number: ", candidate_n, " baxter_valid: ", candidate.baxter_valid.data, " robobo_valid: ", candidate.robobo_valid.data
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
		custom_configuration_file = self.rospack.get_path('mdb_baxter_policies')+"/config/"+rospy.get_param("~motiven_param_file")
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

	def handle_bd(self, srv):
		self.baxter_arm.update_data()
		sens = srv.object_position
		dx, dy = self.translate_pos(sens.angle.data, sens.const_dist.data)
		far = self.right_arm_motiven_poly(abs(sens.angle.data))

		first = self.baxter_arm.choose_gripper_state(srv.arm.data)

		print "\nObject distance: ", sens.const_dist.data, "Limit: ", far

		if sens.const_dist.data < far:
			print "Normal drop"
			if self.normal_reach_drop(dx, dy, srv):
				self.assign_grab_grip(srv.arm.data, first)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(False)

		elif sens.const_dist.data > far:
			print "Far drop"
			if self.far_reach_drop(dx, dy, srv):
				self.assign_grab_grip(srv.arm.data, first)
				return Bool(True)
			else:
				self.baxter_arm.restore_arm_pose(srv.arm.data)
				if first != self.baxter_arm.choose_gripper_state(srv.arm.data):
					self.baxter_arm.gripper_manager(srv.arm.data)
				return Bool(False)

	#####################################
	##   Camera position calibration   ##
	#####################################

	def quadrilateral_grid_points (self, vertex, grid_points, side):

		points = int(math.sqrt(grid_points) - 1)
		w2_inc = [0.785, -0.3925]
		grid = [] 
	
		cont = 0
		for j in range (0, points+1):
			for i in range (0, points+1):
				a0 = float ((points-i)*(points-j))/float(points**2)
				a1 = float ((i)*(points-j))/float(points**2)
				a2 = float ((i)*(j))/float(points**2)
				a3 = float ((points-i)*(j))/float(points**2)

				grid.append(
				[[a0*vertex[0][0] + a1*vertex[1][0] + a2*vertex[2][0] + a3*vertex[3][0],
				a0*vertex[0][1] +a1*vertex[1][1] + a2*vertex[2][1] + a3*vertex[3][1]],
				math.atan2(i,j)*side,
				w2_inc[cont%2]]
				)
				print j, i, cont, w2_inc[cont%2]
				cont+=1
			w2_inc[:] = reversed(w2_inc[:])

		side_points = points+1
		for it in xrange(1,side_points,2):
			grid[side_points*it:side_points*it+side_points] = reversed(grid[side_points*it:side_points*it+side_points])

		return grid

	def select_vertex (self, arg):
		options = {
			'left':[[0.211943,0.610663],[0.5841,0.0742],[1.0156, 0.3568],[0.4137, 1.0421]],
			'hleft':[[0.33,0.48],[0.57,-0.14],[1.05,-0.03],[0.89,0.87]],
			'front':[[0.47,0.32],[0.47,-0.27],[1.0,-0.37],[1.0,0.42]],
			'right':[[0.53,-0.04],[0.351943,-0.6],[0.43,-1.0],[1.1,-0.25]],
		}
		return options[arg]

	def select_offset (self, arg):
		options = {
			'left':-0.785,
			'hleft':-0.3925,
			'front':0.0,
			'right':+0.785,
		}
		return options[arg]

	def select_arm_to_calibrate (self, arg):
		options = {
			'left':'left',
			'hleft':'left',
			'front':'left',
			'right':'right',
		}
		return options[arg]

	def obtain_roll_list (self, side, t_points):
		points_l = int(math.sqrt(t_points))
		roll_list = []
		for j in range (0, points_l):
			for i in range (0, points_l):
				roll_list.append(math.atan2(i,j)*side)
		for it in xrange(1,points_l,2):
			roll_list[points_l*it:points_l*it+points_l] = reversed(roll_list[points_l*it:points_l*it+points_l])
		return roll_list
		
	def matrix_movement (self, req):
		hleft = False
		if req.arm.data == "hleft":
			req.arm.data = "left"
			hleft = True
			
		self.adopt_oap()				
		self.baxter_arm.restore_arm_pose(self.select_arm_to_calibrate(req.arm.data))

		if not hleft:
			vertex = self.select_vertex(req.arm.data)
			offset = self.select_offset(req.arm.data)
		else:
			vertex = self.select_vertex('hleft')
			offset = self.select_offset('hleft')			

		grid_points = self.quadrilateral_grid_points(vertex, req.point_number.data, self.select_sign(req.arm.data))
		grid_points[0][1]=1.57*self.select_sign(req.arm.data)

		if req.arm.data == "right":
			side_points = int(math.sqrt(req.point_number.data))
			roll_l =  self.obtain_roll_list(self.select_sign(req.arm.data), req.point_number.data)
			roll_l[0] = 1.57*self.select_sign(req.arm.data)
			for it in xrange(0,side_points,1):
				grid_points[side_points*it:side_points*it+side_points] = reversed(grid_points[side_points*it:side_points*it+side_points])
			for it in range (0, len(grid_points)):
				grid_points[it][1] = roll_l[it]

		pose_target = Pose()
		pose_target.position.z = 0.15 

		self.scene_clnt(String('table'),String('add'),Float64(0.0), Float64(0.0), Float64(0.0))

		for pose in grid_points:
			pose_target.position.x = pose[0][0]
			pose_target.position.y = pose[0][1]
			roll = pose[1]+offset
			(xf, yf, zf, wf) = tf.transformations.quaternion_from_euler(roll, 1.57, 1.57)	
			pose_target.orientation.w = wf
			pose_target.orientation.x = xf
			pose_target.orientation.y = yf 
			pose_target.orientation.z = zf

			print pose_target

			self.baxter_arm.move_to_pose_goal(pose_target, self.select_arm_to_calibrate(req.arm.data), True, 1.0)

			raw_input("Press Enter to continue")

		self.scene_clnt(String('table'),String('remove'),Float64(0.0), Float64(0.0), Float64(0.0))

		self.baxter_arm.restore_arm_pose("both")
		return Bool(True)

	#####################################
	###   Interpolation calibration   ###
	#####################################

	def lineal_quadrilateral_grid_points (self, vertex, grid_points):
		points = int(math.sqrt(grid_points) - 1)
		grid = [] 
	
		cont = 0
		for j in range (0, points+1):
			for i in range (0, points+1):
				a0 = float ((points-i)*(points-j))/float(points**2)
				a1 = float ((i)*(points-j))/float(points**2)
				a2 = float ((i)*(j))/float(points**2)
				a3 = float ((points-i)*(j))/float(points**2)

				grid.append([a0*vertex[0][0] + a1*vertex[1][0] + a2*vertex[2][0] + a3*vertex[3][0], a0*vertex[0][1] +a1*vertex[1][1] + a2*vertex[2][1] + a3*vertex[3][1]])
		return grid

	def select_arm_to_move(self, y_c):
		if y_c >= 0.0:
			return "left"
		else:
			return "right"

	def pan_to_sp (self, pos, speed):
		self.pan_pub.publish(HeadPanCommand(pos, speed, 0.0))
		rospy.sleep(0.5)
		while self.head_state.isTurning:
			pass

	def calibration_move_loop(self, height_data, side, dx, dy):			
		result = False
		if self.loop_tries > 0:
			if self.adjust_w1(side, dx, dy):
				if self.baxter_arm.move_xyz(dx, dy, height_data + self.safe_operation_height, False, 'current', side, 1.0, 1.0):
					if self.baxter_arm.move_xyz(dx, dy, height_data, False, 'current', side, 1.0, 1.0):
						raw_input("Place the object to detect in the gripper position")
						pos = self.baxter_arm.choose_arm_state(side).current_es.pose.position
						real_position = [pos.x, pos.y]
						if self.baxter_arm.move_xyz(dx, dy, height_data + self.safe_operation_height, False, 'current', side, 1.0, 1.0):
							self.loop_tries = 5
							result = real_position
				else:
					print "Adjusting"
					self.loop_tries -= 1
					self.calibration_move_loop(height_data, side, dx, dy)
		return result

	def calibration_move_close(self, height_data, side, dx, dy):
		result = False
		if self.baxter_arm.move_xyz(dx, dy, height_data + self.safe_operation_height, False, "current", side, 1.0, 1.0):
			if self.baxter_arm.move_xyz(dx, dy, height_data, False, "current", side, 1.0, 1.0):
				raw_input("Place the object to detect in the gripper position")
				pos = self.baxter_arm.choose_arm_state(side).current_es.pose.position
				real_position = [pos.x, pos.y]
				if self.baxter_arm.move_xyz(dx, dy, height_data + self.safe_operation_height, False, "current", side, 1.0, 1.0):
					return real_position
		return result

	def angle_generator (self, number_of_angles, angle_seed):
		list_of_ang = []

		if number_of_angles > 1:
			list_of_ang.append(angle_seed)
			inc = angle_seed*2/(number_of_angles-1)
			for it in range (0, number_of_angles-1):
				angle_seed-=inc
				list_of_ang.append(angle_seed)
		
		return list_of_ang

	def translate_flag (self, use_tag):
		if use_tag: return "tag"
		else: return "cylinder"

	def select_calibration_flag(self, flag):
		options = {
			"tag":self.robobo_status, 
			"cylinder":self.calib_obj_flag,
		}
		return options[flag]

	def pan_data_adquisition (self, pan_angles, use_tag):
		self.adopt_oap()
		pos_data = []
		if len(pan_angles)>1:
			for pan in pan_angles:
				self.pan_to_sp(pan, 0.05)
				rospy.sleep(1.0)
				if not self.select_calibration_flag(self.translate_flag(use_tag)): self.aruco_cent = 'None'
				pos_data.append([self.head_state.pan, self.aruco_cent])
		else:
			if not self.select_calibration_flag(self.translate_flag(use_tag)): self.aruco_cent = 'None'
			print self.aruco_cent
			pos_data.append([self.aruco_cent])
		self.pan_to_sp(0.0, 0.05)
		return pos_data

	def calibration_move_far(self, height_data, arm, dx, dy):
		self.pose_grab_far(arm, dx, dy)
		real_position = self.calibration_move_loop(height_data, arm, dx, dy)
		return real_position

	def translate_move_flag(self, dist, far):
		if dist <= far:	return "close"
		else: return "far"

	def select_calibration_move (self, flag):
		options = {
			"close":self.calibration_move_close,
			"far":self.calibration_move_far, 
		}
		return options[flag]

	def calibration_cycle (self, dist, far, pan_angles, req, calibration_data, dx, dy, arm, non_reach):
		if req.use_arm.data:
			if non_reach:
				calibration_data.append([[dx, dy], 'non_reachable'])
				return True
			else:
				real_position = self.select_calibration_move(self.translate_move_flag(dist, far))(req.height.data, arm, dx, dy)
				if real_position != False:
					rospy.sleep(1)
					pos_data = self.pan_data_adquisition(pan_angles, req.use_tag.data)
					calibration_data.append([real_position, pos_data])
					return True
				else:
					self.loop_tries = 5
					self.adopt_oap()
					calibration_data.append([[dx, dy], 'non_reachable'])
					return False
			
		else:
			rospy.sleep(1)
			pos_data = self.pan_data_adquisition(pan_angles, req.use_tag.data)
			calibration_data.append([[dx, dy], pos_data])
			self.baxter_arm.restore_arm_pose("both")
			return True

	def obtain_calibration_limits (self, req):
		if req.row_to_calibrate.data == -1:
			return 0, req.point_num.data
		else:
			side = int(math.sqrt(req.point_num.data))
			return side*req.row_to_calibrate.data, (side*req.row_to_calibrate.data)+side
			
	def interpolation_calibration (self, req):
		x_min = rospy.get_param("~cal_xmin")
		x_max = rospy.get_param("~cal_xmax")
		y_min = rospy.get_param("~cal_ymin")
		y_max = rospy.get_param("~cal_ymax")

		seed_angle = rospy.get_param("~cal_ang")

		height_data = req.height.data		
		table_vertex = [[x_min,y_max],[x_min,y_min],[x_max,y_min],[x_max,y_max]]
		
		pan_angles = self.angle_generator(req.angle_num.data, seed_angle)
		
		self.adopt_oap()				
		grid_points = self.lineal_quadrilateral_grid_points(table_vertex, req.point_num.data)

		calibration_data = []

		(init, end) = self.obtain_calibration_limits(req)

		row_point_num = int(math.sqrt(req.point_num.data))

		rospy.set_param("baxter_sense", True)

		fail = True
		non_reach = False
		for pose in range(init, end):		

			if (pose%row_point_num ) == 0:
				fail = True

			dx = grid_points[pose][0]
			dy = grid_points[pose][1]

			#Check the arm to use based on the y coordinate:
			arm = self.select_arm_to_move(dy)

			#Translate position in terms of angle and distance
			dist = self.obtain_dist(dx,dy)
			ang = math.atan2(dy,dx)
		
			far = self.g_highpoly(abs(ang))
			self.baxter_arm.restore_arm_pose(arm)

			result = self.calibration_cycle(dist, far, pan_angles, req, calibration_data, dx, dy, arm, non_reach)
			
			fail = fail and not result 
			if ((pose+1)%row_point_num) == 0 and fail:
				non_reach = True

			raw_input("\nPosition "+str(pose)+": information obtained. Press the button to calibrate the next position")

		rospy.delete_param("baxter_sense")
		self.baxter_arm.restore_arm_pose("both")
		rospy.loginfo(calibration_data)
		return Bool(True)

############################################
############################################
############################################    		

def node_shutdown():
	moveit_commander.roscpp_shutdown()

def baxter_policies_server():
	baxter_policies()
	rospy.spin()

if __name__ == '__main__':
	baxter_policies_server()
