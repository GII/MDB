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
import math
import numpy as np
from std_msgs.msg import Float64, Bool
from mdb_common.msg import SensData
from mdb_common.srv import GetSenseMotiv
from mdb_robots_policies.srv import GetSense, GetRobSense
from geometry_msgs.msg import PoseStamped

class exp_senses():
	def __init__(self, global_policies):
		self.global_policies = global_policies

		self.obj_sense = SensData()
		self.box_sense = SensData()
		self.rob_sense = SensData()
		self.rob_ori = Float64()
		self.rob_obj_ori = Float64()
		self.rob_box_ori = Float64()

		self.rgrip_ori = 0.0
		self.rob_grip = 0.0

		self.bobj_sense_sb = rospy.Subscriber("/mdb_baxter/ball", SensData, self.obj_sense_cb)
		self.box_sense_sb = rospy.Subscriber("/mdb_baxter/box", SensData, self.box_sense_cb)
		self.rob_sense_sb = rospy.Subscriber("/mdb_baxter/robobo", SensData, self.rob_sense_cb)

		self.mixed_box_sense_sb = rospy.Subscriber("/baxter_throwing/basket_pose", PoseStamped, self.mixed_box_cb)
		self.mixed_ball_sense_sb = rospy.Subscriber("/baxter_throwing/ball_position", PoseStamped, self.mixed_ball_cb)

		self.rob_ori_sb = rospy.Subscriber("/tracking/robobo_ori", Float64, self.rob_ori_cb)
		self.rob_ori_obj_sb = rospy.Subscriber("/tracking/robobo_ori_obj", Float64, self.rob_ori_obj_cb)
		self.rob_ori_box_sb = rospy.Subscriber("/tracking/robobo_ori_box", Float64, self.rob_ori_box_cb)

		self.obj_dist_pb = rospy.Publisher("/mdb/baxter/sensor/ball_dist", Float64, queue_size = 1)
		self.obj_ang_pb = rospy.Publisher("/mdb/baxter/sensor/ball_ang", Float64, queue_size = 1)
		self.obj_size_pb = rospy.Publisher("/mdb/baxter/sensor/ball_size", Float64, queue_size = 1)

		self.box_dist_pb = rospy.Publisher("/mdb/baxter/sensor/box_dist", Float64, queue_size = 1)
		self.box_ang_pb = rospy.Publisher("/mdb/baxter/sensor/box_ang", Float64, queue_size = 1)
		self.box_size_pb = rospy.Publisher("/mdb/baxter/sensor/box_size", Float64, queue_size = 1)

		self.lgrip_sense = Float64(0.0)
		self.rgrip_sense = Float64(0.0)

		self.lgrip_sense_pb = rospy.Publisher("/mdb/baxter/sensor/ball_in_left_hand", Bool, queue_size = 1)
		self.rgrip_sense_pb = rospy.Publisher("/mdb/baxter/sensor/ball_in_right_hand", Bool, queue_size = 1)

		self.gs_srver = rospy.Service('/baxter/get_sense', GetSense, self.handle_gs)
		self.gs_motiv_srver = rospy.Service('/mdb3/baxter/sensors', GetSenseMotiv, self.handle_gs_motiv)
		self.gs_rob_data_srver = rospy.Service('/baxter/get_rob_data_sens', GetRobSense, self.handle_gs_rob_data)

  	### Callbacks ###
	def mixed_box_cb(self, sense):
		if rospy.has_param("/baxter_sense"):
			do_sense = rospy.get_param("/baxter_sense")
			if do_sense:
				(angle, dist) = self.global_policies.cartesian_to_polar(sense.pose.position.y, sense.pose.position.x)
				self.box_sense.dist.data = dist-0.08
				self.box_sense.angle.data = angle
				self.box_sense.height.data = -0.04
				self.box_sense.radius.data = 0.0

	def mixed_ball_cb(self, sense):
		if rospy.has_param("/baxter_sense"):
			do_sense = rospy.get_param("/baxter_sense")
			if do_sense:
				(angle, dist) = self.global_policies.cartesian_to_polar(sense.pose.position.y, sense.pose.position.x)
				self.obj_sense.dist.data = dist
				self.obj_sense.angle.data = angle
				self.obj_sense.height.data = -0.04
				self.obj_sense.radius.data = 0.0

	def obj_sense_cb (self, sens):
		if sens.height.data > -0.10:
			self.obj_sense = sens

	def box_sense_cb (self, sens):
		if sens.height.data > -0.10 and not rospy.has_param("/check_reward"): #and rospy.has_param("/box_sense"):
			self.box_sense = sens

	def rob_sense_cb (self, sens):
		if sens.height.data > -0.10:
			self.rob_sense = sens

	def rob_ori_cb (self, ori):
		self.rob_ori = ori

	def rob_ori_obj_cb(self, ori):
		self.rob_obj_ori = ori
			
	def rob_ori_box_cb(self, ori):
		self.rob_box_ori = ori

	### Services ###		
	def handle_gs_motiv(self, srv):
		if (srv.request.data == True):
			(box_dx, box_dy) = self.global_policies.polar_to_cartesian(self.box_sense.angle.data, self.box_sense.dist.data)
			(obj_dx, obj_dy) = self.global_policies.polar_to_cartesian(self.obj_sense.angle.data, self.obj_sense.dist.data)
			(rob_dx, rob_dy) = self.global_policies.polar_to_cartesian(self.rob_sense.angle.data, self.rob_sense.dist.data)

			inc_x = 0.1*np.cos(self.rob_ori.data)
			inc_y = 0.1*np.sin(self.rob_ori.data)
			robg_x = rob_dx + inc_x
			robg_y = rob_dy + inc_y

			if self.grip_state_conversion(self.rgrip_sense.data):
				box_ball_dist = self.global_policies.obtain_dist(box_dx-self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x, box_dy-self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y)
				hand_ball_dist = 0.0
				rob_ball_dist = self.global_policies.obtain_dist(robg_x-self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x, robg_y-self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y)
			else:
				if self.rob_grip > 0.0:
					rob_ball_dist = 0.0
					box_ball_dist = self.global_policies.obtain_dist(box_dx-robg_x, box_dy-robg_y)
					hand_ball_dist = self.global_policies.obtain_dist(self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x-robg_x, self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y-robg_y)
				else:
					rob_ball_dist = self.global_policies.obtain_dist(robg_x-obj_dx, robg_y-obj_dy) 
					box_ball_dist = self.global_policies.obtain_dist(box_dx-obj_dx, box_dy-obj_dy) 
					hand_ball_dist = self.global_policies.obtain_dist(self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x-obj_dx, self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y-obj_dy)

			if self.grip_state_conversion(self.rgrip_sense.data):
				obj_dx = self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x
				obj_dy = self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y
			elif self.rob_grip > 0.0:
				obj_dx = robg_x
				obj_dy = robg_y

			print 'rob_obj: ', rob_ball_dist, 'hand_obj: ', hand_ball_dist, 'box_obj: ', box_ball_dist

			return Float64(box_ball_dist), Float64(hand_ball_dist), Float64(rob_ball_dist), Float64(obj_dx), Float64(obj_dy), Float64(box_dx), Float64(box_dy), Float64(self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x), Float64(self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y), Float64(rob_dx), Float64(rob_dy), Float64(self.rgrip_ori), Float64(self.rob_ori.data)

	def handle_gs(self, srv):
		if (srv.request.data == True):
			return self.obj_sense, self.box_sense, self.lgrip_sense, self.rgrip_sense 	

	def handle_gs_rob_data (self, srv):
		(box_dx, box_dy) = self.global_policies.polar_to_cartesian(self.box_sense.angle.data, self.box_sense.dist.data)
		(rob_dx, rob_dy) = self.global_policies.polar_to_cartesian(self.rob_sense.angle.data, self.rob_sense.dist.data)
		rob_box_dist = self.global_policies.obtain_dist(box_dx-rob_dx, box_dy-rob_dy)

		return Float64(box_dx), Float64(box_dy), Float64(rob_dx), Float64(rob_dy), Float64(self.rob_ori.data), Float64(rob_box_dist)

	def choose_gripper_state(self, side):
		options = {
			"left":self.lgrip_sense.data,
			"right":self.rgrip_sense.data,
		}
		return options[side]

	def assign_gripper_sense(self, side, value):
		if side == "left":
			self.lgrip_sense.data = value;
		elif side == "right":
			self.rgrip_sense.data = value;
		elif side == "both":
			self.lgrip_sense.data = value;
			self.rgrip_sense.data = value;

	def grip_state_conversion (self, state):
		if state > 0.5:
			return True
		else:
			return False

	def angle_conversion (self, angle):
		return -math.degrees(angle)
		
	def publish_current_senses (self, box_rad, obj_rad):
		#rospy.loginfo("Publishing sensorization")

		self.obj_dist_pb.publish(Float64(self.obj_sense.dist.data))
		self.obj_ang_pb.publish(Float64(self.obj_sense.angle.data))
		#self.obj_size_pb.publish(Float64(self.obj_sense.radius.data))
		self.obj_size_pb.publish(Float64(obj_rad))

		self.box_dist_pb.publish(Float64(self.box_sense.dist.data))
		self.box_ang_pb.publish(Float64(self.box_sense.angle.data))
		#self.box_size_pb.publish(Float64(self.box_sense.radius.data))
		self.box_size_pb.publish(Float64(box_rad))

		self.lgrip_sense_pb.publish(Bool(self.grip_state_conversion(self.lgrip_sense.data)))
		self.rgrip_sense_pb.publish(Bool(self.grip_state_conversion(self.rgrip_sense.data)))
