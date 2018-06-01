#!/usr/bin/env python
import rospy, math, rospkg, yaml
import numpy as np
from std_msgs.msg import Bool, Int32, Float64
from com_mytechia_robobo_ros_msgs.srv import Command
from com_mytechia_robobo_ros_msgs.msg import KeyValue
from mdb_common.srv import BaxChange, BaxMC

class robobo_policies():
	def __init__(self, global_policies):
		self.rospack = rospkg.RosPack()
		self.global_policies = global_policies

		self.angle_time_l = []
		self.angle_l = []
		self.distance_time_l = []
		self.distance_l = []
		self.area_limit_l = []

		self.readtcfile()

		self.rob_poly = self.global_policies.obtain_poly(self.angle_l, self.angle_time_l, 2)
		self.rob_dist_poly = self.global_policies.obtain_poly(self.distance_l, self.distance_time_l, 2)

		self.robobo_mv_srver = rospy.Service('/robobo_mv', BaxMC, self.handle_rob_move)

		self.robobo_pick_srver = rospy.Service('/robobo_pick', BaxChange, self.handle_rob_pick)
		self.robobo_drop_srver = rospy.Service('/robobo_drop', BaxChange, self.handle_rob_drop)
		self.robobo_mv_b_srver = rospy.Service('/robobo_move_backwards', BaxChange, self.handle_rob_move_backwards)

		try:
			self.robobo_command = rospy.ServiceProxy('/command', Command)
		except rospy.ServiceException, e:
			print "Service exception", str(e)
			exit(1)

	def readtcfile(self):
		custom_configuration_file = self.rospack.get_path('mdb_baxter_policies')+"/config/"+rospy.get_param("~robobo_param_file")
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'angle_time':
				for angle_time in config[k]:
					self.angle_time_l.append(angle_time)
			if k == 'angle':
				for angle in config[k]:
					self.angle_l.append(angle)
			if k == 'distance':
				for distance in config[k]:
					self.distance_l.append(distance)
			if k == 'distance_time':
				for distance_time in config[k]:
					self.distance_time_l.append(distance_time)
			if k == 'area_limit':
				for area_limit in config[k]:
					self.area_limit_l.append(area_limit)

	def candidate_actions (self, srv, distance=0.07, threshold=0.30):
		robobo_l_angle = np.random.uniform(srv.limit.data, -srv.limit.data)
		if self.check_robobo_validity(robobo_l_angle, distance, threshold):
			return Int32(robobo_l_angle), Bool(True) 
		else:
			return Int32(robobo_l_angle), Bool(False)

	def check_robobo_validity(self, robobo_angle, distance=0.07, threshold=0.30):
		new_angle = self.global_policies.exp_senses.rob_ori.data + math.radians(robobo_angle)
		rob_dx, rob_dy = self.global_policies.translate_pos(self.global_policies.exp_senses.rob_sense.angle.data, self.global_policies.exp_senses.rob_sense.dist.data)

		inc_x = distance*np.cos(new_angle)
		inc_y = distance*np.sin(new_angle)

		boxdx, boxdy = self.global_policies.translate_pos(self.global_policies.exp_senses.box_sense.angle.data, self.global_policies.exp_senses.box_sense.dist.data)
		rob_box_dist = self.global_policies.obtain_dist(boxdx-(rob_dx+inc_x), boxdy-(rob_dy+inc_y))

		if ((rob_dx+inc_x > self.area_limit_l[0]) and (rob_dx+inc_x<self.area_limit_l[1]) and (rob_dy+inc_y<self.area_limit_l[2]) and (rob_dy+inc_y>self.area_limit_l[3]) and (rob_box_dist > threshold)):
			return True
		else:
			return False

	def rotate_robobo(self, time, lspeed):
		self.robobo_command.wait_for_service()
		command_name = 'MOVE'
		command_parameters = []
		command_parameters.append(KeyValue('lspeed', str(lspeed)))
		command_parameters.append(KeyValue('rspeed', str(-lspeed)))
		command_parameters.append(KeyValue('time', str(time)))
		self.robobo_command(command_name, 0, command_parameters)
		rospy.sleep(time)

	def	rob_move_straight(self, time, way):
		self.robobo_command.wait_for_service()
		command_name = 'MOVE'
		command_parameters = []
		command_parameters.append(KeyValue('lspeed', str(25*way)))
		command_parameters.append(KeyValue('rspeed', str(25*way)))
		command_parameters.append(KeyValue('time', str(time)))
		self.robobo_command(command_name, 0, command_parameters)
		rospy.sleep(time)

	def spin_robobo(self, angle):
		self.robobo_command.wait_for_service()
		command_name = 'TURNINPLACE'
		command_parameters = []
		command_parameters.append(KeyValue('degrees', str(math.degress)))
		self.robobo_command(command_name, 0, command_parameters)
		rospy.sleep(time)

	def handle_rob_move_backwards (self, srv):
		self.rob_move_straight(1.0, -1)
		return Bool(True)

	def handle_rob_move (self, srv): 
		robdx = self.global_policies.exp_senses.rob_sense.dist.data*np.cos(self.global_policies.exp_senses.rob_sense.angle.data)
		robdy = self.global_policies.exp_senses.rob_sense.dist.data*np.sin(self.global_policies.exp_senses.rob_sense.angle.data)

		if srv.valid.data == True:
			time = self.rob_poly(abs(srv.dest.angle.data))
			lspeed = 10
			if srv.dest.angle.data >= 0.0:
				lspeed = -lspeed

			self.rotate_robobo(time, lspeed)
			self.rob_move_straight(0.5, 1)

			if not self.robobo_status:

				predicted_angle = self.global_policies.angle_fix(self.global_policies.exp_senses.rob_ori.data+srv.dest.angle.data)
				fx = robdx + 0.05*math.cos(predicted_angle)
				fy = robdy + 0.05*math.sin(predicted_angle)

				self.global_policies.exp_senses.rob_sense.dist.data = np.sqrt((fy**2)+(fx**2))
				self.global_policies.exp_senses.rob_sense.angle.data = np.arctan(fy/fx)
				self.global_policies.exp_senses.rob_ori.data = predicted_angle

		return Bool(True)

	def handle_rob_pick (self, srv):
		dx, dy = self.global_policies.cartesian_to_push(self.global_policies.exp_senses.obj_sense.angle.data, self.global_policies.exp_senses.obj_sense.dist.data, self.global_policies.exp_senses.rob_sense.angle.data, self.global_policies.exp_senses.rob_sense.dist.data)
		dist = self.global_policies.obtain_dist(dx, dy) ###Distance between the object and the robot

		print "robobo should rotate ", self.global_policies.exp_senses.rob_obj_ori.data - self.global_policies.exp_senses.rob_ori.data, " since its current angle is ", self.global_policies.exp_senses.rob_ori.data, " and the object is at ", self.global_policies.exp_senses.rob_obj_ori.data
		time = self.rob_poly(abs(self.global_policies.exp_senses.rob_obj_ori.data - self.global_policies.exp_senses.rob_ori.data))

		lspeed = 10
		if (self.global_policies.exp_senses.rob_obj_ori.data - self.global_policies.exp_senses.rob_ori.data) >= 0.0:
			lspeed = -lspeed

		self.rotate_robobo(time+0.2, lspeed)
		rospy.sleep(2)
		dist_time = self.rob_dist_poly(dist-0.09)
		self.rob_move_straight(dist_time, 1)
		self.global_policies.exp_senses.rob_grip = 1.0
		rospy.sleep(2)
		return Bool(True)

	def handle_rob_drop (self, srv):
		dx, dy = self.global_policies.cartesian_to_push(self.global_policies.exp_senses.box_sense.angle.data, self.global_policies.exp_senses.box_sense.dist.data, self.global_policies.exp_senses.rob_sense.angle.data, self.global_policies.exp_senses.rob_sense.dist.data)
		dist = self.global_policies.obtain_dist(dx, dy) ###Distance between the object and the robot

		print "robobo should rotate ", self.global_policies.exp_senses.rob_box_ori.data - self.global_policies.exp_senses.rob_ori.data, " since its current angle is ", self.global_policies.exp_senses.rob_ori.data, " and the object is at ", self.global_policies.exp_senses.rob_box_ori.data
		time = self.rob_poly(abs(self.global_policies.exp_senses.rob_box_ori.data - self.global_policies.exp_senses.rob_ori.data))

		lspeed = 10
		if (self.global_policies.exp_senses.rob_box_ori.data - self.global_policies.exp_senses.rob_ori.data) >= 0.0:
			lspeed = -lspeed

		self.rotate_robobo(time+0.1, lspeed)
		rospy.sleep(2)
		dist_time = self.rob_dist_poly(dist-0.09)
		self.rob_move_straight(dist_time, 1)
		self.rob_move_straight(1.0, -1)
		self.global_policies.exp_senses.rob_grip = 0.0
		rospy.sleep(2)
		return Bool(True)
