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
import numpy as np
import math
import os
import cv2
from tempfile import TemporaryFile
from cv_bridge import CvBridge, CvBridgeError
from mdb_common.srv import BaxMC, BaxMCRequest
from mdb_robots_policies.srv import GetRobSense
from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import Image

class robobo_data_experiment():
	def __init__(self):
		rospy.init_node("robobo_data_experiment")

		self.bridge = CvBridge()
		self.area_limit_l = [0.374722918415, 1.11416313281, 0.479046197325, -0.697901412477]

		self.exp_name='robobo_real'
		self.experiment_number = 0
		self.current_img_cv = None
		self.image_number = 0
		self.first_iteration = True
		self.do_continue = False
		self.action_nature = 'best' #'random' #best

		#Ground Truth
		self.goal_positions = []
		self.robobo_positions = []
		self.images_path = []

		#Preprocessed Data
		self.episode_starts = []
		self.rewards = []
		self.actions = []

		self.robobo_mv_clnt = rospy.ServiceProxy('/robobo_mv', BaxMC)
		self.gs_rob_data_clnt = rospy.ServiceProxy('/baxter/get_rob_data_sens', GetRobSense)

		self.color_img_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.color_cb)
		self.block_move_sub = rospy.Subscriber("/robot/unlock/move", Int16, self.block_move_cb)

	def color_cb (self, img):
		self.current_img_cv = self.image_ros2cv(img)

	def block_move_cb (self, msg):
		if msg.data == 0:
			self.do_continue = True

	def image_ros2cv(self, image):
		try:
			imagecv = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError,e:
			print e
		imagenp = imagecv
		return imagenp

	def create_folder (self, dir_name):
		if not os.path.exists(dir_name):
			os.mkdir(dir_name)
			return True
		else:
			return False

	def initialize_experiment_data(self):
		self.create_folder(self.exp_name)
		self.create_folder(self.exp_name+"/record_"+str(self.experiment_number).zfill(3))				

	def polar_to_cartesian (self, angle, dist):
		dx = dist*np.cos(angle)
		dy = dist*np.sin(angle)
		return dx, dy

	def obtain_dist(self, x, y):
		return np.sqrt((x**2)+(y**2))

	def candidate_actions (self, exp_data, limit=120, distance=0.07, threshold=0.05):
		robobo_l_angle = np.random.uniform(limit, -limit)
		future_dist = self.check_robobo_validity(exp_data, robobo_l_angle, distance, threshold)
		if future_dist != False:
			return robobo_l_angle, future_dist
		else:
			return False, False

	def check_robobo_validity(self, exp_data, robobo_angle, distance=0.07, threshold=0.05):
		new_angle = exp_data.rob_ori.data + math.radians(robobo_angle)
		(inc_x, inc_y) = self.polar_to_cartesian(new_angle, distance)
		future_rob_box_dist = self.obtain_dist(exp_data.box_dx.data-(exp_data.rob_dx.data+inc_x), exp_data.box_dy.data-(exp_data.rob_dy.data+inc_y))

		if ((exp_data.rob_dx.data+inc_x > self.area_limit_l[0]) and (exp_data.rob_dx.data+inc_x<self.area_limit_l[1]) and (exp_data.rob_dy.data+inc_y<self.area_limit_l[2]) and (exp_data.rob_dy.data+inc_y>self.area_limit_l[3]) and (future_rob_box_dist > threshold)):
			return future_rob_box_dist
		else:
			return False

	def choose_action_selector(self, arg):
		options = {
			'random':self.select_random_action,
			'best':self.select_best_action,
		}
		return options[arg]

	def select_best_action(self, exp_data, number_actions=10):
		final_action = None
		final_distance = None
		actions_checked = 0

		while actions_checked < number_actions and not rospy.is_shutdown():
			(singular_action, singular_distance) = self.candidate_actions(exp_data)
			if singular_action != False:
				if actions_checked == 0:
					final_action = singular_action
					final_distance = singular_distance
				elif actions_checked > 0 and singular_distance < final_distance:
					final_action = singular_action
					final_distance = singular_distance
				actions_checked+=1
		return final_action	

	def select_random_action(self, exp_data, number_actions=10):
		final_action = None
		action_found = False

		while not action_found and not rospy.is_shutdown():
			(singular_action, singular_distance) = self.candidate_actions(exp_data)
			if singular_action != False:
				final_action = singular_action
				action_found = True
		return final_action	

	def check_reward (self, distance, threshold=0.05):
		if distance < threshold:
		 	return True
		else:
			return False

	def manage_data (self, sensor_data, action, reward):
		self.robobo_positions.append([sensor_data.rob_dx.data, sensor_data.rob_dy.data]) 
		self.images_path.append(self.exp_name+"/record_"+str(self.experiment_number).zfill(3)+"/frame"+str(self.image_number).zfill(6))
		cv2.imwrite(self.exp_name+"/record_"+str(self.experiment_number).zfill(3)+"/frame"+str(self.image_number).zfill(6)+".jpg", self.current_img_cv)	
		self.actions.append(action)
		self.rewards.append(reward)

		if self.first_iteration:
			self.goal_positions.append([sensor_data.box_dx.data, sensor_data.box_dy.data]) 
			self.episode_starts.append(True)
			self.first_iteration = False
		else:
			self.episode_starts.append(False)

	def wait_for_move_completion (self):
		while not self.do_continue and not rospy.is_shutdown():
			pass
		self.do_continue = False

	def save_data (self):
		np.savez(self.exp_name+'/ground_truth', goal_positions=self.goal_positions, robobo_positions=self.robobo_positions, images_path=self.images_path)
		np.savez(self.exp_name+'/preprocessed_data', episode_starts=self.episode_starts, rewards=self.rewards, actions=self.actions)	

	def reinitialize(self):
		self.experiment_number+=1
		self.image_number = 0
		self.first_iteration=True
		self.create_folder(self.exp_name+"/record_"+str(self.experiment_number).zfill(3))

	def core_loop (self):
		self.initialize_experiment_data()
		rospy.set_param('/baxter_sense', True)
		while not rospy.is_shutdown():
			reward = False
			while not reward:
				sensor_data = self.gs_rob_data_clnt(Bool(True))	
				rob_move_msg = BaxMCRequest()
				rob_move_msg.dest.angle.data = math.radians(self.choose_action_selector(self.action_nature)(sensor_data))
				rob_move_msg.valid.data = True			
				self.robobo_mv_clnt(rob_move_msg)
				self.wait_for_move_completion()
				sensor_data = self.gs_rob_data_clnt(Bool(True))
				if not self.check_reward(sensor_data.rob_box_dist.data):
					self.manage_data(sensor_data, rob_move_msg.dest.angle.data, -1)
					self.image_number+=1
				else:
					self.manage_data(sensor_data, rob_move_msg.dest.angle.data, 1)
					self.save_data()
					reward = True
					raw_input('Press Enter after configuring the next experiment')
					self.reinitialize()
		rospy.delete_param('/baxter_sense')
		rospy.signal_shutdown('experiment_complete')
					
			

def main():
	rde = robobo_data_experiment()
	rde.core_loop()
	rospy.spin()

if __name__ == '__main__':
	main()	

