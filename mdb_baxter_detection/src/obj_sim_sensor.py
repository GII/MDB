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


import sys
import rospy
import numpy as np
from mdb_common.msg import SensData
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates

class obj_sim_sensor:
	def __init__(self):
		rospy.init_node("obj_sim_sensor")

		self.model_name = rospy.get_param("~model_name")
		self.radius = rospy.get_param("~radius")
		self.obj_type = rospy.get_param("~obj_type")
		self.rate = rospy.Rate(rospy.get_param("~rate"))

		self.model_states = None

		self.left_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.left_state_cb)
		self.obj_sensor_pub = rospy.Publisher("/mdb_baxter/"+self.obj_type, SensData, queue_size = 1)

		self.get_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

		self.publish_data()

	
	def left_state_cb(self, state):
		self.model_states = state

	def choose_xd(self, arg):
		options = {
			'exp_box':0.075,
			'exp_small_obj':0.03,
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def choose_yd(self, arg):
		options = {
			'exp_box':0.115,
			'exp_small_obj':0.03,
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def sensorization_conversion(self, trans):
		dist = np.sqrt((trans[0]**2) + (trans[1]**2))
		angle = np.arctan(trans[1]/trans[0])
		return dist, angle, trans[2]
		
	def publish_data(self):
		print 'publish_data'
		msg = SensData()
		while not rospy.is_shutdown():
			if self.model_states != None and self.model_name in self.model_states.name:
				try:
					state = self.get_state_srv(self.model_name, 'base')				
					(dist, angle, height) = self.sensorization_conversion([state.pose.position.x+self.choose_xd(self.model_name), state.pose.position.y+self.choose_yd(self.model_name), state.pose.position.z])
					msg.dist.data = dist
					msg.angle.data = angle
					msg.height.data = height
					msg.radius.data = self.radius
					self.obj_sensor_pub.publish(msg)
					self.rate.sleep()
				except rospy.ServiceException, e:
					rospy.loginfo("Delete Model service call failed: {0}".format(e))
					continue

def main(args):
	obj_sim_sensor()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
