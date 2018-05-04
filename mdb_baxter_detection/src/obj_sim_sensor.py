#!/usr/bin/env python

import rospy, sys
import numpy as np
from std_msgs.msg import Bool
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
