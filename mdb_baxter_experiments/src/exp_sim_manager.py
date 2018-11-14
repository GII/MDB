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
import rospkg
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point
from mdb_baxter_experiments.srv import SimMng
from std_msgs.msg import Bool

class exp_sim_manager():
	def __init__(self):
		rospy.init_node('exp_simulation_manager')

		self.load_srv = rospy.Service("/sim/create_obj", SimMng, self.handle_load)
		self.mofify_srv = rospy.Service("/sim/modify_obj", SimMng, self.handle_mod)
		self.delete_srv = rospy.Service("/sim/delete_obj", SimMng, self.handle_del)

		self.spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
		self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		self.set_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		self.table_fixed_pos = Pose(position=Point(x=0.22, y=-1.221, z=0.865))

	def choose_height(self, arg):
		options = {
			'exp_box':0.019+0.865,
			'exp_small_obj':0.0+0.865,
			'exp_big_obj':0.0+0.865,
		}
		return options[arg]

	def choose_xd(self, arg):
		options = {
			'exp_box':0.075,
			'exp_small_obj':0.025, #0.03,
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def choose_yd(self, arg):
		options = {
			'exp_box':0.115,
			'exp_small_obj':0.025, #0.03,
			'exp_big_obj':0.0675,
		}
		return options[arg]

	def translate_pos (self, angle, dist, xd, yd):
		dx = dist*np.cos(angle)
		dy = dist*np.sin(angle)
		return dx-xd, dy-yd

	def handle_load(self, req):
		if req.model_name.data == "exp_table":
			if self.load_model(self.table_fixed_pos, 'world', 'exp_table'):
				return Bool(True)
		else:
			dx, dy = self.translate_pos(req.sense.angle.data, req.sense.dist.data, self.choose_xd(req.model_name.data), self.choose_yd(req.model_name.data))
			if self.load_model(Pose(position=Point(x=dx, y=dy, z=self.choose_height(req.model_name.data))), 'world', req.model_name.data):
				return Bool(True)
		return Bool(False)

	def handle_mod(self, req):
		try:
			self.set_model(ModelState(model_name=req.model_name.data, pose=req.pose, reference_frame=req.reference_frame.data))
			self.delete_model(req.model_name.data)
			dx, dy = self.translate_pos(req.sense.angle.data, req.sense.dist.data, self.choose_xd(req.model_name.data), self.choose_yd(req.model_name.data))
			if self.load_model(Pose(position=Point(x=dx, y=dy, z=self.choose_height(req.model_name.data)+req.sense.height.data)), 'world', req.model_name.data):						
				return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Set Model State service call failed: {0}".format(e))
			return Bool(False)

	def handle_del(self, req):
		try:
			self.delete_model(req.model_name.data)
			return Bool(True)
		except rospy.ServiceException, e:
			rospy.loginfo("Delete Model service call failed: {0}".format(e))
			return Bool(False)

	def load_urdf (self, model_path, name):
		object_xml = ''
		with open (model_path + name+"/"+name+".urdf", "r") as object_file:
			object_xml=object_file.read().replace('\n', '')
		return object_xml

	def load_model (self, pose, ref, name):
		model_path = rospkg.RosPack().get_path('mdb_baxter_experiments')+"/models/"
		xml = self.load_urdf(model_path, name)

		# Spawn Block URDF
		rospy.wait_for_service('/gazebo/spawn_urdf_model')
		try:
			self.spawn_urdf(name, xml, "/", pose, ref)
			return True
		except rospy.ServiceException, e:
			rospy.logerr("Spawn URDF service call failed: {0}".format(e))
			return False

def exp_sim_manager_server():
	exp_sim_manager()
	rospy.spin()

if __name__ == "__main__":
    exp_sim_manager_server()
