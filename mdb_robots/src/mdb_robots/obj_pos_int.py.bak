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
import scipy.interpolate
import rospkg 
import yaml
import numpy as np
from mdb_common.msg import ObjDet
from mdb_common.msg import SensData

class obj_pos_int:
	def __init__(self):
		rospy.init_node('obj_pos_int')

		self.rospack = rospkg.RosPack()
		self.obj_type = rospy.get_param("~obj_type")
		self.camera_frame = rospy.get_param("~camera_frame")

		self.obj_sensor_pub = rospy.Publisher("/mdb_baxter/"+self.obj_type, SensData, queue_size = 1)
		self.track_sub = rospy.Subscriber("/tracking/"+self.obj_type, ObjDet, self.track_cb)

		self.complete_uv = []
		self.complete_xy = []		
		self.read_grid_data_file()

		self.fu, self.fv = self.obtain_interpolation_rbf(rospy.get_param("~function_type"))

	def read_grid_data_file(self):
		custom_configuration_file = self.rospack.get_path('mdb_robots_detection')+"/config/"+rospy.get_param("~overhead_camera_file") 
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'data':
				for pose in config[k]:
					if pose[1] != 'non_reachable' and pose[1][0][0] != "None":
						self.complete_uv.append(pose[1][0][0]) 
						self.complete_xy.append(pose[0])

	def obtain_interpolation_rbf(self, f_type):
		u = np.array(self.complete_uv)[:,0]
		v = np.array(self.complete_uv)[:,1]

		x = np.array(self.complete_xy)[:,0]
		y = np.array(self.complete_xy)[:,1]

		rbf_x = scipy.interpolate.Rbf (u, v, x, function=f_type, smooth=0)
		rbf_y = scipy.interpolate.Rbf (u, v, y, function=f_type, smooth=0)

		return rbf_x, rbf_y

	def obtain_interpolation_int2d(self, f_type):
		u = np.array(self.complete_uv)[:,0]
		v = np.array(self.complete_uv)[:,1]

		x = np.array(self.complete_xy)[:,0]
		y = np.array(self.complete_xy)[:,1]

		fx = scipy.interpolate.interp2d(u, v, x, kind=f_type)
		fy = scipy.interpolate.interp2d(u, v, y, kind=f_type)
		return fx, fy

	def track_cb(self, coor):
		self.manage_depth(coor.u.data, coor.v.data)

	def sensorization_conversion(self, x, y, z):
		dist = np.sqrt((x**2)+(y**2))
		angle = np.arctan(y/x)
		return dist, angle, z

	def manage_depth (self, u, v):
		msg = SensData()

		x_p = float(self.fu(u, v))
		y_p = float(self.fv(u, v))
		z_p = -0.04

		(dist, angle, height) = self.sensorization_conversion(x_p, y_p, z_p)
		msg.dist.data = dist
		msg.angle.data = angle
		msg.height.data = height
		msg.radius.data = 0.0
	
		self.obj_sensor_pub.publish(msg)

def main(args):
	obj_pos_int()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
