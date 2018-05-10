#!/usr/bin/env python

import cv2, math, rospy, sys, tf, scipy.interpolate, rospkg, yaml
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String
from mdb_common.msg import ObjDet, MObjDet
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
	
		#print self.complete_uv, '\n\n\n', self.complete_xy, '\n\n\n'

		self.fu, self.fv = self.obtain_interpolation_rbf("linear")

	def read_grid_data_file(self):
		custom_configuration_file = self.rospack.get_path('mdb_baxter_detection')+"/config/ceiling_data_sort.yml"
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'data':
				for pose in config[k]:
					if pose[0] != "unreachable" and pose[0] != "pose_xy" and pose[1][0][0] != "None":
						self.complete_uv.append(pose[1][0][0]) ### reversed order in comparison with screen camera position configuration file!!
						self.complete_xy.append(pose[0])

	def obtain_interpolation_rbf(self, f_type):
		u = np.array(self.complete_uv)[:,0]
		v = np.array(self.complete_uv)[:,1]

		x = np.array(self.complete_xy)[:,0]
		y = np.array(self.complete_xy)[:,1]

		rbf_x = scipy.interpolate.Rbf (u, v, x, function=f_type, smooth=0) 
		rbf_y = scipy.interpolate.Rbf (u, v, y, function=f_type, smooth=0)

		return rbf_x, rbf_y

	def track_cb(self, coor):
		self.manage_depth(coor.v.data, coor.u.data)

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
