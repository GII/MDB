#!/usr/bin/env python

import cv2, math, rospy, sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String
import sensor_msgs.point_cloud2 as pc2
import tf
from mdb_baxter_detection.msg import ObjDet, MObjDet, SensData
import yaml
import rospkg
import scipy.interpolate

from baxter_core_msgs.msg import HeadState

class obj_pos_grid:
	def __init__(self):
		rospy.init_node('obj_pos_grid')

		self.rospack = rospkg.RosPack()
		self.obj_type = rospy.get_param("~obj_type")
		self.camera_frame = rospy.get_param("~camera_frame")
		
		self.head_state = None
		self.complete_rva = []
		self.complete_xy = []

		self.read_grid_data_file()

		self.obj_sensor_pub = rospy.Publisher("/mdb_baxter/"+self.obj_type, SensData, queue_size = 1)

		self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
		self.track_sub = rospy.Subscriber("/tracking/"+self.obj_type, ObjDet, self.track_cb)
		self.rbf_x, self.rbf_y = self.obtain_rbf('quintic')
		
	def obtain_rbf(self, f_type):
		u = np.array(self.complete_rva)[:,0]
		v = np.array(self.complete_rva)[:,1]
		a = np.array(self.complete_rva)[:,2]

		x = np.array(self.complete_xy)[:,0]
		y = np.array(self.complete_xy)[:,1]

		rbf_x = scipy.interpolate.Rbf (u, v, a, x, function=f_type, smooth=0) 
		rbf_y = scipy.interpolate.Rbf (u, v, a, y, function=f_type, smooth=0)

		return rbf_x, rbf_y
		
	def read_grid_data_file(self):
		custom_configuration_file = self.rospack.get_path('mdb_baxter_detection')+"/config/headcamera_obj.yml"
		config = yaml.load(open(custom_configuration_file))
		for k in config.keys():
			if k == 'data':
				for pose in config[k]:
					if pose[0] != "unreachable" and pose[0] != "pose_xy": 
						for angle in range(0, len(pose[1])):
							if pose[1][angle][1] != "None":
								self.complete_rva.append([pose[1][angle][1][1], pose[1][angle][1][0], pose[1][angle][0]])
								self.complete_xy.append([pose[0][0], pose[0][1]])

	def head_state_cb(self, state):
		self.head_state = state

	def track_cb(self, coor):
		self.manage_depth(coor.v.data, coor.u.data)

	def sensorization_conversion(self, x, y):
		dist = np.sqrt((x**2)+(y**2))
		angle = math.atan2(y,x)
		return dist, angle

	def manage_depth (self, u, v):
		msg = SensData()

		(tag_x, tag_y) = scipy.interpolate.griddata(self.complete_rva, self.complete_xy, (u, v, self.head_state.pan), method = 'linear', rescale = True)
		#(tag_xn, tag_yn) = scipy.interpolate.griddata(self.complete_rva, self.complete_xy, (u, v, self.head_state.pan), method = 'nearest', rescale = True)

		#(dist, _) = self.sensorization_conversion(tag_xn, tag_yn)
		#(_, angle) = self.sensorization_conversion(tag_xl, tag_yl)

		#tag_x = self.rbf_x(u, v, self.head_state.pan)
		#tag_y = self.rbf_y(u, v, self.head_state.pan)
		
		(dist, angle) = self.sensorization_conversion(tag_x, tag_y)
		
		msg.dist.data = dist
		msg.angle.data = angle
		msg.height.data = -0.04
		msg.radius.data = 0.0
	
		self.obj_sensor_pub.publish(msg)

def main(args):
	obj_pos_grid()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
