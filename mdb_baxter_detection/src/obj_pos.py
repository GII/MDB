#!/usr/bin/env python

import cv2, math, rospy, sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String
import sensor_msgs.point_cloud2 as pc2
import tf
from mdb_baxter_detection.msg import ObjDet, MObjDet	

class obj_pos:
	def __init__(self):
		rospy.init_node('obj_pos')
		self.cloud = None
		
		self.br = tf.TransformBroadcaster()
		self.tflistener = tf.TransformListener()

		self.decimation = rospy.get_param("~decimation")		
		self.obj_type = rospy.get_param("~obj_type")
		self.camera_frame = rospy.get_param("~camera_frame")

		self.cloud_sub = rospy.Subscriber("/camera/pointcloud", PointCloud2, self.cloud_cb)		
		self.track_sub = rospy.Subscriber("/tracking/"+self.obj_type, ObjDet, self.track_cb)
		self.mtrack_sub = rospy.Subscriber("/multiple_tracking/"+self.obj_type, MObjDet, self.mtrack_cb)

	def cloud_cb(self, depth_cloud):
		self.cloud = depth_cloud

	def track_cb(self, coor):
		self.manage_depth(coor.u.data, coor.v.data, "")
		#self.manage_depth(coor.u.data+coor.radius.data, coor.v.data,"_edge")

	def mtrack_cb(self, mcoor):
		for ite in range(0, len(mcoor.objects_detection)):
			self.manage_depth(mcoor.objects_detection[ite].u.data, mcoor.objects_detection[ite].v.data, "_"+str(ite))

	def manage_depth (self, u, v, extend):
		depth_map = self.cloud
		point = self.read_depth (int(u/self.decimation), int(v/self.decimation), depth_map)
		if point != -1:	
			for item in point:
				if not (math.isnan(item[0]) or math.isnan(item[1]) or math.isnan(item[2])):
					#print "depth_info "+extend+" : ", u, v, item[0], item[1], item[2]
					try:
						(trans,rot) = self.tflistener.lookupTransform(self.camera_frame,'/base',rospy.Time(0))
						self.br.sendTransform((item[0],item[1],item[2]), rot, rospy.Time.now(), self.obj_type+extend, self.camera_frame)
					except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
						continue
	
	def read_depth(self, width, height, data) :
		# read function
		if (data==None) or (height >= data.height) or (width >= data.width):
			return -1
		data_out = pc2.read_points(data, skip_nans=False, uvs=[[width, height]])
		return data_out

def main(args):
	obj_pos()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
