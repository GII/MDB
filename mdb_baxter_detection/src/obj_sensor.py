#!/usr/bin/env python

import cv2, math, rospy, sys
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Bool, String
import sensor_msgs.point_cloud2 as pc2
import tf
from mdb_baxter_detection.msg import SensData, BSensData
from geometry_msgs.msg import Point

class obj_sensor:
	def __init__(self):
		rospy.init_node("obj_sensor")

		self.br = tf.TransformBroadcaster()
		self.tflistener = tf.TransformListener()

		self.obj_type = rospy.get_param("~obj_type")
		self.ref_frame = rospy.get_param("~ref_frame")
		self.rate = rospy.Rate(rospy.get_param("~rate"))
		self.stype = rospy.get_param("~stype")
		self.use_beacons = rospy.get_param("~use_beacons")

		self.obj_sensor_pub = rospy.Publisher("/mdb_baxter/"+self.obj_type, SensData, queue_size = 1)
		self.mobj_sensor_pub = rospy.Publisher("/mdb_baxter/multi/"+self.obj_type, BSensData, queue_size = 1)
			
		self.real_beacon_data = [[0.83, 1.2065],[1.4255, 1.2065],[1.4255, 0.6105],[1.4255, 0.0],[1.4255, -0.6105],[1.4255, -1.2065],[0.83, -1.2065]]
		self.detected_beacon_data = None
		self.mobj_sensor_sb = rospy.Subscriber("/mdb_baxter/multi/beacon", BSensData, self.mobj_sensor_cb)

		self.choose_sentype()()

	def mobj_sensor_cb(self, bsd):
		self.detected_beacon_data = bsd

	def obtain_beacon_offset(self):
		dbd = self.detected_beacon_data.beacon_sense
		dx = 0.0
		dy = 0.0
		it = 0.0
		for beacon in dbd:
			print "beacon.z: ", beacon.z
			if beacon.z > -0.1:
				ind = min(range(len(self.real_beacon_data)), key = lambda i: abs((abs(self.real_beacon_data[i][0]-beacon.x) + abs(self.real_beacon_data[i][1]-beacon.y))))
				dx += self.real_beacon_data[ind][0]-beacon.x
				dy += self.real_beacon_data[ind][1]-beacon.y
				it+=1
		print "beacon number: ", it
		if it != 0.0:
			dx = dx/(it)
			dy = dy/(it)
		return dx, dy

	def rectify_sensorization(self, tf):
		dx, dy = self.obtain_beacon_offset()
		#print "offset: ", dx, dy
		return [tf[0]+dx, tf[1]+dy, tf[2]]	

	def choose_sentype (self):
		options = {
			"single":self.publish_data,
			"multi":self.publish_multidata,
		}
		return options[self.stype]

	def read_transform(self, ref, dest):
		try:
			(trans,rot) = self.tflistener.lookupTransform(ref, dest, rospy.Time(0))
			return trans
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			return False

	def selectSign(self, arg):
		if arg < 0:
			return -1.0
		else:
			return 1.0

	def sensorization_conversion(self, trans):
		print trans[0], trans[1], trans[2]
		dist = np.sqrt((trans[0]**2) + (trans[1]**2))
		angle = np.arctan(trans[1]/trans[0])
		print dist, angle
		'''if -trans[0]<0:
			angle = 3.1416*self.selectSign(trans[1]) + sangle'''
		return dist, angle, trans[2]
		
	def publish_data(self):
		print 'publish_data'
		msg = SensData()
		while not rospy.is_shutdown():
			tf = self.read_transform(self.ref_frame,"/"+self.obj_type)
			#tf_e = self.read_transform("/"+self.obj_type+"_edge","/"+self.obj_type)
			if tf: # and tf_e:
				'''if self.use_beacons:
					tf_fix = self.rectify_sensorization(tf)
					#tf_e_fix = self.rectify_sensorization(tf_e)
				else:
					tf_fix = tf
					#tf_e_fix = tf_e'''
				(dist, angle, height) = self.sensorization_conversion(tf)
				msg.dist.data = dist
				msg.angle.data = angle
				msg.height.data = height
				#msg.radius.data = tf_e_fix[1]
				self.obj_sensor_pub.publish(msg)
				self.rate.sleep()

	def publish_multidata(self):
		print 'publish_multidata'
		while not rospy.is_shutdown():
			b_info_l = BSensData()
			check = True
			b_number = 0
			while b_number < 5 and check:
				tf = self.read_transform(self.ref_frame,"/"+self.obj_type+"_"+str(b_number))
				if tf:
					b_info = Point()
					b_info.x = tf[0]
					b_info.y = tf[1]
					b_info.z = tf[2]					
					b_info_l.beacon_sense.append(b_info)					
					b_number+=1
				else:
					check = False
			self.mobj_sensor_pub.publish(b_info_l)
			self.rate.sleep()

def main(args):
	obj_sensor()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
