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
from mdb_baxter_policies.srv import GetHandImg
from sensor_msgs.msg import Image

class get_hand_image():
	def __init__(self):
		rospy.init_node('get_hand_image_server')
		
		self.current_limg = None
		self.current_rimg = None

		self.limg_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.limg_cb)
		self.rimg_sub = rospy.Subscriber('/cameras/right_hand_camera/image', Image, self.rimg_cb)
		self.getimage_srv = rospy.Service('/get_hand_image', GetHandImg, self.handle_get_image)

	def limg_cb(self, Image):
		self.current_limg = Image

	def rimg_cb(self, Image):
		self.current_rimg = Image

	def handle_get_image(self, req):
		if (req.hand.data == "left"):
			print "Sending left hand camera image"
			return self.current_limg
		elif (req.hand.data == "right"):
			print "Sending right hand camera image"
			return self.current_rimg
		else:
			return None

def get_hand_image_server():
	get_hand_image()

	init = rospy.get_rostime()
	rospy.sleep(65.8)
	nseconds = (rospy.get_rostime() - init).to_nsec()
	miliseconds = (nseconds / 10**6)
	seconds = miliseconds / 1000
	minutes = int(seconds/60)
	resto_sec = int(seconds) - (minutes*60)
	resto_msec = miliseconds - resto_sec*1000 - minutes*60*1000
	

	print minutes, resto_sec, resto_msec

	rospy.spin()

if __name__ == "__main__":
    get_hand_image_server()
