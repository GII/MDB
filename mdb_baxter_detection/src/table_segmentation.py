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


import os, sys, rospy, cv2, math
import numpy as np
from skimage import data, io, draw, morphology, color, measure, img_as_float, img_as_ubyte, util
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge, CvBridgeError
from mdb_baxter_detection.srv import TabSeg

class table_segmentation:
	def __init__ (self):
		rospy.init_node("table_segmentation")
		self.bridge = CvBridge()
		self.mask_table = None

		#<ROS Communication>
		self.tabseg_srv = rospy.Service("/table_seg", TabSeg, self.handle_tabseg)

	def handle_tabseg(self, req):
		im = self.image_ros2cv(req.image)
		table_v = self.get_bg(im) 
		return self.segment_table(im, table_v)  

	def image_ros2cv(self, image):
		try:
			imagecv = self.bridge.imgmsg_to_cv2(image, "passthrough")
		except CvBridgeError, e:
			print e

		im_cv = imagecv[:, :, ::-1]
		img_float = img_as_float(im_cv)
		return img_float

	def get_bg(self, im, margin=1.0/3, N=10):

		f_i=int(round(im.shape[0]*margin))
		f_f=int(round((im.shape[0]*1.0)-1))
		c_i=int(round(im.shape[1]*0.0))
		c_f=int(round((im.shape[1]*1.0)-1))

		#Malla NxN
		N = 10 
		f = np.linspace(f_i,f_f,N, dtype=int)
		c = np.linspace(c_i,c_f,N, dtype=int)

		r = np.array([], dtype='int')
		g = np.array([], dtype='int')
		b = np.array([], dtype='int')

		for ff, cc in zip(f,c):    
			r = np.hstack([r, im[f_i:f_f,cc,0], np.transpose(im[ff,c_i:c_f,0])])
			g = np.hstack([g, im[f_i:f_f,cc,0], np.transpose(im[ff,c_i:c_f,1])])
			b = np.hstack([b, im[f_i:f_f,cc,0], np.transpose(im[ff,c_i:c_f,2])])   

		v = np.zeros(3, dtype='float')
		v[0] = np.median(r)
		v[1] = np.median(g)    
		v[2] = np.median(b)

		return v

	def im_angledist(self, im, v):
		#Distancia de valores rgb entre la imagen im y el vector (rgb) v
		norm_im = np.linalg.norm(im, axis=2)
		norm_v = np.linalg.norm(v)
		sprod =  np.tensordot(im, v.reshape(1, 1, 3), axes=([2],[2]))
		aux = norm_im > 0
		c = np.squeeze(sprod[aux])/norm_im[aux]/norm_v # -> cosine of the angle
		im_out = np.ones(norm_im.shape, dtype='float')*np.pi/2
		im_out[aux] = np.arccos(np.clip(c, -1, 1)) # if you really want the angle
		return im_out

	def segment_table(self, im, v, threshold=0.05):
		im_angle = self.im_angledist(im, v)  #angulo entre RGB de pixel y color medio mesa
		hist_h = np.sum(im_angle<threshold,axis=1) #histograma horizontal

		space = 3
		diff_h = np.abs(np.diff(hist_h[::space]))

		#Deteccion de limites de la mesa
		arg_h_medio = np.round(np.size(diff_h)/2)
		
		h1 = np.argmax(diff_h[:arg_h_medio]) * space
		return Int32(h1)

	def track_objects(self, im):
		
		table_v = self.get_bg(im) #color medio mesa, solo habria que calcularlo cada X iteraciones
		self.mask_table = self.segment_table(im, table_v)  #Se haria una vez, al comienzo del experimento

def main(args):
	table_segmentation()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
