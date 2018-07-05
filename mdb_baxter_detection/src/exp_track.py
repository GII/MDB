#!/usr/bin/env python

import os, sys, rospy, cv2, math
import numpy as np
from skimage import data, io, draw, morphology, color, measure, img_as_float, img_as_ubyte, util
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from mdb_common.msg import ObjDet
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import colorsys

import skimage.morphology

from geometry_msgs.msg import PointStamped


class exp_track:
	def __init__ (self):
		rospy.init_node("exp_track")
		
		#<Variables>
		self.bridge = CvBridge()

		self.first_iteration = True
		self.iteration_number = 0
		self.bg_rec = 20

		self.mask_table = None
		self.do_sense = False

		self.exp_rec = rospy.get_param("~exp_rec")
		self.head_camera = rospy.get_param("~head_camera")

		self.objects = []
		self.initialize_objects()

		self.obj_pix = None
		self.box_pix = None

		self.tag_detect = True

		#<ROS Communication>
		self.color_sub = rospy.Subscriber("/image_color", Image, self.color_cb)
		self.rob_loc_sb = rospy.Subscriber("/aruco_single_head/tag_detection", Bool, self.rob_loc_cb)

		self.track_pub = rospy.Publisher("/tracking/"+"img", Image, queue_size = 1)
		self.obj_coor_pub = rospy.Publisher("/tracking/"+"ball", ObjDet, queue_size = 1)
		self.box_coor_pub = rospy.Publisher("/tracking/"+"box", ObjDet, queue_size = 1)
		self.rob_coor_pub = rospy.Publisher("/tracking/"+"robobo", ObjDet, queue_size = 1)

		self.obj_det_pub = rospy.Publisher("/tracking/"+"ball"+"_flag", Bool, queue_size = 1);

		self.rob_ori_pub = rospy.Publisher("tracking/robobo_ori", Float64, queue_size = 1)
		self.rob_ori_obj_pub = rospy.Publisher("tracking/robobo_ori_obj", Float64, queue_size = 1)
		self.rob_ori_box_pub = rospy.Publisher("tracking/robobo_ori_box", Float64, queue_size = 1)

		###
		self.c0_val = None
		self.c1_val = None
		self.c2_val = None
		self.cc_val = None

		self.c0_sub = rospy.Subscriber("/aruco_single_head/corner_0_pixel", PointStamped, self.c0_cb)
		self.c1_sub = rospy.Subscriber("/aruco_single_head/corner_1_pixel", PointStamped, self.c1_cb)
		self.c2_sub = rospy.Subscriber("/aruco_single_head/corner_2_pixel", PointStamped, self.c2_cb)
		self.cc_sub = rospy.Subscriber("/aruco_single_head/pixel", PointStamped, self.cc_cb)
		###
		

	def rob_loc_cb(self, msg):
		self.tag_detect = msg.data

	def c0_cb(self, msg):
		self.c0_val = [int(msg.point.x), int(msg.point.y)]

	def c1_cb(self, msg):
		self.c1_val = [int(msg.point.x), int(msg.point.y)]

	def c2_cb(self, msg):
		self.c2_val = [int(msg.point.x), int(msg.point.y)]

	def cc_cb(self, msg):
		self.cc_val = [int(msg.point.x), int(msg.point.y)]

	def image_ros2cv(self, image):
		try:
			imagecv = self.bridge.imgmsg_to_cv2(image, "passthrough")
		except CvBridgeError,e:
			print e
		imagenp = imagecv
		return imagenp

	def image_cv2ros(self, image):
		try: 
			imageros = self.bridge.cv2_to_imgmsg(image, "rgb8")
		except CvBridgeError, e:
			print e
		return imageros

	def initialize_objects(self):
		self.objects.append(['cilindro', np.array(rospy.get_param("~c_rgb"), dtype='float')/255])
		if self.exp_rec == "mot":
			self.objects.append(['cesta', np.array([23, 62, 99], dtype='float')/255])		
		else:
			self.objects.append(['cesta',   np.array(rospy.get_param("~b_rgb"),  dtype='float')/255])
		###
		#self.objects.append(['robobo',   np.array([19, 67, 125],  dtype='float')/255])
		#self.objects.append(['robobo',   np.array([34, 86, 130],  dtype='float')/255])
		#self.objects.append(['robobo',   np.array([111, 134, 140],  dtype='float')/255]


	def select_publisher(self, arg):
		options = {
			'cilindro':self.obj_coor_pub,
			'cesta':self.box_coor_pub, 
			'robobo':self.rob_coor_pub, 	
		}
		return options[arg]

	def insert_rectangle(self, im, bbox, perimeter=True, color=[1.0, 1.0, 1.0]):
		if perimeter==True:
			rr, cc = draw.polygon_perimeter([bbox[0], bbox[0], bbox[2], bbox[2]], [bbox[1], bbox[3], bbox[3], bbox[1]], shape=im.shape, clip=True)
		else:
			rr, cc = draw.polygon(np.array([bbox[0], bbox[0], bbox[2], bbox[2]]), np.array([bbox[1], bbox[3], bbox[3], bbox[1]]), shape=im.shape)
		im[rr, cc, :] = color
		return im

	def insert_line(self, im, r0, c0, c, color=[1.0, 1.0, 1.0]):
		# skimage.draw.line(r0, c0, r1, c1)
		#rr, cc = draw.line(int(r0), int(c0), int(r0 + evec[0]*scale/2), int(c0 + evec[1]*scale/2))
		r1 = np.max([ int(r0 + c[0]), 0])
		c1 = np.max([ int(c0 + c[1]), 0])
		r1 = np.min([ r1, im.shape[0]])
		c1 = np.min([ c1, im.shape[1]])
		rr, cc = draw.line(int(r0), int(c0), r1, c1)
		im[rr, cc, :] = color
		return im

	def get_bg(self, im, margin=1.0/3, N=10):
		#Limites malla
		margin = margin

		f_i=int(round((im.shape[0]/2)*margin))
		f_f=int(round((im.shape[0]/2)*(1.0-margin)))
		c_i=int(round((im.shape[1])*(margin)))
		c_f=int(round((im.shape[1])*(1.0-(margin))))

		print im.shape[0], im.shape[1], margin
		print f_i, f_f, c_i, c_f

		#cv2.imshow("wipe",  im[f_i:f_f,c_i:c_f])
		#cv2.waitKey(0)

		'''f_i=int(round(im.shape[0]*margin))
		f_f=int(round((im.shape[0]*1.0)-1))
		c_i=int(round(im.shape[1]*0.0))
		c_f=int(round((im.shape[1]*1.0)-1))'''

		#Malla NxN
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

	def region_angledist(self, rgb_region, rgb_obj):
		norm_region = np.linalg.norm(rgb_region, axis=1)
		norm_obj = np.linalg.norm(rgb_obj)
		sprod =  np.dot(rgb_region, rgb_obj)
		aux = norm_region > 0
		c = sprod[aux]/norm_region[aux]/norm_obj # -> cosine of the angle
		region_dist = np.ones(norm_region.shape, dtype='float')*np.pi/2
		region_dist[aux] = np.arccos(np.clip(c, -1, 1)) # if you really want the angle
		return region_dist

	def segment_table(self, im, v, threshold=0.15):
		threshold = rospy.get_param("~table_segmentation_threshold")
		im_angle = self.im_angledist(im, v)  #angulo entre RGB de pixel y color medio mesa
		
		hist_v = np.sum(im_angle<threshold,axis=0) #histograma vertical
		hist_h = np.sum(im_angle<threshold,axis=1) #histograma horizontal

		space = 3
		diff_v = np.abs(np.diff(hist_v[::space]))
		diff_h = np.abs(np.diff(hist_h[::space]))

		#Deteccion de limites de la mesa
		arg_v_medio = np.round(np.size(diff_v)/2)
		arg_h_medio = np.round(np.size(diff_h)/2)
		
		v1 = np.argmax(diff_v[:arg_v_medio]) * space
		v2 = (np.argmax(diff_v[arg_v_medio:]) + arg_v_medio) * space
		
		h1 = np.argmax(diff_h[:arg_h_medio]) * space
		h2 = (np.argmax(diff_h[arg_h_medio:]) + arg_h_medio) * space

		'''if self.head_camera:
			v1 = 0
			v2 = im.shape[1]
			h2 = im.shape[0]'''

		v1 = 0
		v2 = im.shape[1]
		h1 = 98
		h2 = 430

		rr, cc = draw.polygon(np.array([h1, h1, h2, h2]), np.array([v1, v2, v2, v1]), shape=im.shape)
		mask = np.zeros([im.shape[0],im.shape[1]], dtype='bool')
		mask[rr, cc] = True
		S = 10
		mask = morphology.binary_erosion(mask, selem=np.ones([S,S]))
		
		return mask
    
	def segment_objects(self, im, mask_table, v, threshold=0.1,  min_px_size=100):  
		#Segmentar por canal h
		v_hsv = color.rgb2hsv(v.reshape([1,1,3]))
		im_hsv = color.rgb2hsv(im)
		im_huedist = np.abs(im_hsv[:,:,0]-v_hsv[0,0,0])
		im_huedist = im_huedist  * mask_table
		im_seg_hue = im_huedist  > threshold #Objetos segmentados

		im_label_hue = morphology.label(im_seg_hue)
		im_label_hue =morphology.remove_small_objects(im_label_hue, min_px_size)
		regions = measure.regionprops(im_label_hue) #Busqueda de Bounding boxes 
		
		return regions, im_label_hue
    
	def detect_object(self, im, regions, obj, threshold=0.3, min_px_size=30, percentage = 0.5): #threshold=0.15, min_px_size=30):
		rgb_obj = obj[1]
		idx_all = []
		suma = rgb_obj
		for reg in regions:
			i = reg.coords[:,0]
			j = reg.coords[:,1]

			total_pixel = len(i)
			rgb_region = np.hstack([np.reshape(im[i,j,0], [np.size(i), 1]), np.reshape(im[i,j,1], [np.size(i), 1]), np.reshape(im[i,j,2], [np.size(i), 1])])
			#ii = i[self.region_angledist(rgb_region, rgb_obj) < threshold]
			#jj = j[self.region_angledist(rgb_region, rgb_obj) < threshold]
			i = i[self.region_angledist(rgb_region, rgb_obj) < threshold]
			j = j[self.region_angledist(rgb_region, rgb_obj) < threshold]
			idx = np.hstack([np.reshape(i, [np.size(i), 1]), np.reshape(j, [np.size(j), 1])])

			#if len(ii) > min_px_size:

			current_per = float(idx.shape[0])/float(total_pixel)
			#print idx.shape[0], total_pixel, current_per
			if idx.shape[0] > min_px_size and current_per > percentage:
				#print "comparacion:", idx.shape[0], min_px_size
				suma=0
				for ite in range (0, len(i)):				
					suma += im[i[ite], j[ite]]
		
				suma = suma/len(i)
				idx_all.append(idx)
		return idx_all, suma

	def detect_orientation(self, idx):
		centro_i =  np.median(idx[:,0])
		centro_j =  np.median(idx[:,1])
		area = idx.shape[0]
		#print area
		dist_idx_centro = np.sqrt(np.sum((idx - [centro_i, centro_j])**2, -1))

		k = idx[dist_idx_centro > area*0.115,:] 
		punta_i =  np.mean(k[:,0])
		punta_j =  np.mean(k[:,1])
		return [punta_i-centro_i, punta_j-centro_j], k

	def color_cb (self, img):
		img_cv = self.image_ros2cv(img)
		#img_cv = cv2.imread('/home/baxter/Descargas/photo6039835045067336906.jpg')
		#img_cv = cv2.imread('/home/baxter/Descargas/photo6023781007070703177.jpg')
		#img_cv = cv2.imread('/home/baxter/Descargas/photo5776302972840553965.jpg')
		if self.head_camera:
			im_cv = img_cv[:, :, ::-1]
			img_float = img_as_float(im_cv)
			self.track_objects(img_float, np.array(np.array(im_cv, dtype=np.uint8)))
		else:
			img_float = img_as_float(img_cv)
			self.track_objects(img_float, np.array(np.array(img_cv, dtype=np.uint8)))


	def bg_rec_check (self):
		if self.iteration_number == 0 or self.iteration_number>self.bg_rec:
			r = self.iteration_number%self.bg_rec
			if r < 0.2:
				return True
			else:
				return False

	def close_enough(self, orig_v, new_v, threshold = 0.10): #0.05	
		print np.abs(orig_v-new_v).max()
		if np.abs(orig_v-new_v).max() < threshold:
			return True
		return False

	def track_objects(self, im, im_cv):
		if rospy.has_param("/baxter_sense"):
			self.do_sense = rospy.get_param("/baxter_sense")
			if self.do_sense:
				if self.bg_rec_check():
					print "background refresh" 
					self.table_v = self.get_bg(im) #color medio mesa, solo habria que calcularlo cada X iteraciones
				if self.first_iteration:
					self.mask_table = self.segment_table(im, self.table_v)  #Se haria una vez, al comienzo del experimento
					self.first_iteration = False	
				time_before = rospy.get_rostime()
				regions, im_label_hue = self.segment_objects(im, self.mask_table, self.table_v)
				
				im_out = np.copy(im_cv)  #Para representacion   
				im_out[np.logical_not(self.mask_table)] = im_out[np.logical_not(self.mask_table)]*0.5
		
				for reg, j in zip(regions,xrange(0,regions.__len__())):
					#Bounding box (min_row, min_col, max_row, max_col)
					#im_out = self.insert_rectangle(im_out, reg.bbox, color=[255, 0, 0])
					pass

				robobo = False
				cylinder = False
				box = False
				

				for obj in self.objects:
					if self.exp_rec == 'mot':
						idx_all, new_rgb_value = self.detect_object(im, regions, obj, threshold = 0.25)
					else:
						idx_all, new_rgb_value = self.detect_object(im, regions, obj, threshold = 0.25, percentage = 0.0)
					idx_all.sort(key = lambda s: s.shape[0], reverse=True)
					#for iterator in range (0, len(idx_all)):
						#print idx_all[iterator].shape[0]

					for idx in idx_all:
						im_out[idx[:,0],idx[:,1]] = obj[1]*255
						centro_i =  np.sum(idx[:,0])/idx.shape[0]
						centro_j =  np.sum(idx[:,1])/idx.shape[0]
						l = 1
						bbox_centro = np.round([centro_i-l, centro_j-l, centro_i+l, centro_j+l])

						if obj[0]== "robobo" and not robobo:
							robobo = True						
						else:					
							if (obj[0] == "cilindro" and not cylinder) or (obj[0] == "cesta" and not box):
								im_out = self.insert_rectangle(im_out, bbox_centro, perimeter=False, color=obj[1])
								#im_out = self.insert_rectangle(im_out, bbox_centro, perimeter=True, color=[1, 1, 1])
								#print 'Objeto: ' + obj[0] + ', ' + np.str([int(centro_i), int(centro_j)])
	
								obj_det = ObjDet()
								obj_det.u.data = int(centro_j)
								obj_det.v.data = int(centro_i)
								obj_det.radius.data = 0
								self.select_publisher(obj[0]).publish(obj_det)

								print 'Objeto: ' + obj[0] + ', ' + np.str([int(obj_det.u.data), int(obj_det.v.data)])

								if obj[0] == "cilindro": 
									self.obj_pix=[obj_det.u.data, obj_det.v.data]
									cylinder=True

								if obj[0] == "cesta": 
									self.box_pix=[obj_det.u.data, obj_det.v.data]
									box=True

				###Robobo detection through tag:
				#print self.c0_val, self.c1_val, self.c2_val, self.cc_val
				if self.exp_rec == 'mot' and self.c0_val != None and self.c1_val != None and self.c2_val != None and self.cc_val != None and self.tag_detect:

					rr, cc = draw.circle(self.cc_val[1], self.cc_val[0], 3)
					im_out[rr, cc, :] = [255, 255, 0]

					c1 = (self.cc_val[1] - (self.c1_val[1]+self.c2_val[1])/2)
					c2 = (self.cc_val[0] - (self.c1_val[0]+self.c2_val[0])/2) 
					angle = np.arctan2(c2, c1)		

					rr, cc = draw.line(self.cc_val[1], self.cc_val[0], (self.c1_val[1]+self.c2_val[1])/2, (self.c1_val[0]+self.c2_val[0])/2)
					im_out[rr, cc, :] = [0, 255, 255]

					if self.obj_pix != None:
						co1 = (self.cc_val[0] - self.obj_pix[0])
						co2 = (self.cc_val[1] - self.obj_pix[1])
						angle_obj = np.arctan2(co1, co2)
						self.rob_ori_obj_pub.publish(Float64(angle_obj))

					if self.box_pix != None:
						cb1 = (self.cc_val[0] - self.box_pix[0])
						cb2 = (self.cc_val[1] - self.box_pix[1])
						angle_box = np.arctan2(cb1, cb2)
						self.rob_ori_box_pub.publish(Float64(angle_box))

					print 'Objeto: ' + "robobo" + ', ' + np.str([int(self.cc_val[1]), int(self.cc_val[0])])

					obj_det = ObjDet()
					obj_det.u.data = self.cc_val[0]
					obj_det.v.data = self.cc_val[1]

					obj_det.radius.data = 0
					self.select_publisher("robobo").publish(obj_det)
					self.rob_ori_pub.publish(Float64(angle))

				if cylinder:
					self.obj_det_pub.publish(Bool(True))
				else:
					self.obj_det_pub.publish(Bool(False))

				self.track_pub.publish(self.image_cv2ros(im_out))
				self.iteration_number+=1

				time = ((rospy.get_rostime()-time_before).to_nsec())/10**6
				print "Time: ", time

		elif self.do_sense:
				self.iteration_number+=1
				self.do_sense = False

def main(args):
	exp_track()
	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv)
	except RuntimeError, e:
		rospy.logerr('Something went wrong: %s' % (e.strerrror))
