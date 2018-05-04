#!/usr/bin/env python
import rospy, math
import numpy as np
from std_msgs.msg import Float64, Bool, String
from mdb_baxter_detection.msg import SensData
from mdb_baxter_policies.srv import GetSense, GetES, GetSenseMotiv, GetSenseFM

class exp_senses():
	def __init__(self):
		self.obj_sense = SensData()
		self.box_sense = SensData()
		self.rob_sense = SensData()
		self.rob_ori = Float64()
		self.rob_obj_ori = Float64()
		self.rob_box_ori = Float64()

		self.larm_state = None
		self.rarm_state = None

		self.rgrip_ori = 0.0
		self.rob_grip = 0.0

		self.sobj_sense_sb = rospy.Subscriber("/mdb_baxter/small_obj", SensData, self.obj_sense_cb)
		self.bobj_sense_sb = rospy.Subscriber("/mdb_baxter/big_obj", SensData, self.obj_sense_cb)
		self.bobj_sense_sb = rospy.Subscriber("/mdb_baxter/ball", SensData, self.obj_sense_cb)
		self.box_sense_sb = rospy.Subscriber("/mdb_baxter/box", SensData, self.box_sense_cb)
		self.rob_sense_sb = rospy.Subscriber("/mdb_baxter/robobo", SensData, self.rob_sense_cb)

		self.rob_ori_sb = rospy.Subscriber("/tracking/robobo_ori", Float64, self.rob_ori_cb)
		self.rob_ori_obj_sb = rospy.Subscriber("/tracking/robobo_ori_obj", Float64, self.rob_ori_obj_cb)
		self.rob_ori_box_sb = rospy.Subscriber("/tracking/robobo_ori_box", Float64, self.rob_ori_box_cb)

		self.obj_dist_pb = rospy.Publisher("/mdb3/baxter/sensor/ball_dist", Float64, queue_size = 1)
		self.obj_ang_pb = rospy.Publisher("/mdb3/baxter/sensor/ball_ang", Float64, queue_size = 1)
		self.obj_size_pb = rospy.Publisher("/mdb3/baxter/sensor/ball_size", Float64, queue_size = 1)

		self.box_dist_pb = rospy.Publisher("/mdb3/baxter/sensor/box_dist", Float64, queue_size = 1)
		self.box_ang_pb = rospy.Publisher("/mdb3/baxter/sensor/box_ang", Float64, queue_size = 1)
		self.box_size_pb = rospy.Publisher("/mdb3/baxter/sensor/box_size", Float64, queue_size = 1)

		self.lgrip_sense = Float64(0.0)
		self.rgrip_sense = Float64(0.0)
		#self.lgrip_sense_pb = rospy.Publisher("/baxter_exp/left_gripper", Float64, queue_size = 1)
		#self.rgrip_sense_pb = rospy.Publisher("/baxter_exp/left_gripper", Float64, queue_size = 1)

		self.lgrip_sense_pb = rospy.Publisher("/mdb3/baxter/sensor/ball_in_left_hand", Bool, queue_size = 1)
		self.rgrip_sense_pb = rospy.Publisher("/mdb3/baxter/sensor/ball_in_right_hand", Bool, queue_size = 1)

		self.gs_srver = rospy.Service('/baxter_sense', GetSense, self.handle_gs)
		self.gs_motiv_srver = rospy.Service('/mdb3/baxter/sensors', GetSenseMotiv, self.handle_gs_motiv)
		self.gs_fm_srver = rospy.Service('/mdb3/baxter/fm_sensors', GetSenseFM, self.handle_gs_fm)

		self.get_es = rospy.ServiceProxy('/get_end_state', GetES) 

	def endpoint_data (self):
		try:
			self.larm_state = self.get_es(String('left'))
			self.rarm_state = self.get_es(String('right'))
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc)) 

  	### Callbacks ###
	def obj_sense_cb (self, sens):
		if sens.height.data > -0.10:
			self.obj_sense = sens

	def box_sense_cb (self, sens):
		if sens.height.data > -0.10 and not rospy.has_param("/check_reward"): #and rospy.has_param("/box_sense"):
			self.box_sense = sens

	def rob_sense_cb (self, sens):
		if sens.height.data > -0.10:
			self.rob_sense = sens
			#self.rob_sense.dist.data +=0.135

	def rob_ori_cb (self, ori):
		self.rob_ori = ori
		'''ori.data += 3.14
		if ori.data > 3.14:
			self.rob_ori.data = ori.data - 6.28
		elif ori.data < -3.14:
			self.rob_ori.data = ori.data + 6.28'''

	def rob_ori_obj_cb(self, ori):
		self.rob_obj_ori = ori
			
	def rob_ori_box_cb(self, ori):
		self.rob_box_ori = ori

	def rob_loc_cb(self, loc):
		(rob_dx, rob_dy) = self.translate_pos(self.rob_sense.angle.data, self.rob_sense.dist.data)
		inc_x = 0.05*np.cos(new_angle)
		inc_y = 0.05*np.sin(new_angle)

	### Services ###
	def dist_calc (self, x1, y1, x2, y2):
		return np.sqrt(((x1-x2)**2)+((y1-y2)**2))		

	def translate_pos (self, angle, dist):
		dx = dist*np.cos(angle)
		dy = dist*np.sin(angle)
		return dx, dy

	def handle_gs_motiv(self, srv):
		if (srv.request.data == True):
			self.endpoint_data()
			(box_dx, box_dy) = self.translate_pos(self.box_sense.angle.data, self.box_sense.dist.data)
			(obj_dx, obj_dy) = self.translate_pos(self.obj_sense.angle.data, self.obj_sense.dist.data)
			(rob_dx, rob_dy) = self.translate_pos(self.rob_sense.angle.data, self.rob_sense.dist.data)

			inc_x = 0.1*np.cos(self.rob_ori.data)
			inc_y = 0.1*np.sin(self.rob_ori.data)
			robg_x = rob_dx + inc_x
			robg_y = rob_dy + inc_y

			#print "robobo before: ", rob_dx, rob_dy
			#Robobo position = pusher position
			'''angle_to_use = self.rob_ori.data
			if self.rob_ori.data < 0.0:
				if (self.rob_ori.data < -1.57):
					angle_to_use = -3.14-self.rob_ori.data
			else:
				if (self.rob_ori.data > 1.57):
					angle_to_use = 3.14-self.rob_ori.data'''

			if self.grip_state_conversion(self.rgrip_sense.data):
				box_ball_dist = self.dist_calc (box_dx, box_dy, self.rarm_state.current_es.pose.position.x, self.rarm_state.current_es.pose.position.y)
				hand_ball_dist = 0.0
				rob_ball_dist = self.dist_calc (robg_x, robg_y, self.rarm_state.current_es.pose.position.x, self.rarm_state.current_es.pose.position.y)
			else:
				if self.rob_grip > 0.0:
					rob_ball_dist = 0.0
					box_ball_dist = self.dist_calc (box_dx, box_dy, robg_x, robg_y)
					hand_ball_dist = self.dist_calc (self.rarm_state.current_es.pose.position.x, self.rarm_state.current_es.pose.position.y, robg_x, robg_y)
				else:
					rob_ball_dist = self.dist_calc(robg_x,robg_y,obj_dx,obj_dy) #np.sqrt((self.dist_calc(rob_dx,rob_dy,obj_dx,obj_dy)**2) - (0.11**2))
					box_ball_dist = self.dist_calc (box_dx, box_dy, obj_dx, obj_dy) #rob_dx, rob_dy)
					hand_ball_dist = self.dist_calc (self.rarm_state.current_es.pose.position.x, self.rarm_state.current_es.pose.position.y, obj_dx, obj_dy)

			if self.grip_state_conversion(self.rgrip_sense.data):
				obj_dx = self.rarm_state.current_es.pose.position.x
				obj_dy = self.rarm_state.current_es.pose.position.y
			elif self.rob_grip > 0.0:
				obj_dx = robg_x
				obj_dy = robg_y
				
			#print 'box_obj: ', box_ball_dist, 'rob_obj: ', rob_ball_dist, 'obj_dx: ', Float64(obj_dx), 'obj_dy: ', Float64(obj_dy), 'box_dx: ', Float64(box_dx), 'box_dy: ', Float64(box_dy), 'rob_dx: ',Float64(rob_dx), 'rob_dy: ', Float64(rob_dy), 'rob_ori: ', Float64(-self.rob_ori.data+3.14)
			print 'rob_obj: ', rob_ball_dist, 'hand_obj: ', hand_ball_dist, 'box_obj: ', box_ball_dist

			return Float64(box_ball_dist), Float64(hand_ball_dist), Float64(rob_ball_dist), Float64(obj_dx), Float64(obj_dy), Float64(box_dx), Float64(box_dy), Float64(self.rarm_state.current_es.pose.position.x), Float64(self.rarm_state.current_es.pose.position.y), Float64(rob_dx), Float64(rob_dy), Float64(self.rgrip_ori), Float64(self.rob_ori.data)

	def handle_gs(self, srv):
		if (srv.request.data == True):
			return self.obj_sense, self.box_sense, self.lgrip_sense, self.rgrip_sense 	

	def handle_gs_fm (self, srv):
		if (srv.request.data == True):
			self.endpoint_data()
			(obj_dx, obj_dy) = self.translate_pos(self.obj_sense.angle.data, self.obj_sense.dist.data)

			hand_ball_dist = self.dist_calc(obj_dx, obj_dy, self.larm_state.current_es.pose.position.x, self.larm_state.current_es.pose.position.y)
			hand_ball_angle = np.arctan2(obj_dy-self.larm_state.current_es.pose.position.y, obj_dx-self.larm_state.current_es.pose.position.x)
			#print obj_dx, self.larm_state.current_es.pose.position.x, obj_dy, self.rarm_state.current_es.pose.position.y
			#print hand_ball_angle

			return Float64(hand_ball_dist), Float64(hand_ball_angle)

	def choose_gripper_state(self, side):
		options = {
			"left":self.lgrip_sense.data,
			"right":self.rgrip_sense.data,
		}
		return options[side]

	def assign_gripper_sense(self, side, value):
		if side == "left":
			self.lgrip_sense.data = value;
		elif side == "right":
			self.rgrip_sense.data = value;
		elif side == "both":
			self.lgrip_sense.data = value;
			self.rgrip_sense.data = value;

	def grip_state_conversion (self, state):
		if state > 0.5:
			return True
		else:
			return False

	def angle_conversion (self, angle):
		return -math.degrees(angle)
		
	def publish_current_senses (self, box_rad, obj_rad):
		#print "\nSENSE: \nobj > ", self.obj_sense,"\nbox > ", self.box_sense, " \ngrippers > ", self.lgrip_sense, self.rgrip_sense 	

		self.obj_dist_pb.publish(Float64(self.obj_sense.dist.data*100.0))
		self.obj_ang_pb.publish(Float64(self.angle_conversion(self.obj_sense.angle.data)))
		#self.obj_size_pb.publish(Float64(self.obj_sense.radius.data*100.0))
		self.obj_size_pb.publish(Float64(obj_rad*100.0))

		self.box_dist_pb.publish(Float64(self.box_sense.dist.data*100.0))
		self.box_ang_pb.publish(Float64(self.angle_conversion(self.box_sense.angle.data)))
		#self.box_size_pb.publish(Float64(self.box_sense.radius.data*100.0))
		self.box_size_pb.publish(Float64(box_rad*100.0))

		self.lgrip_sense_pb.publish(Bool(self.grip_state_conversion(self.lgrip_sense.data)))
		self.rgrip_sense_pb.publish(Bool(self.grip_state_conversion(self.rgrip_sense.data)))
