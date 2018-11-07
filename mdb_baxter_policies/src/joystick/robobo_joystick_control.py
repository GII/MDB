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

import rospy, math
from com_mytechia_robobo_ros_msgs.srv import Command
from com_mytechia_robobo_ros_msgs.msg import KeyValue, Status
from geometry_msgs.msg import Vector3

class robobo_joystick_control():
	def __init__(self):
    		
		self.neutral_x = 507.0
		self.neutral_y = 499.0
		self.max_velocity = rospy.get_param("~joystick_velocity")
		self.loop_rate = rospy.get_param("~joystick_rate")

		self.max_module = math.sqrt((self.neutral_x**2)+(self.neutral_y**2))		
		self.joystick_data = None

		try:
			self.robobo_command = rospy.ServiceProxy('/command', Command)
		except rospy.ServiceException, e:
			print "Service exception", str(e)
			exit(1)

		self.rob_joy_sb = rospy.Subscriber("/joystick/data", Vector3, self.rob_joy_cb)

	def rob_joy_cb (self, msg):
		self.joystick_data = msg

	def normalize_joystick_values(self, x, y):
		return x-self.neutral_x, y-self.neutral_y

	def is_neutral (self, x, y):
		result = False
		if (-50<x<50) and (-50<y<50):
    			result  = True
		return result	

	def adquire_wheels_speed (self, x, y, turn_coef=3):
		y = -y 
		x = -x
		alpha = 0.5*math.atan2(y, x)				
		m = math.sqrt((x**2)+(y**2))
		beta = alpha + 0.785	
		epsilon=0.00001
		left_speed = (self.max_velocity/self.max_module)*m*math.cos(beta) 
		right_speed = (self.max_velocity/self.max_module)*m*math.sin(beta)

		return left_speed, right_speed

	def execute_command(self, name, parameters, values, command_id):
		self.robobo_command.wait_for_service()
		command_name = name
		command_parameters = []
		for it in range(0, len(parameters)):
			command_parameters.append(KeyValue(parameters[it], values[it]))
		self.robobo_command(command_name, command_id, command_parameters)

	def joystick_rob_move(self, x, y, time):
		(l_wheel, r_wheel) = self.adquire_wheels_speed(x, y)
		self.execute_command('MOVE-BLOCKING', ['lspeed', 'rspeed', 'time', 'blockid'], [str(int(l_wheel)), str(int(r_wheel)), str(time), '0'], 0)

	def control_robobo(self):
		while not rospy.is_shutdown():
			if self.joystick_data:
				rospy.loginfo("command: "+str(self.joystick_data.x)+" "+str(self.joystick_data.y)+" "+str(self.joystick_data.z))
				(nx, ny) = self.normalize_joystick_values(self.joystick_data.x, self.joystick_data.y)
				self.joystick_rob_move(ny, nx, 1.5)
			self.loop_rate.sleep()
			
if __name__ == '__main__':
	rospy.init_node("robobo_joystick_control")
	rjc = robobo_joystick_control()
	rjc.control_robobo()
	rospy.spin()
