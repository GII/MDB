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
from geometry_msgs.msg import Vector3
import serial

class arduino_joystick_module():
	def __init__(self):
		self.blu = None
		self.joystick_pub = rospy.Publisher("/joystick/data", Vector3, queue_size = 1)

	def translate_input (self, data):
		button, coor = data.split('=')
		x, y = coor.split(':')
		return int(button), int(x), int(y)

	def pair_device_as_serial (self, port, baudrate):
		self.blu = serial.Serial(port, baudrate)
		self.blu.flushInput()

	def listen_to_joystick(self):
		msg = Vector3()
		while not rospy.is_shutdown():
			input_data = self.blu.readline()
			if len(input_data) > 6 and input_data.find('=') and input_data.find(':'):
				(button, x, y) = self.translate_input(input_data)
				msg.x = x
				msg.y = y
				self.joystick_pub.publish(msg)

	def initialize_bluetooth_connection(self):
		self.pair_device_as_serial('/dev/rfcomm0', 9600)
		rospy.loginfo("Listening to the joystick...")
		self.listen_to_joystick()
		self.blu.close()
			
if __name__ == '__main__':
	rospy.init_node("arduino_joystick_module")
	ajm = arduino_joystick_module()
	ajm.initialize_bluetooth_connection()
	rospy.spin()
