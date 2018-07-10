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

import rospy, cv2, re, math, vlc
import numpy as np
from std_msgs.msg import String
from mdb_common.srv import ExecPolicy, ExecPolicyRequest

class exp_helper():
	def __init__(self):
		self.exec_pol_clnt = rospy.ServiceProxy("/mdb/baxter/exec_policy", ExecPolicy)  
		self.executed_policy_sub = rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_cb)

	def executed_policy_cb (self, policy_code):
		req = ExecPolicyRequest()
		req.policy_code.data = policy_code.data
		resp = self.exec_pol_clnt(req)		

def main():
	exp_helper()
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("exp_helper")
	main()	

