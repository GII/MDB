#!/usr/bin/env python
import rospy, cv2, re, math, vlc
import numpy as np
from std_msgs.msg import String
from mdb_common.srv import ExecPolicy, ExecPolicyRequest

class exp_ltm_helper():
	def __init__(self):
		self.exec_pol_clnt = rospy.ServiceProxy("/mdb/baxter/exec_policy", ExecPolicy)  
		self.executed_policy_sub = rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_cb)

	def executed_policy_cb (self, policy_code):
		req = ExecPolicyRequest()
		req.policy_code = policy_code
		resp = self.exec_pol_clnt(req)		

def main():
	exp_ltm_helper()
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("exp_ltm_helper")
	main()	

