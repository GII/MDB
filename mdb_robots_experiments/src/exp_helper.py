#!/usr/bin/env python

"""
MDB.

https://github.com/GII/MDB
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *
from builtins import object
import rospy
from std_msgs.msg import String
from mdb_common.srv import ExecPolicy, ExecPolicyRequest


class exp_helper(object):
    def __init__(self):
        self.exec_pol_clnt = rospy.ServiceProxy("/mdb/baxter/exec_policy", ExecPolicy)
        self.executed_policy_sub = rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_cb)

    def executed_policy_cb(self, policy_code):
        req = ExecPolicyRequest()
        req.policy_code.data = policy_code.data
        resp = self.exec_pol_clnt(req)


def main():
    exp_helper()
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("exp_helper")
    main()
