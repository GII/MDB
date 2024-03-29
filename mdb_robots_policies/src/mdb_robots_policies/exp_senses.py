"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function
from future import standard_library

standard_library.install_aliases()
from builtins import object

# Standard imports
import math

# Library imports
import rospy
import numpy as np
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped

# MDB imports
from mdb_common.msg import SensData, ObjectMsg, ObjectListMsg
from mdb_common.srv import GetSenseMotiv
from mdb_robots_policies.srv import GetSense, GetRobSense


class exp_senses(object):
    def __init__(self, global_policies):
        self.global_policies = global_policies

        self.obj_sense = SensData()
        self.box_sense = SensData()
        self.box2_sense = SensData()
        self.rob_sense = SensData()
        self.rob_ori = Float64()
        self.rob_obj_ori = Float64()
        self.rob_box_ori = Float64()
        self.rob_box2_ori = Float64()

        self.rgrip_ori = 0.0
        self.rob_grip = 0.0

        self.obj_sense_sb = rospy.Subscriber("/mdb_baxter/ball", SensData, self.obj_sense_cb)
        self.box_sense_sb = rospy.Subscriber("/mdb_baxter/box", SensData, self.box_sense_cb)
        self.box2_sense_sb = rospy.Subscriber("/mdb_baxter/box2", SensData, self.box2_sense_cb)
        self.rob_sense_sb = rospy.Subscriber("/mdb_baxter/robobo", SensData, self.rob_sense_cb)

        self.mixed_box_sense_sb = rospy.Subscriber("/baxter_throwing/basket_pose", PoseStamped, self.mixed_box_cb)
        self.mixed_ball_sense_sb = rospy.Subscriber("/baxter_throwing/ball_position", PoseStamped, self.mixed_ball_cb)

        self.rob_ori_sb = rospy.Subscriber("/tracking/robobo_ori", Float64, self.rob_ori_cb)
        self.rob_ori_obj_sb = rospy.Subscriber("/tracking/robobo_ori_obj", Float64, self.rob_ori_obj_cb)
        self.rob_ori_box_sb = rospy.Subscriber("/tracking/robobo_ori_box", Float64, self.rob_ori_box_cb)
        self.rob_ori_box2_sb = rospy.Subscriber("/tracking/robobo_ori2_box", Float64, self.rob_ori_box2_cb)

        self.obj = ObjectListMsg()
        self.obj.data = []
        self.obj.data.append(ObjectMsg())
        self.obj_pb = rospy.Publisher("/mdb/baxter/sensor/cylinders", ObjectListMsg, queue_size=1)
        self.box = ObjectListMsg()
        self.box.data = []
        self.box.data.append(ObjectMsg())
        self.box.data.append(ObjectMsg())
        self.box_pb = rospy.Publisher("/mdb/baxter/sensor/boxes", ObjectListMsg, queue_size=1)

        self.lgrip_sense = Float64(0.0)
        self.rgrip_sense = Float64(0.0)
        self.lgrip_sense_pb = rospy.Publisher("/mdb/baxter/sensor/ball_in_left_hand", Bool, queue_size=1)
        self.rgrip_sense_pb = rospy.Publisher("/mdb/baxter/sensor/ball_in_right_hand", Bool, queue_size=1)

        self.reward = Bool()
        self.reward_pb = rospy.Publisher("/mdb/baxter/sensor/happy_human", Bool, queue_size=1)

        self.gs_srver = rospy.Service("/baxter/get_sense", GetSense, self.handle_gs)
        self.gs_motiv_srver = rospy.Service("/mdb3/baxter/sensors", GetSenseMotiv, self.handle_gs_motiv)
        self.gs_rob_data_srver = rospy.Service("/baxter/get_rob_data_sens", GetRobSense, self.handle_gs_rob_data)

        ### Callbacks ###

    def mixed_box_cb(self, sense):
        if rospy.has_param("/baxter_sense"):
            do_sense = rospy.get_param("/baxter_sense")
            if do_sense:
                (angle, dist) = self.global_policies.cartesian_to_polar(sense.pose.position.y, sense.pose.position.x)
                self.box_sense.dist = dist - 0.08
                self.box_sense.angle = angle
                self.box_sense.height = -0.04
                self.box_sense.radius = 0.0

    def mixed_ball_cb(self, sense):
        if rospy.has_param("/baxter_sense"):
            do_sense = rospy.get_param("/baxter_sense")
            if do_sense:
                (angle, dist) = self.global_policies.cartesian_to_polar(sense.pose.position.y, sense.pose.position.x)
                self.obj_sense.dist = dist
                self.obj_sense.angle = angle
                self.obj_sense.height = -0.04
                self.obj_sense.radius = 0.0

    def obj_sense_cb(self, sens):
        if sens.height > -0.10:
            self.obj_sense = sens

    def box_sense_cb(self, sens):
        if sens.height > -0.10 and not rospy.has_param("/check_reward"):  # and rospy.has_param("/box_sense"):
            self.box_sense = sens

    def box2_sense_cb(self, sens):
        if sens.height > -0.10 and not rospy.has_param("/check_reward"):  # and rospy.has_param("/box2_sense"):
            self.box2_sense = sens

    def rob_sense_cb(self, sens):
        if sens.height > -0.10:
            self.rob_sense = sens

    def rob_ori_cb(self, ori):
        self.rob_ori = ori

    def rob_ori_obj_cb(self, ori):
        self.rob_obj_ori = ori

    def rob_ori_box_cb(self, ori):
        self.rob_box_ori = ori

    def rob_ori_box2_cb(self, ori):
        self.rob_box2_ori = ori

    ### Services ###

    def handle_gs_motiv(self, srv):
        if srv.request == True:
            (box_dx, box_dy) = self.global_policies.polar_to_cartesian(self.box_sense.angle, self.box_sense.dist)
            (obj_dx, obj_dy) = self.global_policies.polar_to_cartesian(self.obj_sense.angle, self.obj_sense.dist)
            (rob_dx, rob_dy) = self.global_policies.polar_to_cartesian(self.rob_sense.angle, self.rob_sense.dist)

            inc_x = 0.1 * np.cos(self.rob_ori)
            inc_y = 0.1 * np.sin(self.rob_ori)
            robg_x = rob_dx + inc_x
            robg_y = rob_dy + inc_y

            if self.grip_state_conversion(self.rgrip_sense):
                box_ball_dist = self.global_policies.obtain_dist(
                    box_dx - self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x,
                    box_dy - self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y,
                )
                hand_ball_dist = 0.0
                rob_ball_dist = self.global_policies.obtain_dist(
                    robg_x - self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x,
                    robg_y - self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y,
                )
            else:
                if self.rob_grip > 0.0:
                    rob_ball_dist = 0.0
                    box_ball_dist = self.global_policies.obtain_dist(box_dx - robg_x, box_dy - robg_y)
                    hand_ball_dist = self.global_policies.obtain_dist(
                        self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x - robg_x,
                        self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y - robg_y,
                    )
                else:
                    rob_ball_dist = self.global_policies.obtain_dist(robg_x - obj_dx, robg_y - obj_dy)
                    box_ball_dist = self.global_policies.obtain_dist(box_dx - obj_dx, box_dy - obj_dy)
                    hand_ball_dist = self.global_policies.obtain_dist(
                        self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x - obj_dx,
                        self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y - obj_dy,
                    )

            if self.grip_state_conversion(self.rgrip_sense):
                obj_dx = self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x
                obj_dy = self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y
            elif self.rob_grip > 0.0:
                obj_dx = robg_x
                obj_dy = robg_y

            return (
                Float64(box_ball_dist),
                Float64(hand_ball_dist),
                Float64(rob_ball_dist),
                Float64(obj_dx),
                Float64(obj_dy),
                Float64(box_dx),
                Float64(box_dy),
                Float64(self.global_policies.baxter_arm.rarm_state.current_es.pose.position.x),
                Float64(self.global_policies.baxter_arm.rarm_state.current_es.pose.position.y),
                Float64(rob_dx),
                Float64(rob_dy),
                Float64(self.rgrip_ori),
                Float64(self.rob_ori),
            )

    def handle_gs(self, srv):
        if srv.request == True:
            return self.obj_sense, self.box_sense, self.lgrip_sense, self.rgrip_sense

    def handle_gs_rob_data(self, srv):
        (box_dx, box_dy) = self.global_policies.polar_to_cartesian(self.box_sense.angle, self.box_sense.dist)
        (rob_dx, rob_dy) = self.global_policies.polar_to_cartesian(self.rob_sense.angle, self.rob_sense.dist)
        rob_box_dist = self.global_policies.obtain_dist(box_dx - rob_dx, box_dy - rob_dy)

        return (
            Float64(box_dx),
            Float64(box_dy),
            Float64(rob_dx),
            Float64(rob_dy),
            Float64(self.rob_ori),
            Float64(rob_box_dist),
        )

    def choose_gripper_state(self, side):
        options = {"left": self.lgrip_sense, "right": self.rgrip_sense}
        return options[side]

    def assign_gripper_sense(self, side, value):
        if side == "left":
            self.lgrip_sense = value
        elif side == "right":
            self.rgrip_sense = value
        elif side == "both":
            self.lgrip_sense = value
            self.rgrip_sense = value

    def grip_state_conversion(self, state):
        if state > 0.5:
            return True
        else:
            return False

    def angle_conversion(self, angle):
        return -math.degrees(angle)

    outer = np.poly1d(np.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [1.15, 1.25, 1.325, 1.375, 1.375], 3))

    @classmethod
    def object_too_far(cls, dist, ang):
        """Return True if the object is out of range of the robot."""
        return dist > cls.outer(abs(ang))

    def is_human_happy(self):
        for cylinder in self.box:
            for box in self.obj:
                if (
                    (abs(cylinder.distance - box.distance) < 0.18)
                    and (abs(cylinder.angle - box.angle) < 0.18)
                    and self.object_too_far(box.distance, box.angle)
                ):
                    return True
        return False

    def publish_current_senses(self, box_rad, obj_rad):
        rospy.loginfo("Publishing sensorization")

        self.obj.data[0].distance = self.obj_sense.dist
        self.obj.data[0].angle = self.obj_sense.angle
        self.obj.data[0].diameter = obj_rad
        self.obj_pb.publish(self.obj)

        self.box.data[0].distance = self.box_sense.dist
        self.box.data[0].angle = self.box_sense.angle
        self.box.data[0].diameter = box_rad
        self.box.data[1].distance = self.box2_sense.dist
        self.box.data[1].angle = self.box2_sense.angle
        self.box.data[1].diameter = box_rad
        self.box_pb.publish(self.box)

        self.lgrip_sense_pb.publish(Bool(self.grip_state_conversion(self.lgrip_sense)))
        self.rgrip_sense_pb.publish(Bool(self.grip_state_conversion(self.rgrip_sense)))

        self.reward.data = self.is_human_happy()
        self.reward_pb.publish(self.reward)
