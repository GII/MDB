"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# Standard imports
import math

# Library imports
import rospy
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from baxter_core_msgs.msg import HeadPanCommand, HeadState

# MDB imports
from mdb_common.msg import ObjDet
from mdb_robots_policies.srv import GridCalibration


class calibration_policies(object):
    def __init__(self, global_policies):
        self.global_policies = global_policies

        self.calibration_data = []

        self.head_state = None
        self.calib_obj_flag = False
        self.calib_obj_cent = None

        self.pan_pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size=1)

        self.show_calib_data_sb = rospy.Subscriber("/baxter/show_calibration_data", Bool, self.show_calib_data_cb)
        self.calib_obj_flag_sb = rospy.Subscriber("/tracking/ball_flag", Bool, self.calib_obj_flag_cb)
        self.ball_track_sb = rospy.Subscriber("/tracking/ball", ObjDet, self.ball_track_cb)
        self.head_state_sb = rospy.Subscriber("/robot/head/head_state", HeadState, self.head_state_cb)
        self.tag_cent_sb = rospy.Subscriber("/aruco_single_head/pixel", PointStamped, self.tag_cent_cb)

        self.baxter_int_calib_srver = rospy.Service(
            "/baxter_int_calib", GridCalibration, self.interpolation_calibration
        )

    #################
    ### Callbacks ###
    #################

    def show_calib_data_cb(self, do_show):
        if do_show.data == True:
            rospy.loginfo(self.calibration_data)

    def calib_obj_flag_cb(self, flag):
        self.calib_obj_flag = flag.data

    def head_state_cb(self, state):
        self.head_state = state

    def ball_track_cb(self, objsens):
        if self.calib_obj_flag:
            self.calib_obj_cent = [objsens.u.data, objsens.v.data]

    def tag_cent_cb(self, Pose):
        if self.global_policies.robobo_status:
            self.calib_obj_cent = [Pose.point.x, Pose.point.y]
        else:
            self.calib_obj_cent = "None"

    ################################
    ### Interpolated calibration ###
    ################################

    def lineal_quadrilateral_grid_points(self, vertex, grid_points):
        points = int(math.sqrt(grid_points) - 1)
        grid = []

        cont = 0
        for j in range(0, points + 1):
            for i in range(0, points + 1):
                a0 = float((points - i) * (points - j)) / float(points ** 2)
                a1 = float((i) * (points - j)) / float(points ** 2)
                a2 = float((i) * (j)) / float(points ** 2)
                a3 = float((points - i) * (j)) / float(points ** 2)

                grid.append(
                    [
                        a0 * vertex[0][0] + a1 * vertex[1][0] + a2 * vertex[2][0] + a3 * vertex[3][0],
                        a0 * vertex[0][1] + a1 * vertex[1][1] + a2 * vertex[2][1] + a3 * vertex[3][1],
                    ]
                )
        return grid

    def select_arm_to_move(self, y_c):
        if y_c >= 0.0:
            return "left"
        else:
            return "right"

    def pan_to_sp(self, pos, speed):
        self.pan_pub.publish(HeadPanCommand(pos, speed, 0.0))
        rospy.sleep(0.5)
        while self.head_state.isTurning:
            pass

    def calibration_move_loop(self, height_data, side, dx, dy):
        result = False
        if self.global_policies.loop_tries > 0:
            if self.global_policies.adjust_w1(side, dx, dy):
                if self.global_policies.baxter_arm.move_xyz(
                    dx, dy, height_data + self.global_policies.safe_operation_height, False, "current", side, 1.0, 1.0
                ):
                    if self.global_policies.baxter_arm.move_xyz(dx, dy, height_data, False, "current", side, 1.0, 1.0):
                        input("Place the object to detect in the gripper position")
                        pos = self.global_policies.baxter_arm.choose_arm_state(side).current_es.pose.position
                        real_position = [pos.x, pos.y]
                        if self.global_policies.baxter_arm.move_xyz(
                            dx,
                            dy,
                            height_data + self.global_policies.safe_operation_height,
                            False,
                            "current",
                            side,
                            1.0,
                            1.0,
                        ):
                            self.global_policies.loop_tries = 5
                            result = real_position
                else:
                    print("Adjusting")
                    self.global_policies.loop_tries -= 1
                    self.calibration_move_loop(height_data, side, dx, dy)
        return result

    def calibration_move_close(self, height_data, side, dx, dy):
        result = False
        if self.global_policies.baxter_arm.move_xyz(
            dx, dy, height_data + self.global_policies.safe_operation_height, False, "current", side, 1.0, 1.0
        ):
            if self.global_policies.baxter_arm.move_xyz(dx, dy, height_data, False, "current", side, 1.0, 1.0):
                input("Place the object to detect in the gripper position")
                pos = self.global_policies.baxter_arm.choose_arm_state(side).current_es.pose.position
                real_position = [pos.x, pos.y]
                if self.global_policies.baxter_arm.move_xyz(
                    dx, dy, height_data + self.global_policies.safe_operation_height, False, "current", side, 1.0, 1.0
                ):
                    return real_position
        return result

    def angle_generator(self, number_of_angles, angle_seed):
        list_of_ang = []

        if number_of_angles > 1:
            list_of_ang.append(angle_seed)
            inc = old_div(angle_seed * 2, (number_of_angles - 1))
            for it in range(0, number_of_angles - 1):
                angle_seed -= inc
                list_of_ang.append(angle_seed)

        return list_of_ang

    def translate_flag(self, use_tag):
        if use_tag:
            return "tag"
        else:
            return "cylinder"

    def select_calibration_flag(self, flag):
        options = {
            "tag": self.global_policies.robobo_status,
            "cylinder": self.calib_obj_flag,
        }
        return options[flag]

    def pan_data_adquisition(self, pan_angles, use_tag):
        self.global_policies.adopt_oap()
        pos_data = []
        if len(pan_angles) > 1:
            for pan in pan_angles:
                self.pan_to_sp(pan, 0.05)
                rospy.sleep(1.0)
                if not self.select_calibration_flag(self.translate_flag(use_tag)):
                    self.calib_obj_cent = "None"
                pos_data.append([self.head_state.pan, self.calib_obj_cent])
        else:
            if not self.select_calibration_flag(self.translate_flag(use_tag)):
                self.calib_obj_cent = "None"
            print(self.calib_obj_cent)
            pos_data.append([self.calib_obj_cent])
        self.pan_to_sp(0.0, 0.05)
        return pos_data

    def calibration_move_far(self, height_data, arm, dx, dy):
        self.global_policies.pose_grab_far(arm, dx, dy)
        real_position = self.calibration_move_loop(height_data, arm, dx, dy)
        return real_position

    def translate_move_flag(self, dist, far):
        if dist <= far:
            return "close"
        else:
            return "far"

    def select_calibration_move(self, flag):
        options = {
            "close": self.calibration_move_close,
            "far": self.calibration_move_far,
        }
        return options[flag]

    def calibration_cycle(self, dist, far, pan_angles, req, calibration_data, dx, dy, arm, non_reach):
        if req.use_arm.data:
            if non_reach:
                calibration_data.append([[dx, dy], "non_reachable"])
                return True
            else:
                real_position = self.select_calibration_move(self.translate_move_flag(dist, far))(
                    req.height.data, arm, dx, dy
                )
                if real_position != False:
                    rospy.sleep(1)
                    pos_data = self.pan_data_adquisition(pan_angles, req.use_tag.data)
                    calibration_data.append([real_position, pos_data])
                    return True
                else:
                    self.global_policies.loop_tries = 5
                    self.global_policies.adopt_oap()
                    calibration_data.append([[dx, dy], "non_reachable"])
                    return False

        else:
            rospy.sleep(1)
            pos_data = self.pan_data_adquisition(pan_angles, req.use_tag.data)
            calibration_data.append([[dx, dy], pos_data])
            self.global_policies.baxter_arm.restore_arm_pose("both")
            return True

    def obtain_calibration_limits(self, req):
        if req.row_to_calibrate.data == -1 and req.pos_to_start.data < req.point_num.data:
            return req.pos_to_start.data, req.point_num.data
        else:
            side = int(math.sqrt(req.point_num.data))
            return side * req.row_to_calibrate.data, (side * req.row_to_calibrate.data) + side

    def interpolation_calibration(self, req):
        x_min = rospy.get_param("~cal_xmin")
        x_max = rospy.get_param("~cal_xmax")
        y_min = rospy.get_param("~cal_ymin")
        y_max = rospy.get_param("~cal_ymax")

        seed_angle = rospy.get_param("~cal_ang")

        height_data = req.height.data
        table_vertex = [[x_min, y_max], [x_min, y_min], [x_max, y_min], [x_max, y_max]]
        pan_angles = self.angle_generator(req.angle_num.data, seed_angle)

        self.global_policies.adopt_oap()
        grid_points = self.lineal_quadrilateral_grid_points(table_vertex, req.point_num.data)

        self.calibration_data = []

        (init, end) = self.obtain_calibration_limits(req)

        row_point_num = int(math.sqrt(req.point_num.data))
        rospy.set_param("baxter_sense", True)

        fail = True
        non_reach = False
        for pose in range(init, end):

            if (pose % row_point_num) == 0:
                fail = True

            dx = grid_points[pose][0]
            dy = grid_points[pose][1]

            # Check the arm to use based on the y coordinate:
            arm = self.select_arm_to_move(dy)

            # Translate position in terms of angle and distance
            dist = self.global_policies.obtain_dist(dx, dy)
            ang = math.atan2(dy, dx)

            far = self.global_policies.g_highpoly(abs(ang))
            self.global_policies.baxter_arm.restore_arm_pose(arm)

            result = self.calibration_cycle(dist, far, pan_angles, req, self.calibration_data, dx, dy, arm, non_reach)

            fail = fail and not result
            if ((pose + 1) % row_point_num) == 0 and fail:
                non_reach = True

            input("\nPosition " + str(pose) + ": information obtained. Press the button to calibrate the next position")

        rospy.delete_param("baxter_sense")
        self.global_policies.baxter_arm.restore_arm_pose("both")
        rospy.loginfo(self.calibration_data)
        return Bool(True)
