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
import copy

# Library imports
import rospy
import moveit_commander
import rospkg
import tf
from baxter_core_msgs.msg import EndpointState, EndEffectorState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath, ExecuteKnownTrajectory, GetCartesianPathRequest
from baxter_interface.gripper import Gripper

# MDB imports
from mdb_robots_policies.srv import GetJointsState, GetEndState


class baxter_arm(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()

        self.larm_state = EndpointState()
        self.rarm_state = EndpointState()
        self.joint_states = JointState()
        self.larm_init_state = EndpointState()
        self.rarm_init_state = EndpointState()
        self.joint_init_states = JointState()

        try:
            self.compute_cp = rospy.ServiceProxy("/compute_cartesian_path", GetCartesianPath)
            self.execute_kp = rospy.ServiceProxy("/execute_kinematic_path", ExecuteKnownTrajectory)
            self.get_es = rospy.ServiceProxy("/get_end_state", GetEndState)
            self.get_js = rospy.ServiceProxy("/get_joints_state", GetJointsState)
        except rospy.ServiceException as e:
            print("Service call failed: ", str(e))
            exit(1)

        self.both_group = moveit_commander.MoveGroupCommander("both_arms")
        self.larm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.rarm_group = moveit_commander.MoveGroupCommander("right_arm")

        self.lgripper = Gripper("left")
        self.lgripper_state = False
        self.lgripper.calibrate()

        self.rgripper = Gripper("right")
        self.rgripper_state = False
        self.rgripper.calibrate()

        self.lgripper_instate = EndEffectorState()
        self.rgripper_instate = EndEffectorState()
        self.lgripper_instate_sub = rospy.Subscriber(
            "/robot/end_effector/left_gripper/state", EndEffectorState, self.lgripper_in_cb
        )
        self.rgripper_instate_sub = rospy.Subscriber(
            "/robot/end_effector/right_gripper/state", EndEffectorState, self.rgripper_in_cb
        )

    ####################
    ###   Callbacks  ###
    ####################

    def lgripper_in_cb(self, instate):
        self.lgripper_instate = instate

    def rgripper_in_cb(self, instate):
        self.rgripper_instate = instate

    ####################
    ### Dictionaries ###
    ####################

    def choose_gripper_instate(self, arg):
        options = {
            "left": self.lgripper_instate,
            "right": self.rgripper_instate,
        }
        return options[arg]

    # Choose between baxter's natural arm joints order and the moveit one.
    def choose_joints_order(self, arg):
        options = {
            "baxter": [2, 3, 0, 1, 4, 5, 6],
            "moveit": [0, 1, 2, 3, 4, 5, 6],
        }
        return options[arg]

    def choose_arm_group(self, arg):
        options = {
            "right": self.rarm_group,
            "left": self.larm_group,
            "both": self.both_group,
        }
        return options[arg]

    def choose_arm_state(self, arg):
        options = {
            "right": self.rarm_state,
            "left": self.larm_state,
        }
        return options[arg]

    def choose_arm_init_state(self, arg):
        options = {
            "right": self.rarm_init_state,
            "left": self.larm_init_state,
        }
        return options[arg]

    def choose_gripper(self, arg):
        options = {
            "right": self.rgripper,
            "left": self.lgripper,
        }
        return options[arg]

    def choose_gripper_state(self, arg):
        options = {
            "right": self.rgripper_state,
            "left": self.lgripper_state,
        }
        return options[arg]

    ##########################
    ###      Pose Goal     ###
    ##########################

    def move_to_pose_goal(self, pose, side, wait, scale):
        self.wait_to_move()
        self.update_data()
        self.choose_arm_group(side).clear_pose_targets()
        self.choose_arm_group(side).set_pose_target(pose)

        plan = self.choose_arm_group(side).plan()

        self.change_velocity(plan, scale)
        self.execute_kp(plan, wait)
        self.update_data()
        return True

    ##########################
    ###	   Position Goal   ###
    ##########################
    def move_to_position_goal_both(self, pos, wait, scale):
        self.wait_to_move()
        self.update_data()
        self.choose_arm_group("both").clear_pose_targets()

        l_ori = self.choose_arm_state("left").current_es.pose.orientation
        r_ori = self.choose_arm_state("right").current_es.pose.orientation

        l_pose_target = Pose()
        l_pose_target.orientation = l_ori
        l_pose_target.position.x = pos[0]
        l_pose_target.position.y = pos[1]
        l_pose_target.position.z = pos[2]
        self.choose_arm_group("both").set_pose_target(l_pose_target, "left_gripper")

        r_pose_target = Pose()
        r_pose_target.orientation = r_ori
        r_pose_target.position.x = pos[3]
        r_pose_target.position.y = pos[4]
        r_pose_target.position.z = pos[5]
        self.choose_arm_group("both").set_pose_target(r_pose_target, "right_gripper")

        self.choose_arm_group("both").set_goal_tolerance(0.01)
        plan = self.choose_arm_group("both").plan()
        self.change_velocity(plan, scale)
        try:
            self.execute_kp(plan, wait)
            return True
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

    ##########################
    ###		 Ori Goal      ###
    ##########################

    def move_to_ori_goal_both(self, ori, wait, scale):
        self.wait_to_move()
        self.update_data()
        self.choose_arm_group("both").clear_pose_targets()

        l_pos = self.choose_arm_state("left").current_es.pose.position
        r_pos = self.choose_arm_state("right").current_es.pose.position

        l_pose_target = Pose()
        l_pose_target.position = l_pos
        l_pose_target.orientation.x = ori[0]
        l_pose_target.orientation.y = ori[1]
        l_pose_target.orientation.z = ori[2]
        l_pose_target.orientation.w = ori[3]
        self.choose_arm_group("both").set_pose_target(l_pose_target, "left_gripper")

        r_pose_target = Pose()
        r_pose_target.position = r_pos
        r_pose_target.orientation.x = ori[4]
        r_pose_target.orientation.y = ori[5]
        r_pose_target.orientation.z = ori[6]
        r_pose_target.orientation.w = ori[7]
        self.choose_arm_group("both").set_pose_target(r_pose_target, "right_gripper")

        plan = self.choose_arm_group("both").plan()
        self.change_velocity(plan, scale)
        try:
            self.execute_kp(plan, wait)
            return True
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

    ###################################
    ### Arm joint movement (moveit) ###
    ###################################

    def arrange_angles(self, angles, side, way):
        group_variable_values = None
        if side == "both":
            group_variable_values = angles
        else:
            group_variable_values = list(range(7))
            order = self.choose_joints_order(way)
            for ite in range(0, len(group_variable_values)):
                group_variable_values[ite] = angles[order[ite]]
        return group_variable_values

    def change_velocity(self, plan, scale):
        for point in plan.joint_trajectory.points:
            point.time_from_start = point.time_from_start / scale
            for velocity in point.velocities:
                velocity = velocity * scale
            for acceleration in point.accelerations:
                acceleration = acceleration * scale

    # Returns the joints belonging to one specific arm
    def remove_unnecesary_joints(self, joints, side):
        clean_joint_states = JointState()
        clean_joint_states.header = joints.current_js.header
        for ite in range(0, len(joints.current_js.name)):
            if side == "both":
                if "left" in joints.current_js.name[ite] or "right" in joints.current_js.name[ite]:
                    clean_joint_states.name.append(joints.current_js.name[ite])
                    clean_joint_states.position.append(joints.current_js.position[ite])
                    clean_joint_states.velocity.append(joints.current_js.velocity[ite])
                    clean_joint_states.effort.append(joints.current_js.effort[ite])
            else:
                if side in joints.current_js.name[ite]:
                    clean_joint_states.name.append(joints.current_js.name[ite])
                    clean_joint_states.position.append(joints.current_js.position[ite])
                    clean_joint_states.velocity.append(joints.current_js.velocity[ite])
                    clean_joint_states.effort.append(joints.current_js.effort[ite])
        return clean_joint_states

    # Moves the arm to a specific target of joints angles
    def move_joints_directly(self, angles, way, side, wait, scale):
        self.wait_to_move()
        self.choose_arm_group(side).clear_pose_targets()
        group_variable_values = self.arrange_angles(angles, side, way)

        self.choose_arm_group(side).set_joint_value_target(group_variable_values)
        plan = self.choose_arm_group(side).plan()
        self.change_velocity(plan, scale)
        self.execute_kp(plan, wait)

    # Creates a set of target joints angles based of the initial state of the robot
    def create_group_joints(self, side):
        joints = self.remove_unnecesary_joints(self.joint_init_states, side)
        group_joint_values = self.choose_arm_group(side).get_current_joint_values()
        order = self.choose_joints_order("baxter")
        for ite in range(0, len(group_joint_values)):
            group_joint_values[ite] = joints.position[order[ite]]
        return group_joint_values

    def manage_group_joints(self, side):
        group_joint_values = None
        if side == "both":
            group_joint_values = self.create_group_joints("left") + self.create_group_joints("right")
        else:
            group_joint_values = self.create_group_joints(side)
        return group_joint_values

    # Restores the pose of the arm to its initial state
    def restore_arm_pose(self, side):
        self.wait_to_move()
        group_joint_values = self.manage_group_joints(side)
        rospy.sleep(1)
        self.choose_arm_group(side).clear_pose_targets()
        self.choose_arm_group(side).set_start_state_to_current_state()
        self.choose_arm_group(side).set_joint_value_target(group_joint_values)
        plan = self.choose_arm_group(side).plan()
        self.execute_kp(plan, True)

    def select_joint_names(self, side):
        options = {
            "left": ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"],
            "right": ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"],
            "both": [
                "left_s0",
                "left_s1",
                "left_e0",
                "left_e1",
                "left_w0",
                "left_w1",
                "left_w2",
                "right_s0",
                "right_s1",
                "right_e0",
                "right_e1",
                "right_w0",
                "right_w1",
                "right_w2",
            ],
        }
        return options[side]

    ##############################
    ### Arm cartesian movement ###
    ##############################

    # Creates a pose from a state of the arm
    def create_pose(self, state):
        pos = state.current_es.pose.position
        ori = state.current_es.pose.orientation
        pose_target = Pose(Point(pos.x, pos.y, pos.z), Quaternion(ori.x, ori.y, ori.z, ori.w))
        return pose_target

    # Creates a set of waypoints from the current state of the arm to the destination state
    def generate_wp_xyz(self, x, y, z, code, side):
        start_pose = self.create_pose(self.choose_arm_state(side))

        wpose = Pose()
        if code == "init":
            wpose.orientation = self.choose_arm_init_state(side).current_es.pose.orientation
        elif code == "current":
            wpose.orientation = start_pose.orientation
        elif code == "random":
            wpose.orientation.w = 0.0
            wpose.orientation.x = 1.0
            wpose.orientation.y = 0.0
            wpose.orientation.z = 0.0
        elif code == "joystick":
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                (start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w)
            )
            (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll - 0.10, pitch, yaw)
            wpose.orientation.w = qw
            wpose.orientation.x = qx
            wpose.orientation.y = qy
            wpose.orientation.z = qz

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z

        waypoints = []
        waypoints.append(start_pose)
        waypoints.append(copy.deepcopy(wpose))

        return waypoints

    def select_group_name(self, code):
        options = {
            "left": "left_arm",
            "right": "right_arm",
            "both": "both_arms",
        }
        return options[code]

    # Computes the request for the cartesian moveit service
    def compute_cartesian_req(self, x, y, z, code, side):
        self.update_data()
        arm_joints_state = self.remove_unnecesary_joints(self.joint_states, side)
        waypoints = self.generate_wp_xyz(x, y, z, code, side)

        gcp_req = GetCartesianPathRequest()
        gcp_req.header = self.create_header("base")
        gcp_req.start_state.joint_state = arm_joints_state
        gcp_req.group_name = self.select_group_name(side)
        gcp_req.link_name = str(side) + "_gripper"
        gcp_req.waypoints = waypoints
        gcp_req.max_step = 0.01
        gcp_req.avoid_collisions = True

        return gcp_req

    # Moves the arm to a specific waypoint in the robot 3d space and opens/closes the gripper if desired
    def move_xyz(self, x, y, z, pick, code, side, scale, perc):
        self.wait_to_move()
        self.update_data()
        gcp_req = self.compute_cartesian_req(x, y, z, code, side)
        try:
            fract = 0.0
            resp = None
            tries = 6
            while fract < perc and tries > 0:
                resp = self.compute_cp(gcp_req)
                fract = resp.fraction
                tries -= 1
            if tries == 0:
                return False
            if resp.error_code.val == 1:
                self.change_velocity(resp.solution, scale)
                pet = self.execute_kp(resp.solution, True)
                if (pet.error_code.val == 1 or pet.error_code.val == -4) and pick:
                    self.gripper_manager(side)
                    rospy.sleep(0.5)
                self.update_data()
                return True

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

    def move_xyz_plan(self, x, y, z, code, side, perc):
        self.wait_to_move()
        self.update_data()
        gcp_req = self.compute_cartesian_req(x, y, z, code, side)
        try:
            fract = 0.0
            resp = None
            tries = 5
            while fract < perc and tries > 0:
                resp = self.compute_cp(gcp_req)
                fract = resp.fraction
                tries -= 1
            if tries == 0:
                return False
            if resp.error_code.val == 1:
                return resp.solution
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

    def move_xyz_concatenate(self, l_plan, r_plan):
        l_points = l_plan.joint_trajectory.points
        r_points = r_plan.joint_trajectory.points
        # Concatenate both solutions into one solution
        if len(r_points) > len(l_points):
            r_plan.joint_trajectory.joint_names = (
                l_plan.joint_trajectory.joint_names + r_plan.joint_trajectory.joint_names
            )
            for ite in range(0, len(l_points)):
                r_points[ite].positions = l_points[ite].positions + r_points[ite].positions
                r_points[ite].velocities = l_points[ite].velocities + r_points[ite].velocities
                r_points[ite].accelerations = l_points[ite].accelerations + r_points[ite].accelerations
                r_points[ite].effort = l_points[ite].effort + r_points[ite].effort
                if (l_points[ite].time_from_start.secs > r_points[ite].time_from_start.secs) or (
                    (l_points[ite].time_from_start.secs == r_points[ite].time_from_start.secs)
                    and (l_points[ite].time_from_start.nsecs == r_points[ite].time_from_start.nsecs)
                ):
                    r_points[ite].time_from_start = l_points[ite].time_from_start
            for eti in range(len(l_points), len(r_points)):
                r_points[eti].positions = l_points[len(l_points) - 1].positions + r_points[eti].positions
                r_points[eti].velocities = l_points[len(l_points) - 1].velocities + r_points[eti].velocities
                r_points[eti].accelerations = l_points[len(l_points) - 1].accelerations + r_points[eti].accelerations
                r_points[eti].effort = l_points[len(l_points) - 1].effort + r_points[eti].effort
            return r_plan
        else:
            l_plan.joint_trajectory.joint_names = (
                l_plan.joint_trajectory.joint_names + r_plan.joint_trajectory.joint_names
            )
            for ite in range(0, len(r_points)):
                l_points[ite].positions = l_points[ite].positions + r_points[ite].positions
                l_points[ite].velocities = l_points[ite].velocities + r_points[ite].velocities
                l_points[ite].accelerations = l_points[ite].accelerations + r_points[ite].accelerations
                l_points[ite].effort = l_points[ite].effort + r_points[ite].effort
                if (r_points[ite].time_from_start.secs > l_points[ite].time_from_start.secs) or (
                    (r_points[ite].time_from_start.secs == l_points[ite].time_from_start.secs)
                    and (r_points[ite].time_from_start.nsecs == l_points[ite].time_from_start.nsecs)
                ):
                    l_points[ite].time_from_start = r_points[ite].time_from_start
            for eti in range(len(r_points), len(l_points)):
                l_points[eti].positions = l_points[eti].positions + r_points[len(r_points) - 1].positions
                l_points[eti].velocities = l_points[eti].velocities + r_points[len(r_points) - 1].velocities
                l_points[eti].accelerations = l_points[eti].accelerations + r_points[len(r_points) - 1].accelerations
                l_points[eti].effort = l_points[eti].effort + r_points[len(r_points) - 1].effort
            return l_plan

    def move_xyz_execute(self, points, pick, code, scale, perc):
        self.wait_to_move()
        l_plan = self.move_xyz_plan(points[0], points[1], points[2], code, "left", perc)
        r_plan = self.move_xyz_plan(points[3], points[4], points[5], code, "right", perc)

        if l_plan and r_plan:
            self.change_velocity(l_plan, scale)
            self.change_velocity(r_plan, scale)

        b_plan = None
        if l_plan and r_plan:
            b_plan = self.move_xyz_concatenate(l_plan, r_plan)
        if b_plan:
            try:
                pet = self.execute_kp(b_plan, True)
                if (pet.error_code.val == 1 or pet.error_code.val == -4) and pick:
                    self.gripper_manager(self.lgripper)
                    self.gripper_manager(self.rgripper)
                return True
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                return False
        else:
            return False
        rospy.sleep(1)

    ################
    ### Grippers ###
    ################

    def gripper_state_update(self, side, value):
        if side == "right":
            self.rgripper_state = value
        elif side == "left":
            self.lgripper_state = value

    def gripper_manager(self, side):
        if self.choose_gripper_state(side):
            self.choose_gripper(side).open()
            self.gripper_state_update(side, False)
        else:
            self.choose_gripper(side).close()
            self.gripper_state_update(side, True)

    def gripper_instate_open(self, side):
        if self.choose_gripper_instate(side).position > 90.0:
            return True
        else:
            return False

    def gripper_instate_close(self, side):
        if self.choose_gripper_instate(side).position < 90.0:
            return True
        else:
            return False

    def gripper_is_grip(self, side):
        if self.choose_gripper_instate(side).gripping > 0.0:
            return True
        else:
            return False

    ##################################
    ### Experiment flow management ###
    ##################################

    # Waits for the service to be available, as in, there is no movement in progress
    def wait_to_move(self):
        self.execute_kp.wait_for_service()

    ################
    ### Updaters ###
    ################

    # Updates the current state of the end effectors and joints
    def update_data(self):
        try:
            self.larm_state = self.get_es(String("left"))
            self.rarm_state = self.get_es(String("right"))
            self.joint_states = self.get_js(Bool(True))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    # Updates the initial state of the end effectors and joints
    def update_init_data(self):
        try:
            self.larm_init_state = self.get_es(String("left"))
            self.rarm_init_state = self.get_es(String("right"))
            self.joint_init_states = self.get_js(Bool(True))
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    ###############
    ### Helpers ###
    ###############

    # Creates a header for a ROS msg/srv
    def create_header(self, frame_id):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base"
        return header

    ########################
    ###   Joints bounds   ##
    ########################

    def check_arm_bounds(self, angles):
        bounds = [[-1.7, 1.7], [-2.1, 1.0], [-3.05, 3.05], [-0.0, 2.6], [-3.05, 3.05], [-1.5, 2.0], [-3.05, 3.05]]
        for it in range(0, len(angles)):
            if angles[it] < bounds[it][0]:
                angles[it] = bounds[it][0]
            elif angles[it] > bounds[it][1]:
                angles[it] = bounds[it][1]

        return angles
