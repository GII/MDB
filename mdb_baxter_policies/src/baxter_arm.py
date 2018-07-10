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

import rospy, sys, copy, moveit_commander, rospkg
from baxter_core_msgs.msg import EndpointState, EndEffectorState, JointCommand
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath, ExecuteKnownTrajectory, GetCartesianPathRequest, GetPositionIKRequest
from mdb_baxter_policies.srv import GetJointsState, GetEndState, GetHeadState
from baxter_interface.gripper import Gripper
from moveit_msgs.msg import OrientationConstraint, Constraints, PositionConstraint, RobotTrajectory
from shape_msgs.msg import SolidPrimitive
from moveit_commander import PlanningSceneInterface

from trajectory_msgs.msg import JointTrajectoryPoint
from baxter_interface.limb import Limb

class baxter_arm():
	def __init__(self):
		self.rospack = rospkg.RosPack()

		self.larm_state = EndpointState()
		self.rarm_state = EndpointState()
		self.joint_states = JointState()
		self.larm_init_state = EndpointState()
		self.rarm_init_state = EndpointState()
		self.joint_init_states = JointState()

		try:
			self.compute_cp = rospy.ServiceProxy('/compute_cartesian_path', GetCartesianPath)
			self.execute_kp = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
			self.get_es = rospy.ServiceProxy('/get_end_state', GetEndState)  
			self.get_js = rospy.ServiceProxy('/get_joints_state', GetJointsState)
			self.get_hs = rospy.ServiceProxy('/get_head_state', GetHeadState)
		except rospy.ServiceException, e:
			print "Service call failed: ", str(e)
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
		self.lgripper_instate_sub = rospy.Subscriber("/robot/end_effector/left_gripper/state", EndEffectorState, self.lgripper_in_cb)
		self.rgripper_instate_sub = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, self.rgripper_in_cb)

		self.left_joint_command_pub = rospy.Publisher("/robot/limb/left/joint_command", JointCommand, queue_size = 1)
		self.right_joint_command_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 1)

		self.left_limb = Limb('left')
		self.right_limb = Limb('right')

	####################
	###   Callbacks  ###
	####################

	def lgripper_in_cb (self, instate):
		self.lgripper_instate = instate;

	def rgripper_in_cb (self, instate):
		self.rgripper_instate = instate;

	####################
	### Dictionaries ###
	####################

	def choose_gripper_instate (self, arg):
		options = {
			'left':self.lgripper_instate,
			'right':self.rgripper_instate,
		}
		return options[arg]

	# Choose between baxter's natural arm joints order and the moveit one.
	def choose_joints_order(self, arg):
		options = {
			'baxter':[2,3,0,1,4,5,6],
			'moveit':[0,1,2,3,4,5,6],
		}
		return options[arg]

	# Choose between baxter's natural arm joints order and the moveit one (joint command).
	def choose_joint_command_order(self, arg):
		options = {
			'baxter':[4,5,6,0,1,2,3],
			'moveit':[4,5,6,2,3,0,1],
		}
		return options[arg]

	def choose_arm_group(self, arg):
		options = {
			'right':self.rarm_group,
			'left':self.larm_group,
			'both':self.both_group,
		}
		return options[arg]

	def choose_arm_state(self, arg):
		options = {
			'right':self.rarm_state,
			'left':self.larm_state,
		}
		return options[arg]

	def choose_arm_init_state(self, arg):
		options = {
			'right':self.rarm_init_state,
			'left':self.larm_init_state, 
		}
		return options[arg]

	def choose_gripper (self, arg):
		options = {
			'right':self.rgripper,
			'left':self.lgripper,
		}
		return options[arg]

	def choose_gripper_state (self, arg):
		options = {
			'right':self.rgripper_state,
			'left':self.lgripper_state,
		}
		return options[arg]

	def choose_joint_command_pub (self, arg):
		options = {
			'right':self.right_joint_command_pub,
			'left':self.left_joint_command_pub,  
		}
		return options[arg]

	def choose_limb (self, arg):
		options = {
			'right':self.right_limb,
			'left':self.left_limb,
		}
		return options[arg]
	
	##########################
	###      Pose Goal     ###
	##########################

	def move_to_pose_goal (self, pose, side, wait, scale):
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
	def move_to_position_goal (self, pos, side, wait, scale):
		self.wait_to_move()
		self.update_data()
		self.choose_arm_group(side).clear_pose_targets()

		pose_target = Pose()
		pose_target.orientation.w = 0.0
		pose_target.orientation.x = 1.0
		pose_target.orientation.y = 0.0 
		pose_target.orientation.z = 0.0
		pose_target.position.x = pos[0]
		pose_target.position.y = pos[1]
		pose_target.position.z = pos[2]

		self.choose_arm_group(side).set_pose_target(pose_target)
		plan = self.choose_arm_group(side).plan()
		self.change_velocity(plan, scale)
		self.execute_kp(plan, wait)

	def move_to_position_goal_both (self, pos, wait, scale):
		self.wait_to_move()
		self.update_data()
		self.choose_arm_group('both').clear_pose_targets()

		l_ori = self.choose_arm_state('left').current_es.pose.orientation
		r_ori = self.choose_arm_state('right').current_es.pose.orientation

		l_pose_target = Pose()
		l_pose_target.orientation = l_ori
		l_pose_target.position.x = pos[0]
		l_pose_target.position.y = pos[1]
		l_pose_target.position.z = pos[2]
		self.choose_arm_group('both').set_pose_target(l_pose_target, 'left_gripper')

		r_pose_target = Pose()
		r_pose_target.orientation = r_ori
		r_pose_target.position.x = pos[3]
		r_pose_target.position.y = pos[4]
		r_pose_target.position.z = pos[5]
		self.choose_arm_group('both').set_pose_target(r_pose_target, 'right_gripper')

		self.choose_arm_group('both').set_goal_tolerance(0.01)
		plan = self.choose_arm_group('both').plan()	
		self.change_velocity(plan, scale)
		try:
			self.execute_kp(plan, wait)
			return True
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc))
			return False

	##########################
	###		 Ori Goal      ###
	##########################
	def move_to_ori_goal (self, ori, side, wait, scale):
		self.wait_to_move()
		self.update_data()
		self.choose_arm_group(side).clear_pose_targets()
		pos = self.choose_arm_state(side).current_es.pose.position

		pose_target = Pose()
		pose_target.orientation.x = ori[0]
		pose_target.orientation.y = ori[1]
		pose_target.orientation.z = ori[2]
		pose_target.orientation.w = ori[3]
		pose_target.position = pos

		self.choose_arm_group(side).set_pose_target(pose_target)
		plan = self.choose_arm_group(side).plan()
		self.change_velocity(plan, scale)
		self.execute_kp(plan, wait)

	def move_to_ori_goal_both (self, ori, wait, scale):
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
		self.choose_arm_group('both').set_pose_target(l_pose_target, 'left_gripper')

		r_pose_target = Pose()
		r_pose_target.position = r_pos
		r_pose_target.orientation.x = ori[4]
		r_pose_target.orientation.y = ori[5]
		r_pose_target.orientation.z = ori[6]
		r_pose_target.orientation.w = ori[7]
		self.choose_arm_group('both').set_pose_target(r_pose_target, 'right_gripper')

		plan = self.choose_arm_group('both').plan()	
		self.change_velocity(plan, scale)
		try:
			self.execute_kp(plan, wait)
			return True
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc))
			return False

	#############################################
	###	Arm joint movement (baxter interface) ###
	#############################################

	def move_joints_interface (self, side, positions, ratio, source):
		arranged_joint_names = self.arrange_joint_command(self.select_joint_names(side), 'moveit')
		arranged_positions = self.arrange_joint_command(positions, source)

		dic = dict(zip(arranged_joint_names, arranged_positions))

		self.choose_limb(side).set_joint_position_speed(ratio)
		rospy.sleep(2)
		
		control_rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.choose_limb(side).set_joint_positions(dic, False)
			control_rate.sleep()

	##########################################
	###	Arm joint movement (joint command) ###
	##########################################

	def arrange_joint_command (self, command, source):
		arranged_command_values = list(range(len(command)))
		order = self.choose_joint_command_order(source)
		for ite in range(0, len(arranged_command_values)):
			arranged_command_values[ite] = command[order[ite]]
		return arranged_command_values

	def move_joints_command (self, mode, command, side, source):
		joint_command = JointCommand()

		joint_command.mode = mode
		joint_command.command = self.arrange_joint_command(command, source)
		joint_command.names = self.arrange_joint_command(self.select_joint_names(side), 'moveit')

		self.choose_joint_command_pub(side).publish(joint_command)

	###################################
	### Arm joint movement (moveit) ###
	###################################

	def arrange_angles (self, angles, side, way):
		group_variable_values = None
		if (side=="both"):
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

	#Returns the joints belonging to one specific arm
	def remove_unnecesary_joints (self, joints, side):
		clean_joint_states = JointState()
		clean_joint_states.header = joints.current_js.header
		for ite in range(0, len(joints.current_js.name)):
			if (side == 'both'):
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

	#Moves the arm to a specific target of joints angles
	def move_joints_directly (self, angles, way, side, wait, scale):
		self.wait_to_move()
		self.choose_arm_group(side).clear_pose_targets()
		group_variable_values = self.arrange_angles(angles, side, way)

		self.choose_arm_group(side).set_joint_value_target(group_variable_values)
		plan = self.choose_arm_group(side).plan()
		#print plan
		self.change_velocity(plan, scale)
		self.execute_kp(plan, wait)

	def move_joints_plan(self, angles, way, side):
		self.wait_to_move()
		self.choose_arm_group(side).clear_pose_targets()
		group_variable_values = self.arrange_angles(angles, side, way)
	
		self.choose_arm_group(side).set_joint_value_target(group_variable_values)
		plan = self.choose_arm_group(side).plan()
		return plan

	def move_joints_merge(self, plan_l):
		for plan_it in range (1, len(plan_l)):
			time_from_start_prev = plan_l[plan_it-1].joint_trajectory.points[len(plan_l[plan_it-1].joint_trajectory.points)-1].time_from_start
			for point_it in range(0, len(plan_l[plan_it].joint_trajectory.points)):
				plan_l[plan_it].joint_trajectory.points[point_it].time_from_start += time_from_start_prev
				plan_l[0].joint_trajectory.points.append(plan_l[plan_it].joint_trajectory.points[point_it])
		return plan_l[0]

	def move_joints_execute(self, plan, wait, scale):
		self.wait_to_move()
		self.change_velocity(plan, scale)
		self.execute_kp(plan, wait)

	#Creates a set of target joints angles based of the initial state of the robot
	def create_group_joints(self, side):
		joints = self.remove_unnecesary_joints(self.joint_init_states, side)
		group_joint_values = self.choose_arm_group(side).get_current_joint_values()
		order = self.choose_joints_order('baxter')
		for ite in range(0, len(group_joint_values)):
			group_joint_values[ite] = joints.position[order[ite]]
		return group_joint_values

	def manage_group_joints(self, side):
		group_joint_values = None
		if (side=="both"):
			group_joint_values = self.create_group_joints("left") + self.create_group_joints("right")
		else:
			group_joint_values = self.create_group_joints(side)
		return group_joint_values

	#Restores the pose of the arm to its initial state
	def restore_arm_pose(self, side):
		self.wait_to_move()
		group_joint_values = self.manage_group_joints(side)
		rospy.sleep(1)
		self.choose_arm_group(side).clear_pose_targets()
		self.choose_arm_group(side).set_start_state_to_current_state()
		self.choose_arm_group(side).set_joint_value_target(group_joint_values)
		plan = self.choose_arm_group(side).plan()
		self.execute_kp(plan, True)

	def select_joint_names (self, side):
		options = {
			'left':['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2'],
			'right':['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'], 
			'both':['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'], 
		}
		return options[side]

	def create_joint_trajectory_point(self, positions, velocities, accelerations, time):
		point = JointTrajectoryPoint()
		point.positions = positions
		point.velocities = velocities
		point.accelerations = accelerations
		#point.effort
		point.time_from_start = rospy.Duration(time)

		return point

	def create_first_joint_trajectory_point(self, side):
		point = JointTrajectoryPoint()
		point.positions = self.manage_group_joints(side)
		point.velocities = [0.0] * len(point.positions)
		point.accelerations = [0.0] * len(point.positions)
		#point.effort
		point.time_from_start = rospy.Duration(0)
		
		return point	

	def create_joint_trajectory (self, positions, velocities, accelerations, side, time):
		robot_trajectory = RobotTrajectory()
		robot_trajectory.joint_trajectory.header = self.create_header('base')
		robot_trajectory.joint_trajectory.joint_names = self.select_joint_names(side)

		robot_trajectory.joint_trajectory.points.append(self.create_first_joint_trajectory_point(side))		
		robot_trajectory.joint_trajectory.points.append(self.create_joint_trajectory_point(positions, velocities, accelerations, time))
	
		return robot_trajectory

	def move_joints_raw_position (self, angles, speed, acceleration, way, side, wait, time):
		self.wait_to_move()
		group_variable_values = self.arrange_angles(angles, side, way)
		plan = self.create_joint_trajectory (group_variable_values, speed, acceleration, side, time)
		self.execute_kp(plan, wait)

	##############################
	### Arm cartesian movement ###
	##############################

	#Creates a pose from a state of the arm
	def create_pose(self, state):
		pos = state.current_es.pose.position
		ori = state.current_es.pose.orientation
		pose_target = Pose(Point(pos.x, pos.y, pos.z), Quaternion(ori.x, ori.y, ori.z, ori.w))
		return pose_target
	
	#Creates a set of waypoints from the current state of the arm to the destination state
	def generate_wp_xyz (self, x, y, z, code, side):
		start_pose = self.create_pose(self.choose_arm_state(side))

		wpose = Pose()
		if (code == 'init'):
			wpose.orientation = self.choose_arm_init_state(side).current_es.pose.orientation
		elif (code == 'current'):
			wpose.orientation = start_pose.orientation
		elif (code == 'random'):
			wpose.orientation.w = 0.0
			wpose.orientation.x = 1.0
			wpose.orientation.y = 0.0
			wpose.orientation.z = 0.0

		wpose.position.x = x
		wpose.position.y = y
		wpose.position.z = z

		waypoints = [] 
		waypoints.append(start_pose)
		waypoints.append(copy.deepcopy(wpose))

		return waypoints

	def select_group_name (self, code):
		options = {
			'left':'left_arm',
			'right':'right_arm',
			'both':'both_arms',
		}
		return options[code]

	#Computes the request for the cartesian moveit service
	def compute_cartesian_req(self, x, y, z, code, side):
		self.update_data()
		arm_joints_state = self.remove_unnecesary_joints(self.joint_states, side)
		waypoints = self.generate_wp_xyz(x, y, z, code, side)

		gcp_req = GetCartesianPathRequest()
		gcp_req.header = self.create_header('base')
		gcp_req.start_state.joint_state = arm_joints_state
		gcp_req.group_name = self.select_group_name(side)
		gcp_req.link_name = str(side)+"_gripper"
		gcp_req.waypoints = waypoints
		gcp_req.max_step = 0.01
		gcp_req.avoid_collisions = True 

		return gcp_req

	#Moves the arm to a specific waypoint in the robot 3d space and opens/closes the gripper if desired
	def move_xyz (self, x, y, z, pick, code, side, scale, perc):
		self.wait_to_move()
		self.update_data()
		gcp_req = self.compute_cartesian_req(x, y, z, code, side)
		try:
			fract = 0.0
			resp = None
			tries = 6
			while fract < perc and tries>0:
				resp = self.compute_cp(gcp_req)
				fract = resp.fraction
				tries -= 1
			if tries == 0:
				return False
			if (resp.error_code.val==1):
				self.change_velocity(resp.solution, scale)
				pet = self.execute_kp(resp.solution, True)
				if (pet.error_code.val==1 or pet.error_code.val==-4) and pick:
					self.gripper_manager(side)
					rospy.sleep(0.5)
				self.update_data()
				return True
	
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc))
			return False

	def move_xyz_plan(self, x, y, z, code, side, perc):
		self.wait_to_move()
		self.update_data()
		gcp_req = self.compute_cartesian_req(x, y, z, code, side)
		try:
			fract = 0.0
			resp = None
			tries = 5
			while fract<perc and tries>0:
				resp = self.compute_cp(gcp_req)
				fract = resp.fraction
				tries -= 1
			if tries == 0:
				return False
			if (resp.error_code.val==1):
				return resp.solution
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc))
			return False

	def move_xyz_concatenate (self, l_plan, r_plan):
		l_points = l_plan.joint_trajectory.points
		r_points = r_plan.joint_trajectory.points
		#Concatenate both solutions into one solution
		if len(r_points) > len(l_points):
			r_plan.joint_trajectory.joint_names = l_plan.joint_trajectory.joint_names + r_plan.joint_trajectory.joint_names
			for ite in range (0, len(l_points)):
				r_points[ite].positions = l_points[ite].positions + r_points[ite].positions
				r_points[ite].velocities = l_points[ite].velocities + r_points[ite].velocities
				r_points[ite].accelerations = l_points[ite].accelerations + r_points[ite].accelerations
				r_points[ite].effort =  l_points[ite].effort + r_points[ite].effort
				if (l_points[ite].time_from_start.secs > r_points[ite].time_from_start.secs) or ((l_points[ite].time_from_start.secs == r_points[ite].time_from_start.secs) and (l_points[ite].time_from_start.nsecs == r_points[ite].time_from_start.nsecs)):
					r_points[ite].time_from_start = l_points[ite].time_from_start				
			for eti in range (len(l_points), len(r_points)):
				r_points[eti].positions = l_points[len(l_points)-1].positions + r_points[eti].positions
				r_points[eti].velocities = l_points[len(l_points)-1].velocities + r_points[eti].velocities
				r_points[eti].accelerations = l_points[len(l_points)-1].accelerations + r_points[eti].accelerations
				r_points[eti].effort =  l_points[len(l_points)-1].effort + r_points[eti].effort	
			return r_plan
		else: 
			l_plan.joint_trajectory.joint_names = l_plan.joint_trajectory.joint_names + r_plan.joint_trajectory.joint_names
			for ite in range (0, len(r_points)):
				l_points[ite].positions = l_points[ite].positions + r_points[ite].positions
				l_points[ite].velocities = l_points[ite].velocities + r_points[ite].velocities
				l_points[ite].accelerations = l_points[ite].accelerations + r_points[ite].accelerations
				l_points[ite].effort =  l_points[ite].effort + r_points[ite].effort
				if (r_points[ite].time_from_start.secs > l_points[ite].time_from_start.secs) or ((r_points[ite].time_from_start.secs == l_points[ite].time_from_start.secs) and (r_points[ite].time_from_start.nsecs == l_points[ite].time_from_start.nsecs)):
					l_points[ite].time_from_start = r_points[ite].time_from_start
			for eti in range(len(r_points), len(l_points)):
				l_points[eti].positions = l_points[eti].positions + r_points[len(r_points)-1].positions
				l_points[eti].velocities = l_points[eti].velocities + r_points[len(r_points)-1].velocities
				l_points[eti].accelerations = l_points[eti].accelerations + r_points[len(r_points)-1].accelerations
				l_points[eti].effort =  l_points[eti].effort + r_points[len(r_points)-1].effort
			return l_plan
		
	def move_xyz_execute(self, points, pick, code, scale, perc):
		self.wait_to_move()
		l_plan = self.move_xyz_plan(points[0], points[1], points[2], code, 'left', perc)
		r_plan = self.move_xyz_plan(points[3], points[4], points[5], code, 'right', perc)

		if (l_plan and r_plan):
			l_plan_c = self.change_velocity(l_plan, scale)
			r_plan_c = self.change_velocity(r_plan, scale)

		print (l_plan != None)
		print (r_plan != None)

		b_plan = None
		if (l_plan and r_plan):
			b_plan = self.move_xyz_concatenate(l_plan, r_plan)		
		if b_plan:
			try:
				pet = self.execute_kp(b_plan, True)
				print pet.error_code.val
				if (pet.error_code.val==1 or pet.error_code.val==-4) and pick:
					self.lgripper_state = self.gripper_manager(self.lgripper, self.lgripper_state)
					self.rgripper_state = self.gripper_manager(self.rgripper, self.rgripper_state)		
				return True
			except rospy.ServiceException as exc:
				print ("Service did not process request: " + str(exc))
				return False
		else:
			return False
		rospy.sleep(1)

	def move_xyz_both(self, points, pick, code, scale, perc):
		self.wait_to_move()
		l_plan = self.move_xyz_plan(points[0], points[1], points[2], code, 'left', perc)
		r_plan = self.move_xyz_plan(points[3], points[4], points[5], code, 'right', perc)

		b_plan = None
		if (l_plan and r_plan):
			b_plan = self.move_xyz_concatenate(l_plan, r_plan)

			b_plan_points = b_plan.joint_trajectory.points
			plan_l = []
			fplan = self.move_joints_plan(list(b_plan_points[0].positions), 'moveit', 'both')
			lplan = self.move_joints_plan(list(b_plan_points[len(b_plan_points)-1].positions), 'moveit', 'both')				
			plan_l.append(fplan)
			plan_l.append(lplan)

			plan = self.move_joints_merge(plan_l)
			print "\n"
			#print plan
			self.move_joints_execute(plan, True, scale)
			'''if pick:
				self.lgripper_state = self.gripper_manager(self.lgripper, self.lgripper_state)
				self.rgripper_state = self.gripper_manager(self.rgripper, self.rgripper_state)'''

	##################
	### Compute IK ###
	##################

	def get_last_state (self, x, y, z, code, side, perc):
		plan = self.move_xyz_plan(x, y, z, code, side, perc)

		tray_p = plan.joint_trajectory.points
		print "last_state: ", tray_p[-1].positions
		#return 
		
	################
	### Grippers ###
	################

	def gripper_state_update(self, side, value):
		if (side == 'right'):
			self.rgripper_state = value		
		elif (side == 'left'):
			self.lgripper_state = value
	
	def gripper_manager(self, side):
		if self.choose_gripper_state(side):
			self.choose_gripper(side).open()
			self.gripper_state_update(side, False)
		else:
			self.choose_gripper(side).close()
			self.gripper_state_update(side, True)

	def gripper_instate_open (self, side):
		print self.choose_gripper_instate(side).position
		if (self.choose_gripper_instate(side).position > 90.0):
			return True
		else:
			return False

	def gripper_instate_close (self, side):
		print self.choose_gripper_instate(side).position
		if (self.choose_gripper_instate(side).position < 90.0):
			return True
		else:
			return False

	def gripper_is_grip(self, side):
		if (self.choose_gripper_instate(side).gripping > 0.0):
			return True
		else:
			return False

	##################################
	### Experiment flow management ###
	##################################

	#Waits for the service to be available, as in, there is no movement in progress
	def wait_to_move(self):
		self.execute_kp.wait_for_service()

	################
	### Updaters ###
	################

	#Updates the current state of the end effectors and joints
	def update_data (self):
		try:
			self.larm_state = self.get_es(String('left'))
			self.rarm_state = self.get_es(String('right'))
			self.joint_states = self.get_js(Bool(True))
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc))

	#Updates the initial state of the end effectors and joints
	def update_init_data (self):
		try:
			self.larm_init_state = self.get_es(String('left'))
			self.rarm_init_state = self.get_es(String('right'))
			self.joint_init_states = self.get_js(Bool(True))
		except rospy.ServiceException as exc:
			print ("Service did not process request: " + str(exc))

	###############
	### Helpers ###
	###############

	#Creates a header for a ROS msg/srv
	def create_header(self, frame_id):
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id='base'
		return header

	########################
	###   Joints bounds   ##
	########################

	def check_arm_bounds (self, angles):
		bounds = [[-1.7,1.7],[-2.1,1.0],[-3.05,3.05],[-0.0,2.6],[-3.05,3.05],[-1.5,2.0],[-3.05,3.05]]

		for it in range(0, len(angles)):
			if angles[it] < bounds[it][0]: angles[it] = bounds[it][0]
			elif angles[it] > bounds[it][1]: angles[it] = bounds[it][1]

		return angles

	########################
	### Path Constraints ###
	########################

	def orientation_constraint (self, tolerance, quatn, weight, side):
		oc = OrientationConstraint()
		oc.absolute_x_axis_tolerance = tolerance[0]
		oc.absolute_y_axis_tolerance = tolerance[1]
		oc.absolute_z_axis_tolerance = tolerance[2]
		oc.orientation.x = quatn[0]
		oc.orientation.y = quatn[1]
		oc.orientation.z = quatn[2]
		oc.orientation.w = quatn[3]
		oc.weight = weight
		oc.header = self.create_header('base')
		oc.link_name = side+'_gripper'

		return oc

	def add_orientation_constraints (self, orientation_constraints, side):
		constraints = Constraints()
		for oc in orientation_constraints:
			constraints.orientation_constraints.append(oc)
		self.choose_arm_group(side).set_path_constraints(constraints)

	def remove_path_constraints(self, side):
		self.choose_arm_group(side).clear_path_constraints()

	def position_constraint_region (self, primitive_type, primitive_data):
		primitive = SolidPrimitive()
		primitive.type = primitive_type
		for d in primitive_data:
			primitive.dimensions.append(d)
		return primitive

	def position_constraint_pose (self, pose_data):
		pose = Pose()
		pose.orientation.w = pose_data[0]
		pose.position.x = pose_data[1]
		pose.position.y = pose_data[2]
		pose.position.z = pose_data[3]
		return pose

	def position_constraints (self, primitives, primitive_poses, weight, side):
		pc = PositionConstraint()
		pc.header = self.create_header('/base')
		pc.link_name = side+'_gripper'
		pc.target_point_offset.x = 1.0
		pc.target_point_offset.y = 1.0
		pc.target_point_offset.z = 1.0
		for prim in primitives:
			pc.constraint_region.primitives.append(prim)
		for prim_pos in primitive_poses:
			pc.constraint_region.primitive_poses.append(prim_pos)
		pc.weight = weight

		return pc

	def add_position_constraints (self, positions_constraints, side):
		constraints = Constraints()
		for pc in positions_constraints:
			constraints.position_constraints.append(pc)
		self.choose_arm_group(side).set_path_constraints(constraints)

	def remove_all_constraints (self, side):
		constraints = Constraints()
		self.choose_arm_group(side).set_path_constraints(constraints)

