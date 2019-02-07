"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import * #noqa
import os.path
import math
import threading
from collections import OrderedDict
import yaml
import numpy
from matplotlib import pyplot as plt
# ROS
import rospy
from std_msgs.msg import Bool, String
from mdb_common.msg import GoalMsg, GoalOkMsg, GoalActivationMsg, ObjectListMsg
from mdb_common.srv import ExecPolicy, RefreshWorld, BaxMC, GetSenseMotiv, BaxChange
# MOTIVEN
from mdb_motiven.candidate_state_evaluator import CandidateStateEvaluator
from mdb_motiven.correlations_manager import CorrelationsManager
from mdb_motiven.episode import Episode
from mdb_motiven.episodic_buffer import EpisodicBuffer
from mdb_motiven.goal_manager import GoalManager
from mdb_motiven.traces_buffer import TracesBuffer
from mdb_motiven.traces_memory import TracesMemory
#LTM
from mdb_simulator.simulator import LTMSim

#####MOTIVEN HIGH LEVEL
class MeanReward(object):
    def __init__(self):
        self.mean_value = 0
        self.n_values = 0
#####

class MOTIVEN(object):
    def __init__(self):
        # Object initialization
        self.memory_vf = TracesBuffer()
        self.memory_vf.setMaxSize(50) # 100
        self.traces_memory_vf = TracesMemory()
        # self.StateSpace = StateSpace()
        # self.simulator = Sim()
        self.traces_buffer = TracesBuffer()
        self.traces_buffer.setMaxSize(20)  # 15 50
        self.intrinsic_memory = EpisodicBuffer()
        self.intrinsic_memory.setMaxSize(20)  # 50
        self.episode = Episode()
        self.correlations_manager = CorrelationsManager()
        self.cse = CandidateStateEvaluator()
        self.goal_manager = GoalManager()
        self.stop = 0
        self.iterations = 0
        self.it_reward = 0  # Number of iteraration before obtaining reward
        self.it_blind = 0  # Number of iterations the Intrinsic blind motivation is active
        self.n_execution = 1  # Number of the execution of the experiment
        self.active_mot = 'Int'  # Variable to control the active motivation: Intrinsic ('Int') or Extrinsic ('Ext')
        self.active_mot_old = 'Int'
        self.active_corr = 0  # Variable to control the active correlation. It contains its index
        self.corr_sensor = 0  # 1 - Sensor 1, 2 - Sensor 2, ... n- sensor n, 0 - no hay correlacion
        self.corr_type = ''  # 'pos' - Positive correlation, 'neg' - Negative correlation, '' - no correlation
        self.iter_min = 0  # Minimum number of iterations to consider possible an antitrace
        rospy.loginfo('Iteration  ActiveMotivation  ActiveCorrelation  CorrelatedSensor  CorrelationType  Episode')
        self.use_motiv_manager = 0
        # Graph matrixes
        self.graph1 = []
        self.graph2 = []
        self.graph_exec = []
        self.graphx = []
        self.reward = 0
        self.reset = True # May be this is not really necessary and we can use self.reward. Confirm.
        self.ball_gripper = False
        self.ball_robobo = False
        # Set minimum distances for the ball to be inside the box or grasped by a robot
        self.min_dist_box = 100  # 150#0.275
        self.min_dist_robot = 120  # 150#0.275
        self.baxter_gripper_angle = 0.0
        self.robobo_angle = 0.0
        self.ltm = False  # Variable to decide if MotivEn is executed alone or integrated with the LTM
        # Sensors LTM
        self.sensors_list = [
            'ball_dist',
            'ball_ang',
            'ball_size',
            'box1_dist',
            'box1_ang',
            'box1_size',
            'box2_dist',
            'box2_ang',
            'box2_size',
            'ball_in_left_hand',
            'ball_in_right_hand',
            'happy_human']
        self.perceptions = OrderedDict((sensor, None) for sensor in self.sensors_list)
        self.sens_t = OrderedDict((sensor, None) for sensor in self.sensors_list)
        self.sens_t1 = OrderedDict((sensor, None) for sensor in self.sensors_list)
        self.sensor_lock = threading.Lock()
        self.run_executed_policy_cb = threading.Event()
        self.run_sensor_cb = threading.Event()
        self.file_name = None
        # Policies LTM
        self.n_policies_exec = 0
        self.max_policies_exec = 8
        # Write here the goal list
        self.goals_list = [
            # 'intrinsic',
            'object_held',
            'object_held_with_two_hands',
            'changed_hands',
            'frontal_object',
            'object_in_close_box',
            'object_with_robot',
            'object_in_far_box',
            'approximated_object',
            'clean_area']
        for goal in self.goals_list:
            self.goal_manager.newGoal(goal)
        # Change this for a low-level goal experiment
        self.active_goal = 'clean_area'
        # MOTIVEN HIGH LEVEL
        self.motiven_high_level = True
        if self.motiven_high_level:
            self.reward_vector = TracesMemory()
            self.traces_buffer2 = TracesBuffer()
            self.traces_buffer2.setMaxSize(7)#4
            self.episode2 = Episode()
            self.state_t = 'Unnamed'
            self.state_t1 = 'Unnamed'
            goal_states = [
                'object_held',
                'object_held_with_two_hands',
                'changed_hands',
                'frontal_object',
                'object_in_close_box',
                'object_with_robot',
                'object_in_far_box',
                'approximated_object',
                'clean_area',
                'Unnamed']
            self.reward_dict = dict.fromkeys(goal_states)
            for key in self.reward_dict:
                self.reward_dict[key] = MeanReward()
        # ROS stuff
        self.motivation_pb = None
        self.goal_topic_pb = None
        self.goal_activation_topic_pb = None
        self.goal_ok_topic_pb = None
        self.refresh_world_srv = None
        self.baxter_mov_srv = None
        self.baxter_sa_srv = None
        self.baxter_policy_srv = None
        self.get_sens_srv = None
        self.robobo_mov_srv = None
        self.robobo_pick_srv = None
        self.robobo_drop_srv = None
        self.robobo_mov_back_srv = None

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["sensor_lock"]
        del state["run_executed_policy_cb"]
        del state["run_sensor_cb"]
        del state["motivation_pb"]
        del state["goal_topic_pb"]
        del state["goal_activation_topic_pb"]
        del state["goal_ok_topic_pb"]
        del state["refresh_world_srv"]
        del state["baxter_mov_srv"]
        del state["baxter_sa_srv"]
        del state["baxter_policy_srv"]
        del state["get_sens_srv"]
        del state["robobo_mov_srv"]
        del state["robobo_pick_srv"]
        del state["robobo_drop_srv"]
        del state["robobo_mov_back_srv"]
        return state

    def init_ros(self, log_level):
        rospy.init_node('motiven', log_level=getattr(rospy, log_level))
        # ROS publishers
        self.motivation_pb = rospy.Publisher("/mdb/motivation/active_sur/", String, queue_size=1)
        self.goal_topic_pb = rospy.Publisher("/mdb/motiven/goal", GoalMsg, latch=True, queue_size=0)
        self.goal_activation_topic_pb = rospy.Publisher(
            "/mdb/motiven/goal_activation",
            GoalActivationMsg,
            latch=True,
            queue_size=0)
        self.goal_ok_topic_pb = rospy.Publisher("/mdb/motiven/goal_ok", GoalOkMsg, latch=True, queue_size=0)
        if self.ltm:
            while self.goal_topic_pb.get_num_connections() == 0:
                pass
        # Publish Goal creation
        for goal in self.goal_manager.goals:
            self.goal_topic_pb.publish(
                command='new',
                id=goal.goal_id,
                newdata_topic='',
                execute_service='',
                get_service='',
                class_name='',
                language='python')
        # ROS services
        self.refresh_world_srv = rospy.ServiceProxy('/mdb/baxter/refresh_world', RefreshWorld)
        self.baxter_mov_srv = rospy.ServiceProxy('/baxter_cart_mov', BaxMC)
        self.baxter_sa_srv = rospy.ServiceProxy('/baxter_sa', BaxChange)
        self.baxter_policy_srv = rospy.ServiceProxy('/mdb/baxter/exec_policy', ExecPolicy)
        self.get_sens_srv = rospy.ServiceProxy('/mdb/baxter/sensors', GetSenseMotiv)
        self.robobo_mov_srv = rospy.ServiceProxy('/robobo_mv', BaxMC)
        self.robobo_pick_srv = rospy.ServiceProxy('/robobo_pick', BaxChange)
        self.robobo_drop_srv = rospy.ServiceProxy('/robobo_drop', BaxChange)
        self.robobo_mov_back_srv = rospy.ServiceProxy('robobo_move_backwards', BaxChange)
        # ROS subscribers for LTM integration
        rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_cb)
        rospy.Subscriber("/mdb/baxter/sensor/cylinders", ObjectListMsg, self.sensor_cb, 'cylinders')
        rospy.Subscriber("/mdb/baxter/sensor/boxes", ObjectListMsg, self.sensor_cb, 'boxes')
        rospy.Subscriber("/mdb/baxter/sensor/ball_in_left_hand", Bool, self.sensor_cb, 'ball_in_left_hand')
        rospy.Subscriber("/mdb/baxter/sensor/ball_in_right_hand", Bool, self.sensor_cb, 'ball_in_right_hand')
        rospy.Subscriber("/mdb/baxter/sensor/happy_human", Bool, self.sensor_cb, 'happy_human')
        # It is difficult to guarantee that a control message is processed before new sensor values :-/
        # It is needed to change this for another mechanism... a service asked every time,
        # a new "reset" sensor, whatever...
        # rospy.Subscriber("/mdb/baxter/control", ControlMsg, self.baxter_control_cb)

    @classmethod
    def load_memory_dump(cls, file_name):
        """Load a previous MOTIVEN memory dump from a file."""
        # motiven = yaml.load(open(file_name, "r"), Loader=yaml.CLoader)
        motiven = yaml.load(open(file_name, "r"))
        motiven.sensor_lock = threading.Lock()
        motiven.run_executed_policy_cb = threading.Event()
        motiven.run_sensor_cb = threading.Event()
        motiven.reset = True
        return motiven

    @classmethod
    def restore(cls, file_name):
        """Return a new MOTIVEN object. It loads a MOTIVEN memory dump if it exists."""
        if file_name is not None:
            if os.path.isfile(file_name):
                print("Loading a previous MOTIVEN memory dump from " + file_name + "...")
                motiven = cls.load_memory_dump(file_name)
            else:
                print(file_name + " does not exist, the name will be used to store a new LTM...")
                motiven = MOTIVEN()
        else:
            file_name = "motiven_dump"
            print("Using " + file_name + " to store a new LTM dump...")
            motiven = MOTIVEN()
        motiven.file_name = file_name
        return motiven

    def baxter_control_cb(self, data):
        """Restart necessary things when the experiment is reset."""
        self.reinitialize_memories()
        self.use_motiv_manager = 1
        rospy.loginfo('No reward. Restart scenario.')
        self.it_reward = 0
        self.it_blind = 0
        self.n_execution += 1
        self.graph_exec = []
        self.memory_vf.removeAll()
        self.perceptions = OrderedDict((sensor, None) for sensor in self.sensors_list)  # Esto no tengo claro si es necesario
        self.sens_t = self.sens_t1  # y yo esto tampoco :-)
        # Policies LTM
        self.n_policies_exec = 0
        # Choose active goal
        self.select_goal()

    def sensor_cb(self, sens_value, sensor_id):
        with self.sensor_lock:
            rospy.logdebug('Thread name: ' + threading.currentThread().getName())
            if all(perception is None for perception in self.perceptions.values()):
                if self.reset:
                    rospy.loginfo('MOTIVEN STAGE 0: begin reading sensors after reset')
                else:
                    rospy.loginfo('MOTIVEN STAGE 2: begin reading sensors after policy execution')
            if sensor_id == "cylinders":
                self.perceptions['ball_dist'] = sens_value.data[0].distance
                self.perceptions['ball_ang'] = sens_value.data[0].angle
                self.perceptions['ball_size'] = sens_value.data[0].diameter
            elif sensor_id == "boxes":
                self.perceptions['box1_dist'] = sens_value.data[0].distance
                self.perceptions['box1_ang'] = sens_value.data[0].angle
                self.perceptions['box1_size'] = sens_value.data[0].diameter
                self.perceptions['box2_dist'] = sens_value.data[1].distance
                self.perceptions['box2_ang'] = sens_value.data[1].angle
                self.perceptions['box2_size'] = sens_value.data[1].diameter
            else:
                self.perceptions[sensor_id] = sens_value.data
            rospy.logdebug('Reading ' + sensor_id + ' = ' + str(sens_value.data))
            # This is to be sure that we don't use perceptions UNTIL we have receive ALL OF THEM.
            if not None in self.perceptions.values():
                self.sens_t = self.sens_t1
                self.sens_t1 = self.perceptions
                if self.motiven_high_level:
                    self.state_t = self.state_t1
                    self.state_t1 = self.get_final_state(self.sens_t1, self.sens_t)
                self.perceptions = OrderedDict((sensor, None) for sensor in self.sensors_list)
                if self.reset:
                    rospy.loginfo('MOTIVEN STAGE 0: end reading sensors after reset')
                    self.reset = False
                else:
                    rospy.loginfo('MOTIVEN STAGE 2: end reading sensors after policy execution')
                    self.run_executed_policy_cb.set()
                    # Now, policy callback will be executed and self.reset could change there, so be ware
                    # and don't combine 'if's!
                    self.run_sensor_cb.wait()
                    self.run_sensor_cb.clear()
                if not self.reset:
                    rospy.loginfo('MOTIVEN STAGE 1: begin calculating and publishing goal activations')
                    self.select_goal()
                    # Calculate the goal relevance for the current state
                    self.motivation_manager_ltm()
                    # Set goal activations in Goal Manager and publish in topics for LTM
                    self.publish_goal_activations()
                    rospy.loginfo('MOTIVEN STAGE 1: end calculating and publishing goal activations')

    def executed_policy_cb(self, policy_id):
        """Second part of integration with LTM (a policiy has been executed)."""
        # This is to be sure that we have the new perceptions after the policy has been executed.
        self.run_executed_policy_cb.wait()
        self.run_executed_policy_cb.clear()
        rospy.loginfo('MOTIVEN STAGE 3: begin calculating and publishing goal success values')
        # Check if a new correlation is needed or established
        self.correlations_manager.newSUR(self.active_goal)
        if self.correlations_manager.correlations[self.active_corr].i_reward_assigned == 0:
            self.correlations_manager.assignRewardAssigner(
                self.active_corr,
                list(self.sens_t1.values()),
                self.active_goal)
        # Como conozco el reward???
        if self.sens_t1['happy_human']:
            self.reward = 1
            self.reset = True
            rospy.logdebug('Reward obtained for ' + self.active_goal)
            # self.reward is set to 0 in self.memory_manager_ltm()
            we_are_happy = True
        else:
            self.reward = 0
            we_are_happy = False
        self.episode.setEpisode(list(self.sens_t.values()), policy_id.data, list(self.sens_t1.values()), self.reward)
        #####
        if self.motiven_high_level:
            self.episode2.setEpisode(self.state_t, policy_id.data, self.state_t1, self.reward)
        #####
        # MEMORY MANAGER: Save episode in the pertinent memories and Traces, weak traces and antitraces
        self.memory_manager_ltm()
        # Decide if the agent is improving its behaviour and publish it in topic for LTM
        # Criterion:
        # goal_ok indicates if we are on the right track, that is, if a 'utility' improvement is perceived
        # when changing from the previous state to the current one.
        for goal in self.goal_manager.goals:
            if self.motiven_high_level:
                if goal.goal_id == self.state_t1:
                    # if self.active_goal == self.state_t1:
                    if we_are_happy:
                        goal_ok = 1.0
                    else:
                        goal_ok = 0.5
                else:
                    goal_ok = 0.0
            else:
                goal_ok = 1.0 if self.episode.getReward() else self.is_improving_behavior()
            self.goal_ok_topic_pb.publish(id=goal.goal_id, ok=goal_ok)
            rospy.logdebug('Publishing goal success for ' + goal.goal_id + ' = ' + str(goal_ok))
        # Pongo las percepciones en None para asi controlar cuando debe empezar el nuevo ciclo, es decir,
        # cuando se hayan leido todas las percepciones
        self.iter_min += 1
        self.iterations += 1
        self.it_reward += 1
        # self.stop_condition()
        self.episode.cleanEpisode()
        #####
        if self.motiven_high_level:
            self.episode2.cleanEpisode()
        #####
        file_name = self.file_name + "_" + str(self.iterations) + ".yaml"
        # We will switch to the same plugins system that LTM is already using. Meanwhile...
        if not (self.iterations % 100):
            # yaml.dump(self, open(file_name, "w"), Dumper=yaml.CDumper)
            yaml.dump(self, open(file_name, "w"))
        rospy.loginfo('Iteration: ' + str(self.iterations))
        rospy.loginfo('MOTIVEN STAGE 3: end calculating and publishing goal success values')
        self.run_sensor_cb.set()

    def is_improving_behavior(self, um_type='SUR'):
        """MOTIVEN decides if the agent is improving its behavior, that is, if if follows the active UM correctly."""
        # For now, only SURs are considered as possible utility models
        if um_type == 'SUR':
            if self.active_mot == 'Ext':
                if self.correlations_manager.correlations[self.active_corr].established:
                    goal_ok_response = 0.5
                else:
                    goal_ok_response = 0.0
            else:  # self.active_mot == 'Int'
                goal_ok_response = 0.0
        else:  # um_type == 'VF'
            goal_ok_response = 0.0
        return goal_ok_response

    def publish_goal_activations(self):
        """Set goal activations in Goal Manager and publish them in the corresponding ROS topic."""
        if self.motiven_high_level:
            for goal in self.goal_manager.goals:
                if goal.goal_id == self.active_goal:
                    goal.activation = self.reward_dict[goal.goal_id].mean_value
                elif self.reward_dict[goal.goal_id].mean_value > self.reward_dict[self.state_t1].mean_value:
                    goal.activation = self.reward_dict[goal.goal_id].mean_value
                else:
                    goal.activation = 0.0
        else:
            if self.active_mot == 'Int':
                for idx, goal in enumerate(self.goal_manager.goals):
                    if idx == 0:
                        goal.activation = 1.0
                    else:
                        goal.activation = 0.0
            else:
                for goal in self.goal_manager.goals:
                    if goal.goal_id == self.active_goal:
                        goal.activation = 1.0
                    else:
                        goal.activation = 0.0
        for goal in self.goal_manager.goals:
            rospy.logdebug('Publishing goal activation for ' + goal.goal_id + ' = ' + str(goal.activation))
            self.goal_activation_topic_pb.publish(id=goal.goal_id, activation=goal.activation)

    def select_goal(self):
        """Select the current goal (ad-hoc at the moment)."""
        if self.iterations > 0:
            self.active_goal = 'clean_area'#self.goals_list[1]

    def run_standalone(self):
        self.stop = 0
        self.iterations = 0
        self.refresh_world_srv(String("motiven"), Bool(False))  # Restart scenario
        candidate_state = None

        while not self.stop and not rospy.is_shutdown():

            ## SENSORIZATION in t(distances, action and motivation)
            sensorization = self.get_sens_srv(Bool(True))
            self.episode.setSensorialStateT((
                sensorization.obj_rob_dist.data * 1000,
                sensorization.obj_grip_dist.data * 1000,
                sensorization.obj_box_dist.data * 1000))

            if self.iterations > 0:
                print('*************************')
                print('Predicted state: ', candidate_state)

            self.motivation_pb.publish(String(
                'Motivation: ' + str(self.active_mot) +
                '\nActive SUR: ' + str(self.active_corr + 1) +
                ', Correlated Sensor: ' + str(self.corr_sensor) +
                ' ' + str(self.corr_type)))

            # Apply Action
            if self.iterations == 0:
                self.episode.setAction(10)
            else:
                self.episode.setAction((action.robobo_action.data, action.baxter_action.data))
            # Action (Baxter + Robobo)
            movement_req = BaxMCRequest()
            movement_req.dest.const_dist.data = 0.05
            if self.iterations == 0:
                movement_req.dest.angle.data = 10 * (math.pi / 180.0)
                movement_req.valid.data = True
            else:
                movement_req.dest.angle.data = action.baxter_action.data * (math.pi / 180.0)  # Conversion to rad
                movement_req.valid = action.baxter_valid
            movement_req.dest.height.data = 0.0
            movement_req.orientation.data = "current"
            movement_req.arm.data = "right"
            movement_req.scale.data = 1.0
            try:
                self.baxter_mov_srv(movement_req)  # self.simulator.baxter_larm_action(action)
            except rospy.ServiceException as e:
                rospy.logerr("Movement service call failed: {0}".format(e))
            ######
            movement_req = BaxMCRequest()
            movement_req.dest.const_dist.data = 0.05
            if self.iterations == 0:
                movement_req.dest.angle.data = 10 * (math.pi / 180.0)
                movement_req.valid.data = True
            else:
                movement_req.dest.angle.data = action.robobo_action.data * (math.pi / 180.0)  # Conversion to rad
                movement_req.valid = action.robobo_valid
            try:
                self.robobo_mov_srv(movement_req)
            except rospy.ServiceException as e:
                rospy.logerr("Movement service call failed: {0}".format(e))
            self.world_rules()

            # SENSORIZATION in t+1 (distances and reward)
            sensorization = self.get_sens_srv(Bool(True))
            if self.reward:
                self.episode.setSensorialStateT1((sensorization.obj_rob_dist.data * 1000, 0.0, 0.0))
            else:
                self.episode.setSensorialStateT1((
                    sensorization.obj_rob_dist.data * 1000,
                    sensorization.obj_grip_dist.data * 1000,
                    sensorization.obj_box_dist.data * 1000))
            self.episode.setReward(self.reward)

            print('Real state: ', self.episode.getSensorialStateT1())
            print('*************************')

            ###########################
            if self.iterations > 0:
                # self.write_logs()
                self.debug_print()
                self.save_graphs()
            ###########################

            # Check if a new correlation is needed or established
            self.correlations_manager.newSUR(self.active_goal)
            if self.correlations_manager.correlations[self.active_corr].i_reward_assigned == 0:
                self.correlations_manager.assignRewardAssigner(
                    self.active_corr,
                    self.episode.getSensorialStateT1(),
                    self.active_goal)

            ### Memory Manager: Save episode in the pertinent memories and Traces, weak traces and antitraces
            self.memory_manager()

            ### Motiv. Manager
            self.motivation_manager()
            # CANDIDATE STATE EVALUATOR and ACTION CHOOSER
            sensorization = self.get_sens_srv(Bool(True))
            self.baxter_gripper_angle = sensorization.grip_angle.data * 180.0 / math.pi
            self.robobo_angle = sensorization.rob_angle.data * 180.0 / math.pi
            # Generate new action
            sim_data = (
                (sensorization.grip_x.data * 1000, sensorization.grip_y.data * 1000),
                self.baxter_gripper_angle,
                (sensorization.rob_x.data * 1000, sensorization.rob_y.data * 1000),
                self.robobo_angle,
                (sensorization.obj_x.data * 1000, sensorization.obj_y.data * 1000),
                (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000),
                (sensorization.box_x.data * 1000, sensorization.box_y.data * 1000),
                movement_req.dest.const_dist.data * 1000)
            action = self.cse.getAction(
                self.active_mot, sim_data,
                tuple((
                    sensorization.obj_rob_dist.data * 1000,
                    sensorization.obj_grip_dist.data * 1000,
                    sensorization.obj_box_dist.data * 1000)),
                self.corr_sensor, self.corr_type,
                self.intrinsic_memory.getContents())
            # Predicted state
            candidate_state = self.cse.ForwModel.predictedState(action, sim_data)
            # print "predicted State en i =: ", self.iterations+1, candidate_state

            # Others
            # self.write_logs()
            # self.debug_print()
            self.iter_min += 1
            self.iterations += 1
            self.it_reward += 1
            self.stop_condition()
            self.episode.cleanEpisode()
        self.plot_graphs()
        plt.pause(0.0001)
        rospy.sleep(15)

    def run(self, log_level='INFO', standalone=True, plot=False):
        """Start the MOTIVEN part of the brain."""
        try:
            self.ltm = not standalone
            self.init_ros(log_level)
            self.cse.init_ros()
            self.publish_goal_activations()
            if not self.ltm:
                self.run_standalone()
            else:
                rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logerr("Exception caught! Or you pressed CTRL+C or something went wrong...")

    def stop_condition(self):
        if self.iterations > 10000:
            self.stop = 1

    def write_logs(self):
        rospy.logdebug(
            '%s  -  %s  -  %s  -  %s  -  %s  -  %s',
            self.iterations,
            self.active_mot,
            self.active_corr,
            self.corr_sensor,
            self.corr_type,
            self.episode.getEpisode())

    def debug_print(self):
        rospy.logdebug('------------------')
        rospy.logdebug('Iteration: ' + str(self.iterations))
        rospy.logdebug('Active correlation: ' +  self.active_corr)
        rospy.logdebug('Active motivation: ' + self.active_mot)
        rospy.logdebug('Correlated sensor: ' + self.corr_sensor + ', ' + self.corr_type)
        rospy.logdebug('Sensorization: ' + str(tuple(self.episode.getSensorialStateT1())))

    def reinitialize_memories(self):
        self.traces_buffer.removeAll()  # Reinitialize traces buffer
        self.iter_min = 0
        self.intrinsic_memory.removeAll()  # Reinitialize intrinsic memory
        self.intrinsic_memory.addEpisode(self.episode.getSensorialStateT1())

    def motivation_manager(self):
        if self.use_motiv_manager:
            sensorization = self.get_sens_srv(Bool(True))
            if self.correlations_manager.correlations[self.active_corr].goal != self.active_goal:  # If the goal changes
                self.active_corr = self.correlations_manager.getActiveCorrelationPrueba(
                    tuple((sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                           sensorization.obj_box_dist.data * 1000)), self.active_goal)
            self.corr_sensor, self.corr_type = self.correlations_manager.getActiveCorrelation(
                tuple((sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                       sensorization.obj_box_dist.data * 1000)), self.active_corr, self.active_goal)
            if self.corr_sensor == 0:
                self.active_mot = 'Int'
            else:
                if self.active_mot == 'Int':
                    self.iter_min = 0
                self.active_mot = 'Ext'

    def motivation_manager_ltm(self):
        self.active_mot_old = self.active_mot
        if self.use_motiv_manager:
            if self.correlations_manager.correlations[self.active_corr].goal != self.active_goal:
                self.active_corr = self.correlations_manager.getActiveCorrelationPrueba(
                    list(self.sens_t1.values()),
                    self.active_goal)
            self.corr_sensor, self.corr_type = self.correlations_manager.getActiveCorrelation(
                tuple(self.sens_t1.values()),
                self.active_corr,
                self.active_goal)
            if self.corr_sensor == 0:
                self.active_mot = 'Int'
            else:
                if self.active_mot == 'Int':
                    self.iter_min = 0
                self.active_mot = 'Ext'

    def memory_manager(self):
        """Save episode in the pertinent memories."""
        self.traces_buffer.addEpisode(self.episode.getEpisode())
        self.intrinsic_memory.addEpisode(self.episode.getSensorialStateT1())
        self.memory_vf.addEpisode(self.episode.getEpisode())
        # Memory Manager (Traces, weak traces and antitraces)
        if self.active_mot == 'Int':
            self.it_blind += 1
            self.use_motiv_manager = 1
            # If there is a reward, realise reward assignment and save trace in Traces Memory
            if self.episode.getReward():
                ###
                if self.correlations_manager.correlations[self.active_corr].i_reward_assigned == 0:
                    self.correlations_manager.assignRewardAssigner(
                        self.active_corr,
                        self.episode.getSensorialStateT1(),
                        self.active_goal,
                        1)
                ###
                self.refresh_world_srv(String("motiven"), Bool(True))  # Restart scenario
                self.reward = 0
                self.ball_gripper = False
                self.ball_robobo = False
                self.correlations_manager.correlations[self.active_corr].correlationEvaluator(
                    self.traces_buffer.getTrace())
                sensorization = self.get_sens_srv(Bool(True))
                self.active_corr = self.correlations_manager.getActiveCorrelationPrueba((
                    sensorization.obj_rob_dist.data * 1000,
                    sensorization.obj_grip_dist.data * 1000,
                    sensorization.obj_box_dist.data * 1000), self.active_goal)
                self.reinitialize_memories()
                rospy.loginfo('Goal reward when Intrinsic Motivation')
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.save_matrix()
                self.traces_memory_vf.addTraces(self.memory_vf.getTraceReward())
                self.memory_vf.removeAll()
            elif self.correlations_manager.getReward(
                    self.active_corr,
                    self.reward,
                    tuple(self.episode.getSensorialStateT1()),
                    self.active_goal):
                self.correlations_manager.correlations[self.active_corr].correlationEvaluator(
                    self.traces_buffer.getTrace())
                # The active correlation is now the correlation that has provided the reward
                self.active_corr = self.correlations_manager.correlations[self.active_corr].i_reward
                self.reinitialize_memories()
                rospy.loginfo('Correlation reward when Intrinsic Motivation')
        elif self.active_mot == 'Ext':
            self.use_motiv_manager = 0
            if self.episode.getReward():  # GOAL MANAGER - Encargado de asignar la recompensa?
                self.refresh_world_srv(String("motiven"), Bool(True))  # Restart scenario
                self.reward = 0
                self.ball_gripper = False
                self.ball_robobo = False
                # Save as trace in TracesMemory of the correlated sensor
                self.correlations_manager.correlations[self.active_corr].addTrace(
                    self.traces_buffer.getTrace(),
                    self.corr_sensor,
                    self.corr_type)
                sensorization = self.get_sens_srv(Bool(True))
                self.active_corr = self.correlations_manager.getActiveCorrelationPrueba((
                    sensorization.obj_rob_dist.data * 1000,
                    sensorization.obj_grip_dist.data * 1000,
                    sensorization.obj_box_dist.data * 1000), self.active_goal)
                self.reinitialize_memories()
                rospy.loginfo('Goal reward when Extrinsic Motivation')
                self.use_motiv_manager = 1
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.save_matrix()
                self.traces_memory_vf.addTraces(self.memory_vf.getTraceReward())
                self.memory_vf.removeAll()
            elif self.correlations_manager.getReward(
                    self.active_corr,
                    self.reward,
                    tuple(self.episode.getSensorialStateT1()),
                    self.active_goal):
                # Save as trace in TracesMemory of the correlated sensor
                self.correlations_manager.correlations[self.active_corr].addTrace(
                    self.traces_buffer.getTrace(),
                    self.corr_sensor,
                    self.corr_type)
                # The active correlation is now the correlation that has provided the reward
                self.active_corr = self.correlations_manager.correlations[self.active_corr].i_reward
                self.reinitialize_memories()
                rospy.loginfo('Correlation reward when Extrinsic Motivation')
                self.use_motiv_manager = 1
            else:
                # Check if the the active correlation is still active
                if self.iter_min > 2:
                    sens_t = self.traces_buffer.getTrace()[-2][self.corr_sensor - 1]
                    sens_t1 = self.traces_buffer.getTrace()[-1][self.corr_sensor - 1]
                    dif = sens_t1 - sens_t
                    if (self.corr_type == 'pos' and dif <= 0) or (self.corr_type == 'neg' and dif >= 0):
                        # Guardo antitraza en el sensor correspondiente y vuelvo a comezar el bucle
                        self.correlations_manager.correlations[self.active_corr].addAntiTrace(
                            self.traces_buffer.getTrace(), self.corr_sensor, self.corr_type)
                        sensorization = self.get_sens_srv(Bool(True))
                        self.active_corr = self.correlations_manager.getActiveCorrelationPrueba((
                            sensorization.obj_rob_dist.data * 1000,
                            sensorization.obj_grip_dist.data * 1000,
                            sensorization.obj_box_dist.data * 1000), self.active_goal)
                        self.reinitialize_memories()
                        rospy.loginfo('Antitrace in sensor %s of type %s', self.corr_sensor, self.corr_type)
                        rospy.loginfo('Sens_t %s, sens_t1 %s, diff %s', sens_t, sens_t1, dif)
                        self.use_motiv_manager = 1
                        print('ANTITRAZA \n')

    def memory_manager_ltm(self):
        # Save episode in the pertinent memories
        self.traces_buffer.addEpisode(self.episode.getEpisode())
        #####
        if self.motiven_high_level:
        #     if self.episode2.getSensorialStateT1() != 'Unnamed':
        #         self.traces_buffer2.addEpisode(self.episode2.getEpisode())
            state = self.episode2.getSensorialStateT1()
            if state != 'Unnamed':
                if state in self.traces_buffer2.getTrace():
                    self.traces_buffer2.getContents().pop(self.traces_buffer2.getTrace().index(state))
                self.traces_buffer2.addEpisode(self.episode2.getEpisode())
        #####
        self.intrinsic_memory.addEpisode(self.episode.getSensorialStateT1())
        self.memory_vf.addEpisode(self.episode.getEpisode())
        # Memory Manager (Traces, weak traces and antitraces)
        if self.active_mot == 'Int':
            self.it_blind += 1
            self.use_motiv_manager = 1
            # If there is a reward, make reward assignment and save trace in Traces Memory
            if self.episode.getReward():
                ###
                if self.correlations_manager.correlations[self.active_corr].i_reward_assigned == 0:
                    self.correlations_manager.assignRewardAssigner(
                        self.active_corr,
                        self.episode.getSensorialStateT1(),
                        self.active_goal,
                        1)
                ###
                self.reward = 0
                self.correlations_manager.correlations[self.active_corr].correlationEvaluator(
                    self.traces_buffer.getTrace())
                self.active_corr = self.correlations_manager.getActiveCorrelationPrueba(
                    list(self.sens_t1.values()),
                    self.active_goal)
                rospy.loginfo('Goal reward when Intrinsic Motivation')
                self.reinitialize_memories()
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.save_matrix()
                self.traces_memory_vf.addTraces(self.memory_vf.getTraceReward())
                self.memory_vf.removeAll()
                #####
                if self.motiven_high_level:
                    self.reward_vector.addTraces(self.traces_buffer2.getTraceReward())
                    self.update_reward_values(self.traces_buffer2.getTraceReward())
                    self.traces_buffer2.removeAll()
                #####
            elif self.correlations_manager.getReward(
                    self.active_corr,
                    self.reward,
                    tuple(self.episode.getSensorialStateT1()),
                    self.active_goal):
                self.correlations_manager.correlations[self.active_corr].correlationEvaluator(
                    self.traces_buffer.getTrace())
                # The active correlation is now the correlation that has provided the reward
                self.active_corr = self.correlations_manager.correlations[self.active_corr].i_reward
                self.reinitialize_memories()
                rospy.loginfo('Correlation reward when Intrinsic Motivation')
        elif self.active_mot == 'Ext':
            self.use_motiv_manager = 0
            if self.episode.getReward():  # GOAL MANAGER - Encargado de asignar la recompensa?
                self.reward = 0
                # Save as trace in TracesMemory of the correlated sensor
                self.correlations_manager.correlations[self.active_corr].addTrace(
                    self.traces_buffer.getTrace(),
                    self.corr_sensor,
                    self.corr_type)
                self.active_corr = self.correlations_manager.getActiveCorrelationPrueba(
                    list(self.sens_t1.values()),
                    self.active_goal)
                rospy.loginfo('Goal reward when Extrinsic Motivation')
                self.reinitialize_memories()
                self.use_motiv_manager = 1
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.save_matrix()
                self.traces_memory_vf.addTraces(self.memory_vf.getTraceReward())
                self.memory_vf.removeAll()
                #####
                if self.motiven_high_level:
                    self.reward_vector.addTraces(self.traces_buffer2.getTraceReward())
                    self.update_reward_values(self.traces_buffer2.getTraceReward())
                    self.traces_buffer2.removeAll()
                #####
            elif self.correlations_manager.getReward(
                    self.active_corr,
                    self.reward,
                    tuple(self.episode.getSensorialStateT1()),
                    self.active_goal):
                # Save as trace in TracesMemory of the correlated sensor
                self.correlations_manager.correlations[self.active_corr].addTrace(
                    self.traces_buffer.getTrace(),
                    self.corr_sensor,
                    self.corr_type)
                # The active correlation is now the correlation that has provided the reward
                self.active_corr = self.correlations_manager.correlations[self.active_corr].i_reward
                self.reinitialize_memories()
                rospy.loginfo('Correlation reward when Extrinsic Motivation')
                self.use_motiv_manager = 1
            else:
                # Check if the the active correlation is still active
                if self.iter_min > 2:  # Aqui debo gestionar lo del contador para las policies usadas por JA
                    sens_t = self.traces_buffer.getTrace()[-2][self.corr_sensor - 1]
                    sens_t1 = self.traces_buffer.getTrace()[-1][self.corr_sensor - 1]
                    dif = sens_t1 - sens_t
                    if (self.corr_type == 'pos' and dif <= 0) or (self.corr_type == 'neg' and dif >= 0):
                        self.n_policies_exec += 1
                        if self.n_policies_exec == self.max_policies_exec:
                            # Guardo antitraza en el sensor correspondiente y vuelvo a comezar el bucle
                            self.correlations_manager.correlations[self.active_corr].addAntiTrace(
                                self.traces_buffer.getTrace(),
                                self.corr_sensor,
                                self.corr_type)
                            self.active_corr = self.correlations_manager.getActiveCorrelationPrueba(
                                list(self.sens_t1.values()),
                                self.active_goal)
                            # self.reinitialize_memories()
                            rospy.loginfo('Antitrace in sensor %s of type %s', self.corr_sensor, self.corr_type)
                            rospy.loginfo('Sens_t %s, sens_t1 %s, diff %s', sens_t, sens_t1, dif)
                            self.use_motiv_manager = 1
                            print('ANTITRAZA\n')
                            self.n_policies_exec = 0
                    else:
                        self.n_policies_exec = 0  # Si sigo bien la SUR reinicio el numero de policies ejecutadas

    def world_rules(self):
        """Return the reward checking if the ball is inside one of the boxes."""
        sens = self.get_sens_srv(Bool(True))
        if sens.obj_box_dist.data * 1000 < self.min_dist_box and self.ball_gripper:  # distance ball-box1
            print('Baxter ACTION Drop')
            self.baxter_policy_srv(String('drop_object'))
            self.reward = 1
        elif sens.obj_grip_dist.data * 1000 < self.min_dist_robot and not self.ball_gripper:
        # and (sens.obj_x.data < 0.8 and sens.obj_y.data < 0.05 and sens.obj_y.data > -0.73):  # dist ball-baxter_larm
            print('\nBAXTER ACTION PICK', sens.obj_grip_dist.data, '\n')
            rospy.loginfo(self.ball_gripper)
            if self.ball_robobo:
                self.robobo_mov_back_srv(Bool(True), Bool(True))
                self.ball_robobo = False
            self.baxter_sa_srv(Bool(True), Bool(True))
            self.baxter_policy_srv(String('grasp_object'))
            self.ball_gripper = True
        elif sens.obj_rob_dist.data * 1000 < self.min_dist_robot and not self.ball_robobo and not self.ball_gripper:
            print('ROBOBO ACTION PICK')
            self.robobo_pick_srv(Bool(True), Bool(True))
            self.ball_robobo = True
            self.ball_gripper = False  # Not needed

    def save_graphs(self):
        # Graph 1 - Iterations to reach the goal vs. Total number of iterations
        self.graph1.append((
            self.iterations,
            self.episode.getReward(),
            self.it_reward,
            self.it_blind,
            len(self.correlations_manager.correlations),
            self.active_mot,
            self.active_corr,
            self.episode.getSensorialStateT1()))
        self.graph2.append((
            self.correlations_manager.correlations[-1].S1_neg.numberOfGoalsWithoutAntiTraces,
            self.correlations_manager.correlations[-1].S1_pos.numberOfGoalsWithoutAntiTraces,
            self.correlations_manager.correlations[-1].S2_neg.numberOfGoalsWithoutAntiTraces,
            self.correlations_manager.correlations[-1].S2_pos.numberOfGoalsWithoutAntiTraces,
            self.correlations_manager.correlations[-1].S3_neg.numberOfGoalsWithoutAntiTraces,
            self.correlations_manager.correlations[-1].S3_pos.numberOfGoalsWithoutAntiTraces))
        # Save executions
        self.graph_exec.append((
            self.iterations,
            self.episode.getReward(),
            self.it_reward,
            self.it_blind,
            len(self.correlations_manager.correlations),
            self.active_mot,
            self.active_corr,
            self.episode.getSensorialStateT1()))

    def save_matrix(self):
        self.graphx.append(self.graph_exec)
        self.graph_exec = []

    def plot_graphs(self):
        # Graph 1
        fig = plt.figure()
        ax = fig.add_subplot(111)
        n_reward = 0
        iter_goal = []  # Number of iteration increase at the same time as goals
        for idx, data in enumerate(self.graph1):
            # for i in range(40000):
            if self.graph1[idx][1]:
                n_reward += 1
                iter_goal.append(self.graph1[idx][0])
                if self.graph1[idx][7][1] == 0.0:  # Distance Baxter-ball=0.0 when reward
                    # plt.plot(self.graph1[i][0], self.graph1[i][2], 'ro', color='red')
                    ax.plot(n_reward, self.graph1[idx][2], 'ro', color='red')
                else:  # The reward is given to the Robobo
                    # plt.plot(self.graph1[i][0], self.graph1[i][2], 'ro', color='blue')
                    ax.plot(n_reward, self.graph1[idx][2], 'ro', color='blue')
            if self.graph1[idx][4] > self.graph1[idx - 1][4]:
                ax.axvline(x=n_reward)
        # for i in self.graph1:
        #     if self.graph1[i][4] > self.graph1[i - 1][4]:
        #         plt.axvline(x=self.graph1[i][0])

        # plt.axes().set_xlabel('Iterations')
        ax.set_xlabel('Number of rewards')
        ax.set_ylabel('Iterations needed to reach the goal')
        ax.grid()
        ax2 = ax.twinx()
        print('range(n_reward)', list(range(n_reward)), 'iter_goal', iter_goal)
        ax2.plot(list(range(n_reward)), iter_goal, marker='.', markersize=1.0, color='green', linewidth=1.0, label='active')
        ax2.set_ylabel('Number of iterations')
        # plt.xlim(0, 40000)
        # plt.show()
        # Simple moving average
        reward_matrix = []
        blind_matrix = []
        itera = []
        for i in self.graph1:
            # for i in range(40000):
            if self.graph1[i][1]:
                reward_matrix.append(self.graph1[i][2])
                blind_matrix.append(min(self.graph1[i][3] / 50, 1))
                itera.append(self.graph1[i][0])
        window = 10
        window_aux = 1
        media = self.calc_sma(reward_matrix, window)
        media_aux = self.calc_sma(reward_matrix[:window - 1], window_aux)
        media_sum = media_aux + media[window - 1:]
        # plt.plot(iter, media_sum, marker='.', color='cyan', linewidth=0.5, label='simple moving average')
        print('media_sum', media_sum)
        ax.plot(list(range(n_reward)), media_sum, marker='.', markersize=2.0, color='cyan', linewidth=2.0,
                label='simple moving average')
        # print media_sum
        # print media_sum[-1]
        # print media_sum[205], media_sum[204], media_sum[206]
        #
        # Graph 2
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for i in self.graph1:
            if self.graph1[i][1]:
                ax.plot(self.graph1[i][0], self.graph1[i][3], 'ro', color='green')
        for i in self.graph1:
            if self.graph1[i][4] > self.graph1[i - 1][4]:
                ax.axvline(x=self.graph1[i][0])
        ax.set_xlabel('Iterations')
        ax.set_ylabel('Iterations the Ib motivation is active')
        ax.grid()
        ax2 = ax.twinx()
        # Simple moving average: 1 = use of Ib, 0 = no use of Ib
        window = 10
        window_aux = 1
        media = self.calc_sma(blind_matrix, window)
        media_aux = self.calc_sma(blind_matrix[:window - 1], window_aux)
        media_sum = media_aux + media[window - 1:]
        ax2.plot(iter, media_sum, marker='.', markersize=1.0, color='orange', linewidth=1.0, label='active')
        ax2.set_ylabel('Use of Ib')
        # plt.xlim(0, 40000)
        plt.show()

    def calc_sma(self, data, smaPeriod):
        j = next(i for i, x in enumerate(data) if x is not None)
        our_range = list(range(len(data)))[j + smaPeriod - 1:]
        empty_list = [None] * (j + smaPeriod - 1)
        sub_result = [numpy.mean(data[i - smaPeriod + 1:i + 1]) for i in our_range]
        return list(empty_list + sub_result)

    @staticmethod
    def object_in_close_box(perceptions):
        """Check if there is an object inside of a box."""
        inside = False
        if not LTMSim.object_too_far(perceptions["box1_dist"], perceptions["box1_ang"]):
            inside = (
                (abs(perceptions["box1_dist"] - perceptions["ball_dist"]) < 0.05) and
                (abs(perceptions["box1_ang"] - perceptions["ball_ang"]) < 0.05)
            )
        if not inside:
            if not LTMSim.object_too_far(perceptions["box2_dist"], perceptions["box2_ang"]):
                inside = (
                    (abs(perceptions["box2_dist"] - perceptions["ball_dist"]) < 0.05) and
                    (abs(perceptions["box2_ang"] - perceptions["ball_ang"]) < 0.05)
                )
        return inside

    @staticmethod
    def object_in_far_box(perceptions):
        """Check if there is an object inside of a box."""
        inside = False
        if LTMSim.object_too_far(perceptions["box1_dist"], perceptions["box1_ang"]):
            inside = (
                (abs(perceptions["box1_dist"] - perceptions["ball_dist"]) < 0.05) and
                (abs(perceptions["box1_ang"] - perceptions["ball_ang"]) < 0.05)
            )
        if not inside:
            if LTMSim.object_too_far(perceptions["box2_dist"], perceptions["box2_ang"]):
                inside = (
                    (abs(perceptions["box2_dist"] - perceptions["ball_dist"]) < 0.05) and
                    (abs(perceptions["box2_ang"] - perceptions["ball_ang"]) < 0.05)
                )
        return inside

    @staticmethod
    def object_with_robot(perceptions):
        """Check if there is an object adjacent to the robot."""
        together = False
        if (not perceptions["ball_in_left_hand"]) and (not perceptions["ball_in_right_hand"]):
            dist_near, ang_near = LTMSim.calculate_closest_position(perceptions["ball_ang"])
            together = (
                (abs(perceptions["ball_dist"] - dist_near) < 0.05) and
                (abs(perceptions["ball_ang"] - ang_near) < 0.05)
            )
        return together

    @staticmethod
    def ball_held(perceptions):
        """Check if the ball is held with one hand."""
        return perceptions['ball_in_left_hand'] or perceptions['ball_in_right_hand']

    @staticmethod
    def ball_held_with_two_hands(perceptions):
        """Check if the ball is held with two hands."""
        return perceptions['ball_in_left_hand'] and perceptions['ball_in_right_hand']

    @staticmethod
    def hand_was_changed(new_perceptions, old_perceptions):
        """Check if a ball and a box are on the same side."""
        return (
            (new_perceptions["ball_in_left_hand"] and (not new_perceptions["ball_in_right_hand"]))
            and ((not old_perceptions["ball_in_left_hand"]) and old_perceptions["ball_in_right_hand"])
        ) or (
            ((not new_perceptions["ball_in_left_hand"]) and new_perceptions["ball_in_right_hand"])
            and (old_perceptions["ball_in_left_hand"] and (not old_perceptions["ball_in_right_hand"]))
        )

    @classmethod
    def get_final_state(cls, perceptions_t1, perceptions_t):
        """Translate perceptions into final states."""
        state = 'Unnamed'
        if (None in list(perceptions_t.values())) or (None in list(perceptions_t1.values())):
            rospy.logdebug('Found None value in Perceptions')
        else:
            # if perceptions_t1['happy_human'] and (not perceptions_t['happy_human']):
            #     state = 'clean_area'
            if cls.object_with_robot(perceptions_t1):
                if not cls.object_with_robot(perceptions_t):
                    state = "object_with_robot"
            elif cls.object_in_close_box(perceptions_t1):
                if not cls.object_in_close_box(perceptions_t):
                    state = "object_in_close_box"
            elif cls.object_in_far_box(perceptions_t1):
                if not cls.object_in_far_box(perceptions_t):
                    state = "object_in_far_box"
            elif cls.ball_held(perceptions_t1):
                if cls.ball_held_with_two_hands(perceptions_t1):
                    if not cls.ball_held_with_two_hands(perceptions_t):
                        state = 'object_held_with_two_hands'
                elif not cls.ball_held(perceptions_t):
                    state = 'object_held'
                elif cls.hand_was_changed(perceptions_t1, perceptions_t):
                    state = 'changed_hands'
            elif not cls.ball_held(perceptions_t):
                if LTMSim.object_pickable_withtwohands(perceptions_t1['ball_dist'], perceptions_t1['ball_ang']):
                    if not LTMSim.object_pickable_withtwohands(perceptions_t['ball_dist'], perceptions_t['ball_ang']):
                        state = 'frontal_object'
                elif (
                    not LTMSim.object_too_far(perceptions_t1['ball_dist'], perceptions_t1['ball_ang']) and
                    LTMSim.object_too_far(perceptions_t['ball_dist'], perceptions_t['ball_ang'])
                ):
                    state = 'approximated_object'
        rospy.logdebug('Perceptions correspond to state => ' + str(state))
        return state

    def update_reward_values(self, Trace):
        """Method to update the reward values according to the new traces.
        For now, the method consists of an arithmetic mean of the rewards that are obtained."""
        if isinstance(Trace, list):
            Trace = (Trace,)  # Convert into tuple of len=1
        for trace in Trace:
            self.reward_dict[trace[0]].mean_value = (self.reward_dict[trace[0]].mean_value * self.reward_dict[trace[0]].n_values + trace[1]) / (self.reward_dict[trace[0]].n_values + 1)
            self.reward_dict[trace[0]].n_values += 1
        rospy.logdebug('Reward vector values: ')
        for key, value in self.reward_dict.items():
            rospy.logdebug('Goal state ' + key + ' , value ' + str(value.mean_value))
