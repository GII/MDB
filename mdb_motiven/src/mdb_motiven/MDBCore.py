#!/usr/bin/env python
# coding=utf-8

# from Simulador import *
from Episode import *
from CandidateStateEvaluator import *
from TracesBuffer import *
from CorrelationsManager import *
from StateSpace import *
from GoalManager import *

import logging
import pickle
from matplotlib import colors

# ROS libraries
import rospy

# ROS services
from mdb_baxter_policies.srv import BaxMC, GetSenseMotiv, BaxMCRequest, GetSenseMotivRequest, BaxChange, BaxChangeRequest
from mdb_common.srv import ExecPolicy, RefreshWorld
from std_msgs.msg import Bool, String, Float64


class MDBCore(object):
    def __init__(self):
        rospy.init_node("MDBCore")
        # Object initialization
        self.memoryVF = TracesBuffer()
        self.memoryVF.setMaxSize(100)
        self.TracesMemoryVF = TracesMemory()
        # self.StateSpace = StateSpace()
        # self.simulator = Sim()
        self.tracesBuffer = TracesBuffer()
        self.tracesBuffer.setMaxSize(50)  # 15
        self.intrinsicMemory = EpisodicBuffer()
        self.intrinsicMemory.setMaxSize(20)  # 50

        self.episode = Episode()
        self.correlationsManager = CorrelationsManager()
        self.CSE = CandidateStateEvaluator()
        self.goalManager = GoalManager()

        self.stop = 0
        self.iterations = 0
        self.it_reward = 0  # Number of iteraration before obtaining reward
        self.it_blind = 0  # Number of iterations the Intrinsic blind motivation is active
        self.n_execution = 1  # Number of the execution of the experiment

        self.activeMot = 'Int'  # Variable to control the active motivation: Intrinsic ('Int') or Extrinsic ('Ext')

        self.activeCorr = 0  # Variable to control the active correlation. It contains its index
        self.corr_sensor = 0  # 1 - Sensor 1, 2 - Sensor 2, ... n- sensor n, 0 - no hay correlacion
        self.corr_type = ''  # 'pos' - Positive correlation, 'neg' - Negative correlation, '' - no correlation

        self.iter_min = 0  # Minimum number of iterations to consider possible an antitrace

        logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG, filename='LogFile.log')
        logging.info('Iteration  ActiveMotivation  ActiveCorrelation  CorrelatedSensor  CorrelationType  Episode')

        self.useMotivManager = 0

        # Graph matrixes
        self.graph1 = []
        self.graph2 = []
        self.graphExec = []
        self.graphx = []

        self.reward = 0
        self.ball_gripper = False
        self.ball_robobo = False

        # Set minimum distances for the ball to be inside the box or grasped by a robot
        self.min_dist_box = 100  # 150#0.275
        self.min_dist_robot = 120  # 150#0.275

        # ROS publishers
        self.motivation_pb = rospy.Publisher("/mdb/motivation/active_sur/", String, queue_size=1)
        self.goal_topic_pb = rospy.Publisher("/mdb/motiven/goal", GoalMsg, queue_size=1)  # Integration LTM
        self.goal_activation_topic_pb = rospy.Publisher("/mdb/motiven/goal_activation", GoalActivationMsg,
                                                        queue_size=1)  # Integration LTM
        self.goal_ok_topic_pb = rospy.Publisher("/mdb/motiven/goal_ok", GoalOkMsg, queue_size=1)  # Integration LTM

        # ROS subscribers # Integration LTM
        rospy.Subscriber("/mdb/ltm/executed_policy", String, self.executed_policy_topic_cb)  # Integration LTM
        rospy.Subscriber("/mdb/baxter/sensor/ball_dist", Float64, self.sensor_cb, 'ball_dist')
        rospy.Subscriber("/mdb/baxter/sensor/ball_ang", Float64, self.sensor_cb, 'ball_ang')
        rospy.Subscriber("/mdb/baxter/sensor/ball_size", Float64, self.sensor_cb, 'ball_size')
        rospy.Subscriber("/mdb/baxter/sensor/box_dist", Float64, self.sensor_cb, 'box_dist')
        rospy.Subscriber("/mdb/baxter/sensor/box_ang", Float64, self.sensor_cb, 'box_ang')
        rospy.Subscriber("/mdb/baxter/sensor/box_size", Float64, self.sensor_cb, 'box_size')
        rospy.Subscriber("/mdb/baxter/sensor/ball_in_left_hand", Bool, self.sensor_cb, 'ball_in_left_hand')
        rospy.Subscriber("/mdb/baxter/sensor/ball_in_right_hand", Bool, self.sensor_cb, 'ball_in_right_hand')
        # rospy.Subscriber("/mdb/baxter/sensor/dist_diff", Float64, self.sensor_cb, 'dist_diff')
        # rospy.Subscriber("/mdb/baxter/sensor/ang_diff", Float64, self.sensor_cb, 'ang_diff')
        rospy.Subscriber("/mdb/baxter/sensor/ball_in_box", Bool, self.sensor_cb, 'ball_in_box')
        rospy.Subscriber("/mdb/baxter/sensor/ball_with_robot", Bool, self.sensor_cb, 'ball_with_robot')

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

        self.baxter_gripper_angle = 0.0
        self.robobo_angle = 0.0

        self.loadDataFile = 1  # Variable to decide if load data from file
        self.LTM = 0  # Variable to decide if MotivEn is executed alone or integrated with the LTM

        # Sensors LTM
        # sensors_list = ['ball_dist', 'ball_ang', 'ball_size', 'box_dist', 'box_ang', 'box_size', 'ball_in_left_hand',
        #                 'ball_in_right_hand', 'dist_diff', 'ang_diff', 'ball_in_box', 'ball_with_robot']
        sensors_list = ['ball_dist', 'ball_ang', 'ball_size', 'box_dist', 'box_ang', 'box_size', 'ball_in_left_hand',
                        'ball_in_right_hand', 'ball_in_box', 'ball_with_robot']
        self.perceptions = dict.fromkeys(sensors_list)

        # Policies LTM
        self.n_policies_exec = 0
        self.max_policies_exec = 8

        # Goals LTM
        # Establezco lista de goals
        self.goals_list = ['Intrinsic', 'goal_ball_in_box', 'goal_ball_in_robot']
        for i in self.goals_list:
            self.goalManager.newGoal(i)
        self.active_goal = self.goals_list[1]
        # Publish Goal creation
        for i in range(len(self.goalManager.goals)):
            self.goal_topic_pb.publish(String('new'), String(self.goalManager.goals[i].goal_id))

    def executed_policy_topic_cb(self, policy_id):
        ############## PARTE 2: PARA CUANDO LEO LA POLICY EJECUTADA
        # Check if a new correlation is needed or established
        self.correlationsManager.newSUR(self.active_goal)
        if self.correlationsManager.correlations[self.activeCorr].i_reward_assigned == 0:
            self.correlationsManager.assignRewardAssigner(self.activeCorr, self.episode.getSensorialStateT1(),
                                                          self.active_goal)
        # Leo sensorizacion de topic y la guardo en el episode
        if self.iterations < 1:
            sens_t = 0
        else:
            sens_t = sens_t1
        sens_t1 = self.get_sensorization_LTM()
        # Como conozco el reward???
        if self.perceptions[self.active_goal]:
            self.reward = 1
        else:
            self.reward = 0
        self.episode.setEpisode(sens_t, policy_id, sens_t1, self.reward)
        # MEMORY MANAGER: Save episode in the pertinent memories and Traces, weak traces and antitraces
        self.MemoryManagerLTM()
        # Decide if the agent is improving its behaviour and publish it in topic for LTM
        for i in range(len(self.goalManager.goals)):
            if self.active_goal == self.goalManager.goals[i].goal_id:
                self.goal_ok_topic_pb.publish(String(self.goalManager.goals[i].goal_id),
                                              Float64(self.isImprovingBehavior()))
            else:
                self.goal_ok_topic_pb.publish(String(self.goalManager.goals[i].goal_id),
                                              Float64(0))
        #############
        # Pongo las percepciones en None para asi controlar cuando debe empezar el nuevo ciclo, es decir,
        # cuando se hayan leido todas las percepciones
        self.perceptions = dict.fromkeys(self.perceptions, None)
        self.iterations += 1
        self.iter_min += 1
        self.iterations += 1
        self.it_reward += 1
        # self.stopCondition()
        self.episode.cleanEpisode()

    def isImprovingBehavior(self, UMtype='SUR'):
        """MOTIVEN decides if the agent is improving its behavior, that is, if if follows the active UM correctly"""
        #For now, only SURs are considered as possible utility models
        if UMtype == 'SUR':
            if self.activeMot == 'Ext':
                sens_t = self.tracesBuffer.getTrace()[-2][self.corr_sensor - 1]
                sens_t1 = self.tracesBuffer.getTrace()[-1][self.corr_sensor - 1]
                dif = sens_t1 - sens_t
                if (self.corr_type == 'pos' and dif <= 0) or (self.corr_type == 'neg' and dif >= 0):
                    goal_ok_response = 0
                else:
                    goal_ok_response = 1
            else:  # self.activeMot == 'Int'
                goal_ok_response = 0
        else:  # UMtype == 'VF'
            goal_ok_response = 0

        return goal_ok_response

    def publishGoalActivations(self):
        """Set goal activations in Goal Manager and publish them in the corresponding ROS topic"""
        if self.activeMot == 'Int':
            for i in range(len(self.goalManager.goals)):
                if i == 0:
                    self.goalManager.goals[i].activation = 1.0
                else:
                    self.goalManager.goals[i].activation = 0.0
        else:  # self.activeMot == 'Ext'
            for i in range(len(self.goalManager.goals)):
                # if i == 0:
                #     self.goalManager.goals[i].activation = 0.0
                # else:
                    if self.goalManager.goals[i].goal_id == self.active_goal:
                        self.goalManager.goals[i].activation = 1.0
                    else:
                        self.goalManager.goals[i].activation = 0.0
        # Publish
        for i in range(len(self.goalManager.goals)):
            self.goal_activation_topic_pb.publish(String(self.goalManager.goals[i].goal_id),
                                                  Float64(self.goalManager.goals[i].activation))

    def sensor_cb(self, sens_value, sensor_id): # Integration LTM
        self.perceptions[sensor_id] = sens_value
        # Necesito algo que me indique que todas las percepciones estan actualizadas
        # Cuando el valor de ninguna sea None
        if not None in self.perceptions.values():
            ############## PARTE 1: PARA CUANDO LEO LAS PERCEPCIONES
            # MOTIVATION MANAGER
            # MOTIVEN calculates the goal relevance for the current state
            self.select_goal()
            self.MotivationManagerLTM()
            # Set goal activations in Goal Manager and publish in topics for LTM
            self.publishGoalActivations()
            #############

    def get_sensorization_LTM(self):  # Integration LTM
        return self.perceptions.values()  # List with the values of the sensors without key

    def select_goal(self):
        """Method ad hoc to select the current goal"""
        if self.iterations > 0:
            self.active_goal = self.goals_list[1]

    def run(self):
        # Load data
        if self.loadDataFile:
            self.loadData()
        self.main()

    def main(self):
        if not self.LTM:
            self.stop = 0
            self.iterations = 0
            self.refresh_world_srv(String("motiven"), Bool(False))  # Restart scenario

            while not self.stop and not rospy.is_shutdown():

                ## SENSORIZATION in t(distances, action and motivation)
                sensorization = self.get_sens_srv(Bool(True))
                self.episode.setSensorialStateT(
                    (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                     sensorization.obj_box_dist.data * 1000))

                if self.iterations > 0:
                    print '*************************'
                    print "Predicted state: ", candidate_state

                self.motivation_pb.publish(String('Motivation: ' + str(self.activeMot) + '\nActive SUR: ' + str(
                    self.activeCorr + 1) + ', Correlated Sensor: ' + str(self.corr_sensor) + ' ' + str(self.corr_type)))

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
                except rospy.ServiceException, e:
                    rospy.loginfo("Movement service call failed: {0}".format(e))
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
                except rospy.ServiceException, e:
                    rospy.loginfo("Movement service call failed: {0}".format(e))
                self.world_rules()

                # SENSORIZATION in t+1 (distances and reward)
                sensorization = self.get_sens_srv(Bool(True))
                if self.reward:
                    self.episode.setSensorialStateT1((sensorization.obj_rob_dist.data * 1000, 0.0, 0.0))
                else:
                    self.episode.setSensorialStateT1(
                        (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                         sensorization.obj_box_dist.data * 1000))
                self.episode.setReward(self.reward)

                print "Real state: ", self.episode.getSensorialStateT1()
                print '*************************'

                ###########################
                if self.iterations > 0:
                    # self.writeLogs()
                    self.debugPrint()
                    self.saveGraphs()
                ###########################

                # Check if a new correlation is needed or established
                self.correlationsManager.newSUR(self.active_goal)
                if self.correlationsManager.correlations[self.activeCorr].i_reward_assigned == 0:
                    self.correlationsManager.assignRewardAssigner(self.activeCorr, self.episode.getSensorialStateT1(),
                                                              self.active_goal)

                ### Memory Manager: Save episode in the pertinent memories and Traces, weak traces and antitraces
                self.MemoryManager()

                ### Motiv. Manager
                self.MotivationManager()
                # CANDIDATE STATE EVALUATOR and ACTION CHOOSER
                sensorization = self.get_sens_srv(Bool(True))
                self.baxter_gripper_angle = sensorization.grip_angle.data * 180.0 / math.pi
                self.robobo_angle = sensorization.rob_angle.data * 180.0 / math.pi
                # Generate new action
                SimData = (
                    (sensorization.grip_x.data * 1000, sensorization.grip_y.data * 1000), self.baxter_gripper_angle,
                    (sensorization.rob_x.data * 1000, sensorization.rob_y.data * 1000), self.robobo_angle,
                    (sensorization.obj_x.data * 1000, sensorization.obj_y.data * 1000),
                    (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000),
                    (sensorization.box_x.data * 1000, sensorization.box_y.data * 1000),
                    movement_req.dest.const_dist.data * 1000)
                action = self.CSE.getAction(self.activeMot, SimData,
                                            tuple((sensorization.obj_rob_dist.data * 1000,
                                                   sensorization.obj_grip_dist.data * 1000,
                                                   sensorization.obj_box_dist.data * 1000)),
                                            self.corr_sensor, self.corr_type,
                                            self.intrinsicMemory.getContents())
                # Predicted state
                candidate_state = self.CSE.ForwModel.predictedState(action, SimData)
                # print "predicted State en i =: ", self.iterations+1, candidate_state

                # Others
                # self.writeLogs()
                # self.debugPrint()
                self.iter_min += 1
                self.iterations += 1
                self.it_reward += 1
                self.stopCondition()
                self.episode.cleanEpisode()
            self.saveData()
            self.plotGraphs()
            plt.pause(0.0001)

            rospy.sleep(15)
        else:
            rospy.spin()

    def stopCondition(self):
        if self.iterations > 10000:
            self.stop = 1

    def writeLogs(self):
        logging.debug('%s  -  %s  -  %s  -  %s  -  %s  -  %s', self.iterations, self.activeMot, self.activeCorr,
                      self.corr_sensor, self.corr_type, self.episode.getEpisode())

    def debugPrint(self):
        print '------------------'
        print "Iteration: ", self.iterations
        print "Active correlation: ", self.activeCorr
        print "Active motivation: ", self.activeMot
        print "Correlated sensor: ", self.corr_sensor, self.corr_type
        print "Trazas consecutivas S3 neg Reestructuracion: ", self.correlationsManager.correlations[
            0].S3_neg.numberOfGoalsWithoutAntiTraces
        print "Trazas consecutivas S2 neg: ", self.correlationsManager.correlations[
            1].S2_neg.numberOfGoalsWithoutAntiTraces
        print "Trazas consecutivas S1 neg: ", self.correlationsManager.correlations[
            2].S1_neg.numberOfGoalsWithoutAntiTraces
        print "Sensorization: ", tuple(self.episode.getSensorialStateT1())

    def reinitializeMemories(self):
        self.tracesBuffer.removeAll()  # Reinitialize traces buffer
        self.iter_min = 0
        self.intrinsicMemory.removeAll()  # Reinitialize intrinsic memory
        self.intrinsicMemory.addEpisode(self.episode.getSensorialStateT1())

    def MotivationManager(self):
        if self.useMotivManager:
            sensorization = self.get_sens_srv(Bool(True))
            if self.correlationsManager.correlations[self.activeCorr].goal != self.active_goal:  # If the goal changes
                self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(
                    tuple((sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                           sensorization.obj_box_dist.data * 1000)), self.active_goal)
            self.corr_sensor, self.corr_type = self.correlationsManager.getActiveCorrelation(
                tuple((sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                       sensorization.obj_box_dist.data * 1000)), self.activeCorr, self.active_goal)
            if self.corr_sensor == 0:
                self.activeMot = 'Int'
            else:
                if self.activeMot == 'Int':
                    self.iter_min = 0
                self.activeMot = 'Ext'

    def MotivationManagerLTM(self):
        if self.useMotivManager:
            if self.correlationsManager.correlations[self.activeCorr].goal != self.active_goal:  # If the goal changes
                self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(self.get_sensorization_LTM(), self.active_goal)
            self.corr_sensor, self.corr_type = self.correlationsManager.getActiveCorrelation(tuple(self.get_sensorization_LTM()), self.activeCorr, self.active_goal)
            if self.corr_sensor == 0:
                self.activeMot = 'Int'
            else:
                if self.activeMot == 'Int':
                    self.iter_min = 0
                self.activeMot = 'Ext'

    def MemoryManager(self):
        # Save episode in the pertinent memories
        self.tracesBuffer.addEpisode(self.episode.getEpisode())
        self.intrinsicMemory.addEpisode(self.episode.getSensorialStateT1())
        self.memoryVF.addEpisode(self.episode.getEpisode())
        # Memory Manager (Traces, weak traces and antitraces)
        if self.activeMot == 'Int':
            self.it_blind += 1
            self.useMotivManager = 1
            # If there is a reward, realise reward assignment and save trace in Traces Memory
            if self.episode.getReward():
                ###
                if self.correlationsManager.correlations[self.activeCorr].i_reward_assigned == 0:
                    self.correlationsManager.assignRewardAssigner(self.activeCorr,
                                                                  self.episode.getSensorialStateT1(),
                                                                  self.active_goal,
                                                                  1)
                ###
                self.refresh_world_srv(String("motiven"), Bool(True))  # Restart scenario
                self.reward = 0
                self.ball_gripper = False
                self.ball_robobo = False
                self.correlationsManager.correlations[self.activeCorr].correlationEvaluator(
                    self.tracesBuffer.getTrace())
                sensorization = self.get_sens_srv(Bool(True))
                self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(
                    (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                     sensorization.obj_box_dist.data * 1000), self.active_goal)
                self.reinitializeMemories()
                logging.info('Goal reward when Intrinsic Motivation')
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.saveMatrix()
                self.saveData()
                self.TracesMemoryVF.addTraces(self.memoryVF.getTraceReward())
                self.memoryVF.removeAll()
            elif self.correlationsManager.getReward(self.activeCorr, self.reward,
                                                    tuple(self.episode.getSensorialStateT1()), self.active_goal):
                self.correlationsManager.correlations[self.activeCorr].correlationEvaluator(
                    self.tracesBuffer.getTrace())
                # The active correlation is now the correlation that has provided the reward
                self.activeCorr = self.correlationsManager.correlations[self.activeCorr].i_reward
                self.reinitializeMemories()
                logging.info('Correlation reward when Intrinsic Motivation')
        elif self.activeMot == 'Ext':
            self.useMotivManager = 0
            if self.episode.getReward():  # GOAL MANAGER - Encargado de asignar la recompensa?
                self.refresh_world_srv(String("motiven"), Bool(True))  # Restart scenario
                self.reward = 0
                self.ball_gripper = False
                self.ball_robobo = False
                # Save as trace in TracesMemory of the correlated sensor
                self.correlationsManager.correlations[self.activeCorr].addTrace(self.tracesBuffer.getTrace(),
                                                                                self.corr_sensor, self.corr_type)
                sensorization = self.get_sens_srv(Bool(True))
                self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(
                    (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                     sensorization.obj_box_dist.data * 1000), self.active_goal)
                self.reinitializeMemories()
                logging.info('Goal reward when Extrinsic Motivation')
                self.useMotivManager = 1
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.saveMatrix()
                self.saveData()
                self.TracesMemoryVF.addTraces(self.memoryVF.getTraceReward())
                self.memoryVF.removeAll()
            elif self.correlationsManager.getReward(self.activeCorr, self.reward,
                                                    tuple(self.episode.getSensorialStateT1()), self.active_goal):
                # Save as trace in TracesMemory of the correlated sensor
                self.correlationsManager.correlations[self.activeCorr].addTrace(self.tracesBuffer.getTrace(),
                                                                                self.corr_sensor, self.corr_type)
                # The active correlation is now the correlation that has provided the reward
                self.activeCorr = self.correlationsManager.correlations[self.activeCorr].i_reward
                self.reinitializeMemories()
                logging.info('Correlation reward when Extrinsic Motivation')
                self.useMotivManager = 1
            else:
                # Check if the the active correlation is still active
                if self.iter_min > 2:
                    sens_t = self.tracesBuffer.getTrace()[-2][self.corr_sensor - 1]
                    sens_t1 = self.tracesBuffer.getTrace()[-1][self.corr_sensor - 1]
                    dif = sens_t1 - sens_t
                    if (self.corr_type == 'pos' and dif <= 0) or (self.corr_type == 'neg' and dif >= 0):
                        # Guardo antitraza en el sensor correspondiente y vuelvo a comezar el bucle
                        self.correlationsManager.correlations[self.activeCorr].addAntiTrace(
                            self.tracesBuffer.getTrace(), self.corr_sensor, self.corr_type)
                        sensorization = self.get_sens_srv(Bool(True))
                        self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(
                            (sensorization.obj_rob_dist.data * 1000, sensorization.obj_grip_dist.data * 1000,
                             sensorization.obj_box_dist.data * 1000), self.active_goal)
                        self.reinitializeMemories()
                        logging.info('Antitrace in sensor %s of type %s', self.corr_sensor, self.corr_type)
                        logging.info('Sens_t %s, sens_t1 %s, diff %s', sens_t, sens_t1, dif)
                        self.useMotivManager = 1
                        print "ANTITRAZA \n"

    def MemoryManagerLTM(self):
        # Save episode in the pertinent memories
        self.tracesBuffer.addEpisode(self.episode.getEpisode())
        self.intrinsicMemory.addEpisode(self.episode.getSensorialStateT1())
        self.memoryVF.addEpisode(self.episode.getEpisode())
        # Memory Manager (Traces, weak traces and antitraces)
        if self.activeMot == 'Int':
            self.it_blind += 1
            self.useMotivManager = 1
            # If there is a reward, make reward assignment and save trace in Traces Memory
            if self.episode.getReward():
                ###
                if self.correlationsManager.correlations[self.activeCorr].i_reward_assigned == 0:
                    self.correlationsManager.assignRewardAssigner(self.activeCorr, self.episode.getSensorialStateT1(),
                                                                  self.active_goal, 1)
                ###
                self.reward = 0
                self.correlationsManager.correlations[self.activeCorr].correlationEvaluator(self.tracesBuffer.getTrace())
                self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(self.get_sensorization_LTM(), self.active_goal)
                self.reinitializeMemories()
                logging.info('Goal reward when Intrinsic Motivation')
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.saveMatrix()
                self.saveData()
                self.TracesMemoryVF.addTraces(self.memoryVF.getTraceReward())
                self.memoryVF.removeAll()
            elif self.correlationsManager.getReward(self.activeCorr, self.reward,
                                                    tuple(self.episode.getSensorialStateT1()), self.active_goal):
                self.correlationsManager.correlations[self.activeCorr].correlationEvaluator(
                    self.tracesBuffer.getTrace())
                # The active correlation is now the correlation that has provided the reward
                self.activeCorr = self.correlationsManager.correlations[self.activeCorr].i_reward
                self.reinitializeMemories()
                logging.info('Correlation reward when Intrinsic Motivation')
        elif self.activeMot == 'Ext':
            self.useMotivManager = 0
            if self.episode.getReward():  # GOAL MANAGER - Encargado de asignar la recompensa?
                self.reward = 0
                # Save as trace in TracesMemory of the correlated sensor
                self.correlationsManager.correlations[self.activeCorr].addTrace(self.tracesBuffer.getTrace(),
                                                                                self.corr_sensor, self.corr_type)
                self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(self.get_sensorization_LTM(), self.active_goal)
                self.reinitializeMemories()
                logging.info('Goal reward when Extrinsic Motivation')
                self.useMotivManager = 1
                self.it_reward = 0
                self.it_blind = 0
                self.n_execution += 1
                self.saveMatrix()
                self.saveData()
                self.TracesMemoryVF.addTraces(self.memoryVF.getTraceReward())
                self.memoryVF.removeAll()
            elif self.correlationsManager.getReward(self.activeCorr, self.reward,
                                                    tuple(self.episode.getSensorialStateT1()), self.active_goal):
                # Save as trace in TracesMemory of the correlated sensor
                self.correlationsManager.correlations[self.activeCorr].addTrace(self.tracesBuffer.getTrace(),
                                                                                self.corr_sensor, self.corr_type)
                # The active correlation is now the correlation that has provided the reward
                self.activeCorr = self.correlationsManager.correlations[self.activeCorr].i_reward
                self.reinitializeMemories()
                logging.info('Correlation reward when Extrinsic Motivation')
                self.useMotivManager = 1
            else:
                # Check if the the active correlation is still active
                if self.iter_min > 2:  # Aqui debo gestionar lo del contador para las policies usadas por JA
                    sens_t = self.tracesBuffer.getTrace()[-2][self.corr_sensor - 1]
                    sens_t1 = self.tracesBuffer.getTrace()[-1][self.corr_sensor - 1]
                    dif = sens_t1 - sens_t
                    if (self.corr_type == 'pos' and dif <= 0) or (self.corr_type == 'neg' and dif >= 0):
                        self.n_policies_exec += 1
                        if self.n_policies_exec == self.max_policies_exec:
                            # Guardo antitraza en el sensor correspondiente y vuelvo a comezar el bucle
                            self.correlationsManager.correlations[self.activeCorr].addAntiTrace(
                                self.tracesBuffer.getTrace(), self.corr_sensor, self.corr_type)
                            self.activeCorr = self.correlationsManager.getActiveCorrelationPrueba(self.get_sensorization_LTM(), self.active_goal)
                            self.reinitializeMemories()
                            logging.info('Antitrace in sensor %s of type %s', self.corr_sensor, self.corr_type)
                            logging.info('Sens_t %s, sens_t1 %s, diff %s', sens_t, sens_t1, dif)
                            self.useMotivManager = 1
                            print "ANTITRAZA \n"
                            self.n_policies_exec = 0
                    else:
                        self.n_policies_exec = 0  # Si sigo bien la SUR reinicio el numero de policies ejecutadas

    def world_rules(self):
        """Return the reward checking if the ball is inside one of the boxes"""
        sens = self.get_sens_srv(Bool(True))
        if sens.obj_box_dist.data * 1000 < self.min_dist_box and self.ball_gripper:  # distance ball-box1
            print 'Baxter ACTION Drop'
            self.baxter_policy_srv(String('drop_object'))
            self.reward = 1
        elif sens.obj_grip_dist.data * 1000 < self.min_dist_robot and not self.ball_gripper:  # and (sens.obj_x.data < 0.8 and sens.obj_y.data < 0.05 and sens.obj_y.data > -0.73):  # distance ball-baxter_larm
            print '\nBAXTER ACTION PICK', sens.obj_grip_dist.data, '\n'
            rospy.loginfo(self.ball_gripper)
            if self.ball_robobo:
                self.robobo_mov_back_srv(Bool(True), Bool(True))
                self.ball_robobo = False
            self.baxter_sa_srv(Bool(True), Bool(True))
            self.baxter_policy_srv(String('grasp_object'))
            self.ball_gripper = True
        elif sens.obj_rob_dist.data * 1000 < self.min_dist_robot and not self.ball_robobo and not self.ball_gripper:
            print 'ROBOBO ACTION PICK'
            self.robobo_pick_srv(Bool(True), Bool(True))
            self.ball_robobo = True
            self.ball_gripper = False  # Not needed

    def saveGraphs(self):
        # Graph 1 - Iterations to reach the goal vs. Total number of iterations

        self.graph1.append(
            (self.iterations, self.episode.getReward(), self.it_reward, self.it_blind,
             len(self.correlationsManager.correlations), self.activeMot, self.activeCorr,
             self.episode.getSensorialStateT1()))

        self.graph2.append((self.correlationsManager.correlations[-1].S1_neg.numberOfGoalsWithoutAntiTraces,
                            self.correlationsManager.correlations[-1].S1_pos.numberOfGoalsWithoutAntiTraces,
                            self.correlationsManager.correlations[-1].S2_neg.numberOfGoalsWithoutAntiTraces,
                            self.correlationsManager.correlations[-1].S2_pos.numberOfGoalsWithoutAntiTraces,
                            self.correlationsManager.correlations[-1].S3_neg.numberOfGoalsWithoutAntiTraces,
                            self.correlationsManager.correlations[-1].S3_pos.numberOfGoalsWithoutAntiTraces))
        # Save executions
        self.graphExec.append(
            (self.iterations, self.episode.getReward(), self.it_reward, self.it_blind,
             len(self.correlationsManager.correlations), self.activeMot, self.activeCorr,
             self.episode.getSensorialStateT1()))

    def saveMatrix(self):
        self.graphx.append(self.graphExec)
        self.graphExec = []

    def plotGraphs(self):
        # Graph 1
        fig = plt.figure()
        ax = fig.add_subplot(111)
        n_reward = 0
        iter_goal = []  # Number of iteration increase at the same time as goals
        for i in range(len(self.graph1)):
            # for i in range(40000):
            if self.graph1[i][1]:
                n_reward += 1
                iter_goal.append(self.graph1[i][0])
                if self.graph1[i][7][1] == 0.0:  # Distance Baxter-ball=0.0 when reward
                    # plt.plot(self.graph1[i][0], self.graph1[i][2], 'ro', color='red')
                    ax.plot(n_reward, self.graph1[i][2], 'ro', color='red')
                else:  # The reward is given to the Robobo
                    # plt.plot(self.graph1[i][0], self.graph1[i][2], 'ro', color='blue')
                    ax.plot(n_reward, self.graph1[i][2], 'ro', color='blue')
            if self.graph1[i][4] > self.graph1[i - 1][4]:
                ax.axvline(x=n_reward)
        # for i in range(len(self.graph1)):
        #     if self.graph1[i][4] > self.graph1[i - 1][4]:
        #         plt.axvline(x=self.graph1[i][0])

        # plt.axes().set_xlabel('Iterations')
        ax.set_xlabel('Number of rewards')
        ax.set_ylabel('Iterations needed to reach the goal')
        ax.grid()
        ax2 = ax.twinx()
        print "range(n_reward)", range(n_reward), 'iter_goal', iter_goal
        ax2.plot(range(n_reward), iter_goal, marker='.', markersize=1.0, color='green', linewidth=1.0, label='active')
        ax2.set_ylabel('Number of iterations')
        # plt.xlim(0, 40000)
        # plt.show()
        # Simple moving average
        reward_matrix = []
        blind_matrix = []
        iter = []
        for i in range(len(self.graph1)):
            # for i in range(40000):
            if self.graph1[i][1]:
                reward_matrix.append(self.graph1[i][2])
                blind_matrix.append(min(self.graph1[i][3] / 50, 1))
                iter.append(self.graph1[i][0])
        window = 10
        window_aux = 1
        media = self.calcSma(reward_matrix, window)
        media_aux = self.calcSma(reward_matrix[:window - 1], window_aux)
        media_sum = media_aux + media[window - 1:]
        # plt.plot(iter, media_sum, marker='.', color='cyan', linewidth=0.5, label='simple moving average')
        print "media_sum", media_sum
        ax.plot(range(n_reward), media_sum, marker='.', markersize=2.0, color='cyan', linewidth=2.0,
                label='simple moving average')
        # print media_sum
        # print media_sum[-1]
        # print media_sum[205], media_sum[204], media_sum[206]
        #
        # Graph 2
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for i in range(len(self.graph1)):
            if self.graph1[i][1]:
                ax.plot(self.graph1[i][0], self.graph1[i][3], 'ro', color='green')
        for i in range(len(self.graph1)):
            if self.graph1[i][4] > self.graph1[i - 1][4]:
                ax.axvline(x=self.graph1[i][0])
        ax.set_xlabel('Iterations')
        ax.set_ylabel('Iterations the Ib motivation is active')
        ax.grid()
        ax2 = ax.twinx()
        # Simple moving average: 1 = use of Ib, 0 = no use of Ib
        window = 10
        window_aux = 1
        media = self.calcSma(blind_matrix, window)
        media_aux = self.calcSma(blind_matrix[:window - 1], window_aux)
        media_sum = media_aux + media[window - 1:]
        ax2.plot(iter, media_sum, marker='.', markersize=1.0, color='orange', linewidth=1.0, label='active')
        ax2.set_ylabel('Use of Ib')
        # plt.xlim(0, 40000)
        plt.show()

    def calcSma(self, data, smaPeriod):
        j = next(i for i, x in enumerate(data) if x is not None)
        our_range = range(len(data))[j + smaPeriod - 1:]
        empty_list = [None] * (j + smaPeriod - 1)
        sub_result = [np.mean(data[i - smaPeriod + 1:i + 1]) for i in our_range]

        return list(empty_list + sub_result)

    def saveData(self):  # , action, seed):

        f = open('SimulationDataRealFinal' + str(self.n_execution) + '.pckl', 'wb')
        pickle.dump(len(self.correlationsManager.correlations), f)
        for i in range(len(self.correlationsManager.correlations)):
            pickle.dump(self.correlationsManager.correlations[i].S1_pos, f)
            pickle.dump(self.correlationsManager.correlations[i].S1_neg, f)
            pickle.dump(self.correlationsManager.correlations[i].S2_pos, f)
            pickle.dump(self.correlationsManager.correlations[i].S2_neg, f)
            pickle.dump(self.correlationsManager.correlations[i].S3_pos, f)
            pickle.dump(self.correlationsManager.correlations[i].S3_neg, f)
            pickle.dump(self.correlationsManager.correlations[i].corr_active, f)
            pickle.dump(self.correlationsManager.correlations[i].corr_type, f)
            pickle.dump(self.correlationsManager.correlations[i].established, f)
            pickle.dump(self.correlationsManager.correlations[i].corr_established, f)
            pickle.dump(self.correlationsManager.correlations[i].corr_established_type, f)
            pickle.dump(self.correlationsManager.correlations[i].i_reward, f)
            pickle.dump(self.correlationsManager.correlations[i].i_reward_assigned, f)
        # pickle.dump(self.activeMot, f)
        # pickle.dump(self.activeCorr, f)
        # pickle.dump(self.corr_sensor, f)
        # pickle.dump(self.corr_type, f)
        pickle.dump(self.graph1, f)
        pickle.dump(self.graphx, f)
        pickle.dump(self.graph2, f)
        f.close()

    def loadData(self):

        # f = open('SimulationDataLongExecBis2.pckl', 'rb')  # SimulationDataLongExecBis2
        # numero = pickle.load(f)
        # for i in range(numero):  # for i in range(numero):
        #     self.correlationsManager.correlations.append(Correlations(None))
        # for i in range(numero):  # for i in range(numero):
        #     self.correlationsManager.correlations[i].S1_pos = pickle.load(f)
        #     self.correlationsManager.correlations[i].S1_neg = pickle.load(f)
        #     self.correlationsManager.correlations[i].S2_pos = pickle.load(f)
        #     self.correlationsManager.correlations[i].S2_neg = pickle.load(f)
        #     self.correlationsManager.correlations[i].S3_pos = pickle.load(f)
        #     self.correlationsManager.correlations[i].S3_neg = pickle.load(f)
        #     self.correlationsManager.correlations[i].corr_active = pickle.load(f)
        #     self.correlationsManager.correlations[i].corr_type = pickle.load(f)
        #     self.correlationsManager.correlations[i].established = pickle.load(f)
        #     self.correlationsManager.correlations[i].corr_established = pickle.load(f)
        #     self.correlationsManager.correlations[i].corr_established_type = pickle.load(f)
        #     self.correlationsManager.correlations[i].i_reward = pickle.load(f)
        #     self.correlationsManager.correlations[i].i_reward_assigned = pickle.load(f)
        # # # self.activeMot = pickle.load(f)
        # # # self.activeCorr = pickle.load(f)
        # # # self.corr_sensor = pickle.load(f)
        # # # self.corr_type = pickle.load(f)
        # # self.graph1 = pickle.load(f)
        # # self.graphx = pickle.load(f)
        # # self.graph2 = pickle.load(f)
        # f.close()

        f = open('SimulationDataRealSUR3R75.pckl', 'rb')  # SimulationDataLongExecBis2 SimulationDataRealSUR3R75
        numero = pickle.load(f)
        for i in range(numero):  # for i in range(numero):
            self.correlationsManager.correlations.append(Correlations(None))
        for i in range(numero):  # for i in range(numero):
            self.correlationsManager.correlations[i].S1_pos = pickle.load(f)
            self.correlationsManager.correlations[i].S1_neg = pickle.load(f)
            self.correlationsManager.correlations[i].S2_pos = pickle.load(f)
            self.correlationsManager.correlations[i].S2_neg = pickle.load(f)
            self.correlationsManager.correlations[i].S3_pos = pickle.load(f)
            self.correlationsManager.correlations[i].S3_neg = pickle.load(f)
            self.correlationsManager.correlations[i].corr_active = pickle.load(f)
            self.correlationsManager.correlations[i].corr_type = pickle.load(f)
            self.correlationsManager.correlations[i].established = pickle.load(f)
            self.correlationsManager.correlations[i].corr_established = pickle.load(f)
            self.correlationsManager.correlations[i].corr_established_type = pickle.load(f)
            self.correlationsManager.correlations[i].i_reward = pickle.load(f)
            self.correlationsManager.correlations[i].i_reward_assigned = pickle.load(f)
        # # self.activeMot = pickle.load(f)
        # # self.activeCorr = pickle.load(f)
        # # self.corr_sensor = pickle.load(f)
        # # self.corr_type = pickle.load(f)
        # self.graph1 = pickle.load(f)
        # self.graphx = pickle.load(f)
        # self.graph2 = pickle.load(f)
        f.close()

        f = open('GraphData.pckl', 'rb')
        self.graph1 = pickle.load(f)
        self.graphx = pickle.load(f)
        self.graph2 = pickle.load(f)
        f.close()
        # self.correlationsManager.correlations[0].S1_pos = DistancesCertainty()
        # self.correlationsManager.correlations[0].S1_neg = DistancesCertainty()
        # self.correlationsManager.correlations[0].S2_pos = DistancesCertainty()
        # self.correlationsManager.correlations[0].S2_neg = DistancesCertainty()
        # self.correlationsManager.correlations[0].S3_pos = DistancesCertainty()
        # self.correlationsManager.correlations[0].S3_neg.numberOfGoalsWithoutAntiTraces = 0
        self.n_execution = 362
        self.iterations = 50483

        # self.correlationsManager.correlations[1].S1_pos = DistancesCertainty()
        # self.correlationsManager.correlations[1].S1_neg = DistancesCertainty()
        # self.correlationsManager.correlations[1].S2_pos = DistancesCertainty()
        # self.correlationsManager.correlations[1].S3_neg = DistancesCertainty()
        # self.correlationsManager.correlations[1].S3_pos = DistancesCertainty()
        # self.correlationsManager.correlations[1].S3_neg.numberOfGoalsWithoutAntiTraces = 0
        #
        # self.correlationsManager.correlations[2].S1_neg = self.correlationsManager.correlations[3].S1_neg
        # self.correlationsManager.correlations[2].S1_pos = DistancesCertainty()
        # self.correlationsManager.correlations[2].S2_neg = DistancesCertainty()
        # self.correlationsManager.correlations[2].S2_pos = DistancesCertainty()
        # self.correlationsManager.correlations[2].S3_neg = DistancesCertainty()
        # self.correlationsManager.correlations[2].S3_pos = DistancesCertainty()
        # #
        # self.correlationsManager.correlations[3].S1_pos = DistancesCertainty()
        # self.correlationsManager.correlations[3].S2_neg = DistancesCertainty()
        # self.correlationsManager.correlations[3].S2_pos = DistancesCertainty()
        # self.correlationsManager.correlations[3].S3_neg = DistancesCertainty()
        # self.correlationsManager.correlations[3].S3_pos = DistancesCertainty()

        # self.correlationsManager.correlations[4].S1_pos = DistancesCertainty()
        # self.correlationsManager.correlations[4].S1_neg = DistancesCertainty()
        # self.correlationsManager.correlations[4].S2_pos = DistancesCertainty()
        # self.correlationsManager.correlations[4].S3_neg = DistancesCertainty()
        # self.correlationsManager.correlations[4].S3_pos = DistancesCertainty()

        print "Corr est", self.correlationsManager.correlations[0].established, self.correlationsManager.correlations[
            0].corr_established, self.correlationsManager.correlations[0].corr_established_type
        print "Corr est 2", self.correlationsManager.correlations[1].established, self.correlationsManager.correlations[
            1].corr_established, self.correlationsManager.correlations[1].corr_established_type
        print "Corr est 3", self.correlationsManager.correlations[2].established, self.correlationsManager.correlations[
            2].corr_established, self.correlationsManager.correlations[2].corr_established_type
        # print "Corr est 4", self.correlationsManager.correlations[3].established, self.correlationsManager.correlations[
        #     3].corr_established, self.correlationsManager.correlations[3].corr_established_type
        # print "Corr est 5", self.correlationsManager.correlations[4].established, self.correlationsManager.correlations[
        #     4].corr_established, self.correlationsManager.correlations[4].corr_established_type


def main():
    instance = MDBCore()
    instance.run()


if __name__ == '__main__':
    main()
