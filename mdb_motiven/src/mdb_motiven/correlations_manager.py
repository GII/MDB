"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import * #noqa
from mdb_motiven.correlations import Correlations


class CorrelationsManager(object):
    """
    Class that represents the Correlations Manager module.

    This module identifies when new correlations are needed and contains the set of existing correlations.
    It contains a list with all the existing Correlations.
    It also chooses the active correlation and gives the value of the reward based on this active correlation.
    """

    def __init__(self):
        self.correlations = []
        self.threshold = 0.1  # Threshold to know when to give reward to the sub-correlations

    def newSUR(self, active_goal):
        """ This method decides when a new SUR has to be created. Two conditions are considered to do it:
        1- There are no SURs associated with the active goal.
        2- All the SURs associated with the active goal are established.

        :return:
        """
        surs_asoc_active_goal = 0  # Used to count the number of SURs associated with the active goal. Condition 1
        index_last_sur_asoc = None  # Used to save the index of the last SUR associated wiht the active goal. Condition 2
        for i in range(len(self.correlations)):
            if self.correlations[i].goal == active_goal:
                surs_asoc_active_goal += 1
                index_last_sur_asoc = i
        if surs_asoc_active_goal == 0:  # Condition 1
            self.correlations.append(Correlations(None, active_goal))
            # self.correlations[-1].figure.canvas.set_window_title('SUR' + ' ' + str(len(self.correlations) - 1))
            # rospy.loginfo('New correlation. Number of existing correlations: %s', len(self.correlations))
        elif self.correlations[index_last_sur_asoc].established:  # Condition 2
            self.correlations.append(Correlations(None, active_goal))
            # self.correlations[-1].figure.canvas.set_window_title('SUR' + ' ' + str(len(self.correlations) - 1))
            # rospy.loginfo('New correlation. Number of existing correlations: %s', len(self.correlations))
        # if len(self.correlations) == 0:
        #     self.correlations.append(Correlations(None))
        #     self.correlations[-1].figure.canvas.set_window_title('Correlation' + ' ' + str(len(self.correlations) - 1))
        #     rospy.loginfo('New correlation. Number of existing correlations: %s', len(self.correlations))
        #
        #     self.correlations[-1].i_reward_assigned = 1
        #
        # if len(self.correlations) < 3:
        #     if self.correlations[-1].established:
        #         self.correlations.append(Correlations(len(self.correlations) - 1))
        #         #self.correlations.append(Correlations(0))
        #         self.correlations[-1].figure.canvas.set_window_title(
        #             'Correlation' + ' ' + str(len(self.correlations) - 1))
        #         rospy.loginfo('New correlation. Number of existing correlations: %s', len(self.correlations))

    def getActiveCorrelation(self, p, active_corr, active_goal):
        """ This method provides the active correlation among all the possible correlations for a given point p

        :return: active_correlation
        """
        # active_corr = len(self.correlations)-1
        # max_certainty = 0
        # for i in range(len(self.correlations)):
        #     certainty = self.correlations[i].getCertainty(p)
        #     if certainty > max_certainty:
        #         max_certainty = certainty
        #         active_corr = i
        corr_sensor, corr_type = self.correlations[active_corr].getActiveCorrelation(p, active_goal)
        return corr_sensor, corr_type  # active_corr, corr_sensor, corr_type

    def getActiveCorrelationPrueba(self, p, active_goal):
        # active_corr = len(self.correlations)-1
        for i in range(len(self.correlations)):  # Index last sur asociada al goal, que debe ser la que no este consolidada
            if self.correlations[i].goal == active_goal:
                active_corr = i
        max_certainty = 0
        for i in range(len(self.correlations)):
            certainty = self.correlations[i].getCertainty(p, active_goal)
            if certainty > max_certainty:
                max_certainty = certainty
                active_corr = i
        return active_corr

    # def getActiveCorrelation(self, p):
    #     """ This method provides the active correlation among all the possible correlations for a given point p
    #
    #     :return: active_correlation
    #     """
    #     active_corr = len(self.correlations)-1
    #     max_certainty = 0
    #     for i in range(len(self.correlations)):
    #         certainty = self.correlations[i].getCertainty(p)
    #         if certainty > max_certainty:
    #             max_certainty = certainty
    #             active_corr = i
    #
    #     corr_sensor, corr_type = self.correlations[active_corr].getActiveCorrelation(p)
    #
    #     return corr_sensor, corr_type, active_corr

    def getReward(self, active_corr, simulator, p, active_goal):
        """This method is in charge of provide reward if required
        :param: active_corr: index of the active correlation needed to know who is providing its reward
        :return: reward
        """
        i_r = self.correlations[active_corr].i_reward
        # if i_r is None:
        #     reward = self.simulator.getReward()
        # elif self.correlations[i_r].getCertainty() > self.threshold:
        if i_r is None:
            reward = simulator
        elif self.correlations[i_r].getCertainty(p, active_goal) > self.threshold:
            reward = 1
        else:
            reward = 0
        return reward

    def assignRewardAssigner(self, active_corr, p, active_goal, scenario=0):
        ####
        # self.correlations[active_corr].associated_goal = goal_id
        ####
        ###
        if scenario:
            self.correlations[active_corr].i_reward = None
            self.correlations[active_corr].i_reward_assigned = 1
            # self.correlations[active_corr].associated_goal = goal_id
        else:
        ###
            for i in range(len(self.correlations[:active_corr])):
                if self.correlations[i].getCertainty(p, active_goal) > self.threshold:
                    self.correlations[active_corr].i_reward = i
                    self.correlations[active_corr].i_reward_assigned = 1
                    # self.correlations[active_corr].associated_goal = goal_id
                    break
