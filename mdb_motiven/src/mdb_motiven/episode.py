"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import * #noqa

class Episode(object):
    """
    Class that represents an Episode.

    An Episode (E) is a sample of the real world response to the robot actions.
    Within the MDB, an episode is made up of the sensorial state in t, the applied action in t,
    the sensorial state in t + 1 and the reward in t + 1:
    episode = {S(t), A(t), S(t+1), R(t+1)}
    This class implements different methods to get/set the sensorial states, action or reward
    of the episode.
    """

    def __init__(self):
        self.sensorialStateT = []
        self.actionT = 0
        self.sensorialStateT1 = []
        self.rewardT1 = 0
        # self.motivation = ''

    def getSensorialStateT(self):
        return self.sensorialStateT

    def getAction(self):
        return self.actionT

    def getSensorialStateT1(self):
        return self.sensorialStateT1

    def getReward(self):
        return self.rewardT1

    # def getMotivation(self):
    #     return self.motivation

    def getEpisode(self):
        return [self.sensorialStateT, self.actionT, self.sensorialStateT1, self.rewardT1]  #, self.motivation]

    def setSensorialStateT(self, sensorialState):
        self.sensorialStateT = sensorialState

    def setAction(self, action):
        self.actionT = action

    def setSensorialStateT1(self, sensorialState):
        self.sensorialStateT1 = sensorialState

    def setReward(self, reward):
        self.rewardT1 = reward

    # def setMotivation(self, motivation):
    #     self.motivation = motivation

    def setEpisode(self, sensorialStateT, action, sensorialStateT1, reward):#, motivation):
        self.sensorialStateT = sensorialStateT
        self.actionT = action
        self.sensorialStateT1 = sensorialStateT1
        self.rewardT1 = reward
        # self.motivation = motivation

    def cleanEpisode(self):
        self.sensorialStateT = []
        self.actionT = 0
        self.sensorialStateT1 = []
        self.rewardT1 = 0
        # self.motivation = ''
