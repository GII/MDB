"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import * #noqa

class Goal(object):
    """
    Class that represents a Goal.

    This class implements different methods to get/set the goal 'id' and its 'activation',
    as well as many other possible characteristics.
    """

    def __init__(self, id):
        self.goal_id = id
        self.activation = 0.0

    def getGoalId(self):
        return self.goal_id

    def getActivation(self):
        return self.activation

    def getGoal(self):
        return self.goal_id, self.activation

    def setGoalId(self, id):
        self.goal_id = id

    def setActivation(self, activation_value):
        self.activation = activation_value

    def setGoal(self, id, activation_value):
        self.goal_id = id
        self.activation = activation_value
