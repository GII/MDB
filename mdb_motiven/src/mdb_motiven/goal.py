"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import


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
