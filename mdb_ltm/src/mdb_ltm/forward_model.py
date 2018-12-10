"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
from mdb_ltm.node import Node


class ForwardModel(Node):
    """A model of the environment (including the own robot) for a given set of sensors, so it can be used for prediction."""

    def __init__(self, **kwargs):
        """Constructor."""
        super(ForwardModel, self).__init__(**kwargs)

    def update_activation(self, **kwargs):
        """
        Calculate the new activation value.

        By default, the activation value is 1 if we are in the right world, 0 otherwise.
        """
        if self.ltm.current_world == self.ident:
            self.activation = 1.0
        else:
            self.activation = 0.0
        super(ForwardModel, self).update_activation(**kwargs)


class ForwardModelGripper(ForwardModel):
    """
    A world with grippers and a low friction table.

    This forward model corresponds with a world where the robot has grippers in its hands but there is no control when
    moving small objects due to the low friction of the table.
    """


class ForwardModelNoGripper(ForwardModel):
    """
    A world without grippers and a high friction table.

    This forward model corresponds with a world where the robot has not grippers in its hands but the friction is high
    so it can move, in a controllable way, small objects pushing them.
    """
