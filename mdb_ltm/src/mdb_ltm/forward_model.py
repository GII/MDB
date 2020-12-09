"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# Library imports
import numpy

# MDB imports
from mdb_ltm.node import Node


class ForwardModel(Node):
    """A model of the environment (including the own robot) for a given set of sensors."""

    @property
    def max_activation(self):
        """Return the maximum value of all activations."""
        return numpy.max(self.activation)

    def calc_activation(self, perception=None):
        """
        Calculate the new activation value.

        By default, the activation value is 1 if we are in the right world, 0 otherwise.
        """
        if self.ident in self.ltm.current_world:
            activation = 1.0
        else:
            activation = 0.0
        return activation
