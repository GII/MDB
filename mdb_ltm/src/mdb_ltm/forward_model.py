"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import (  # noqa pylint: disable=unused-import
    bytes,
    dict,
    int,
    list,
    object,
    range,
    str,
    ascii,
    chr,
    hex,
    input,
    next,
    oct,
    open,
    pow,
    round,
    super,
    filter,
    map,
    zip,
)
import numpy
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
