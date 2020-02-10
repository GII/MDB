"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
from mdb_ltm.node import Node


class PNode(Node):
    """
    A subspace of the input space (sensorial or result of a redescription process).

    This subspace is linked to every node for which it is relevant,
    activating them when a new perception pertaining to this subspace occurs.
    """

    def __init__(self, space_class=None, **kwargs):
        """Constructor."""
        self.space = self.class_from_classname(space_class)(ident=kwargs.get("ident") + " space")
        super().__init__(**kwargs)

    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""
        self.space.add_point(perception, confidence)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        return self.space.get_probability(perception)
