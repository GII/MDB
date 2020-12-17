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
import rospy

# MDB imports
from mdb_ltm.node import Node


class CNode(Node):
    """
    It represents a context, that is, a link between nodes that were activated together in the past.

    It is assumed that there is only one element of each type connected to the C-Node.
    """

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        raise NotImplementedError

    def update_activation(self, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the product of the activation value of the connected nodes, excluding the policy.
        It is assumed that all the neighbors have the same list of perceptions but, probably, it should
        be checked (although this would have a huge performance penalty).
        """
        pnode = [node for node in self.neighbors if node.type == "PNode"][0]
        activation_list = numpy.prod([node.activation for node in self.neighbors if node.type != "Policy"], axis=0)
        self.activation = numpy.max(activation_list)
        if self.activation > self.threshold:
            self.perception = pnode.perception[numpy.argmax(activation_list)]
        else:
            # Even if there is not C-node activation, we want to know the perception that leaded to the hightest
            # P-node activation, in order to add a point to the P-node / create a new P-node and C-node
            # when a random policy is executed and that execution would satisfy a goal in that goal was activated.
            # This is an interin solution, see __add_point()...
            if numpy.max(pnode.activation) > self.threshold:
                self.perception = pnode.perception[numpy.argmax(pnode.activation)]
            else:
                self.perception = []
        rospy.logdebug(self.type + " activation for " + self.ident + " = " + str(self.activation))
        self.publish()

    def context_has_reward(self):
        """
        Check if the context contains a goal that is being accomplished.

        The result of this check is true even if the goal or the P-Node are not activated, but the Forward
        Model needs to be activated.
        """
        return (
            len(
                [
                    node
                    for node in self.neighbors
                    if (node.type == "ForwardModel" and max(node.activation) >= node.threshold)
                    or (node.type == "Goal" and node.reward >= node.threshold)
                ]
            )
            == 2
        )

    @property
    def p_node(self):
        """Return the P-Node of this context."""
        for node in self.neighbors:
            if node.type == "PNode":
                return node
        return None

    @property
    def forward_model(self):
        """Return the forward model of this context."""
        for node in self.neighbors:
            if node.type == "ForwardModel":
                return node
        return None

    @property
    def goal(self):
        """Return the goal of this context."""
        for node in self.neighbors:
            if node.type == "Goal":
                return node
        return None

    @property
    def policy(self):
        """Return the policy of this context."""
        for node in self.neighbors:
            if node.type == "Policy":
                return node
        return None
