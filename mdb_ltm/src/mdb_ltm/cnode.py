"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

import numpy
from mdb_ltm.node import Node


class CNode(Node):
    """
    It represents a context, that is, a link between nodes that were activated together in the past.

    It is assumed that there is only one element of each type connected to the C-Node.
    """

    def update_activation(self, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the product of its connected nodes, excluding the policy.
        """
        self.activation = numpy.prod(node.activation for node in self.neighbors if node.type != "Policy")
        super(CNode, self).update_activation(**kwargs)

    def context_is_on(self):
        """Check if the context is activated or not."""
        return (
            len([node for node in self.neighbors if node.type != "Policy" and node.activation >= node.threshold]) == 3
        )

    def context_has_reward(self):
        """
        Check if the context contains a goal that is being accomplished.

        The result of this check is true even if the goal or the P-Node are not activated.
        """
        return (
            len(
                [
                    node
                    for node in self.neighbors
                    if (node.type == "ForwardModel" and node.activation >= node.threshold)
                    or (node.type == "Goal" and node.reward >= node.threshold)
                ]
            )
            == 2
        )
