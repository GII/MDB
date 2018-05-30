"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

import math
from mdb_ltm.node import Node


class CNode(Node):
    """
    It represents a context, that is, a link between nodes that were activated together in the past.

    Attributes:

    """

    def update_activation(self, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the product of the sums for each type of its connected nodes.
        """
        pnode_act = min(1.0, math.fsum(node.activation for node in self.neighbors if node.type == 'PNode'))
        fm_act = min(1.0, math.fsum(node.activation for node in self.neighbors if node.type == 'ForwardModel'))
        goal_act = min(1.0, math.fsum(node.activation for node in self.neighbors if node.type == 'Goal'))
        self.activation = pnode_act * fm_act * goal_act
        super(CNode, self).update_activation(**kwargs)

    def context_is_on(self):
        """Check if the context of this CNode is activated or not."""
        fms = [node for node in self.neighbors if node.type == 'ForwardModel' and node.activation >= node.threshold]
        goals = [node for node in self.neighbors if node.type == 'Goal' and node.activation >= node.threshold]
        if (fms) and (goals):
            return True
        else:
            return False

    def context_has_reward(self):
        """Check if the context of this CNode is activated or not."""
        fms = [node for node in self.neighbors if node.type == 'ForwardModel' and node.activation >= node.threshold]
        goals = [node for node in self.neighbors if node.type == 'Goal' and node.reward >= node.threshold]
        if (fms) and (goals):
            return True
        else:
            return False
