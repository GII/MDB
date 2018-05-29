"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

import math
import rospy
from mdb_ltm.node import Node


class Policy(Node):
    """Action or set of actions (behaviour in Evolutionary Robotics / controller in Robotics)."""

    def __init__(self, ros_name_prefix=None, **kwargs):
        """Constructor."""
        super(Policy, self).__init__(**kwargs)
        topic = rospy.get_param(ros_name_prefix + '_topic')
        message = self.class_from_classname(rospy.get_param(ros_name_prefix + '_msg'))
        self.publisher = rospy.Publisher(topic, message, latch=True, queue_size=None)

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state['publisher']
        return state

    def update_activation(self, ros_name_prefix=None, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the sum of the connected c-nodes.
        """
        self.activation = min(1.0, math.fsum(node.activation for node in self.neighbors if node.type == 'CNode'))
        super(Policy, self).update_activation(**kwargs)

    def execute(self):
        """Run the policy."""
        rospy.loginfo('Executing policy ' + self.ident)
        self.publisher.publish(self.ident)
