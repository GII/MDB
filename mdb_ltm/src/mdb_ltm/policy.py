"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
from operator import attrgetter
import rospy
from mdb_ltm.node import Node


class Policy(Node):
    """Action or set of actions (behaviour in Evolutionary Robotics / controller in Robotics)."""

    def __init__(self, ros_name_prefix=None, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.topic = rospy.get_param(ros_name_prefix + "_topic")
        self.message = self.class_from_classname(rospy.get_param(ros_name_prefix + "_msg"))
        self.publisher = None
        self.init_ros()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["publisher"]
        return state

    def init_ros(self):
        """Create publishers and make subscriptions."""
        self.publisher = rospy.Publisher(self.topic, self.message, latch=True, queue_size=0)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        raise NotImplementedError

    def update_activation(self, **kwargs):
        """
        Calculate the new activation value.

        This activation value is the sum of the connected c-nodes.
        """
        cnodes = [node for node in self.neighbors if node.type == "CNode"]
        if cnodes:
            cnode = max(cnodes, key=attrgetter("activation"))
            self.perception = cnode.perception
            self.activation = cnode.activation
        else:
            self.perception = []
            self.activation = 0.0
        rospy.logdebug(self.type + " activation for " + self.ident + " = " + str(self.activation))

    def execute(self):
        """Run the policy."""
        rospy.loginfo("Executing policy " + self.ident)
        self.publisher.publish(self.ident)


class SuperThrow(Policy):
    """Interface to the learnt throw policy."""

    def __init__(self, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.throw_topic = rospy.get_param("/mdb/superthrow_topic")
        self.throw_message = self.class_from_classname(rospy.get_param("/mdb/superthrow_msg"))
        self.throw_publisher = None
        self.real_robot = kwargs.get("Data")

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = super().__getstate__()
        del state["throw_publisher"]
        return state

    def init_ros(self):
        """Create publishers and make subscriptions."""
        super().init_ros()
        if self.real_robot:
            self.throw_publisher = rospy.Publisher(self.throw_topic, self.throw_message, latch=True, queue_size=0)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        raise NotImplementedError

    def execute(self):
        """Run the policy."""
        super().execute()
        if self.real_robot:
            rospy.loginfo('Sending "set basket target position" to baxter_throwing' + self.ident)
            self.throw_publisher.publish(9)
            rospy.sleep(4)
            rospy.loginfo('Sending "go to the initial posture" to baxter_throwing' + self.ident)
            self.throw_publisher.publish(4)
            rospy.sleep(8)
            rospy.loginfo('Sending "throw the ball" to baxter_throwing' + self.ident)
            self.throw_publisher.publish(5)
