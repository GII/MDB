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
import sys
from collections import OrderedDict
import rospy


class Node(object):
    """An MDB element."""

    def __init__(
        self, ident=None, node_type=None, threshold=0.1, ltm=None, ros_node_prefix=None, ros_data_prefix=None, **kwargs
    ):
        """Init attributes when a new object is created."""
        self.ident = ident
        self.type = node_type
        self.perception = None
        self.activation = None
        self.threshold = threshold
        self.neighbors = []
        self.ltm = ltm
        self.ros_node_prefix = ros_node_prefix
        self.ros_data_prefix = ros_data_prefix
        self.init_ros()

    @staticmethod
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        if sys.version_info < (3, 0):
            node_module = __import__(module_string, fromlist=[bytes(class_string, "utf-8")])
        else:
            node_module = __import__(module_string, fromlist=[class_string])
        # node_module = importlib.import_module('.' + class_string, package=module_string)
        node_class = getattr(node_module, class_string)
        return node_class

    def init_ros(self):
        """Create publishers and make subscriptions."""
        if self.ros_node_prefix is not None:
            self.node_topic = rospy.get_param(self.ros_node_prefix + "_topic")
            self.node_message = self.class_from_classname(rospy.get_param(self.ros_node_prefix + "_msg"))
            self.node_publisher = rospy.Publisher(self.node_topic, self.node_message, latch=True, queue_size=0)
        if self.ros_data_prefix is not None:
            self.data_topic = rospy.get_param(self.ros_data_prefix + "_topic")
            self.data_message = self.class_from_classname(rospy.get_param(self.ros_data_prefix + "_msg"))
            self.data_publisher = rospy.Publisher(self.data_topic, self.data_message, latch=True, queue_size=0)
        self.publish()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        if self.ros_node_prefix is not None:
            del state["node_publisher"]
        if self.ros_data_prefix is not None:
            del state["data_publisher"]
        return state

    def publish(self):
        """Publish node information."""
        message = self.node_message()
        message.command = "update"
        message.id = self.ident
        if isinstance(self.activation, list):
            message.activation = max(self.activation)
        else:
            message.activation = self.activation
        message.execute_service = ""
        message.get_service = ""
        message.class_name = ""
        message.language = ""
        self.node_publisher.publish(message)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        raise NotImplementedError

    def update_activation(self, **kwargs):
        """Calculate a new activation value for each perception."""
        perception = kwargs.pop("perception")
        self.perception = []
        self.activation = []
        for i in range(max([len(sensor) for sensor in perception.values()])):
            perception_line = OrderedDict()
            for sensor, value in perception.items():
                sid = i % len(value)
                perception_line[sensor + str(sid)] = value[sid]
            self.perception.append(perception_line)
            activation_value = self.calc_activation(perception_line)
            if activation_value < self.threshold:
                activation_value = 0.0
            self.activation.append(activation_value)
            rospy.logdebug(self.type + " activation for " + self.ident + " = " + str(activation_value))
        self.publish()
