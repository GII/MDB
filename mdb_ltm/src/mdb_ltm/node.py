"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import importlib
import rospy


class Node(object):
    """An MDB element."""

    def __init__(self, ident, node_type, threshold=0.1, ltm=None, **kwargs):
        """Constructor."""
        self.ident = ident
        self.type = node_type
        self.perception = None
        self.activation = None
        self.threshold = threshold
        self.neighbors = []
        self.ltm = ltm

    @staticmethod
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        node_module = __import__(module_string, fromlist=[bytes(class_string, 'utf-8')])
        # node_module = importlib.import_module('.' + class_string, package=module_string)
        node_class = getattr(node_module, class_string)
        return node_class

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        raise NotImplementedError

    def update_activation(self, **kwargs):
        """Calculate a new activation value for each perception."""
        perception = kwargs.pop("perception")
        self.perception = []
        self.activation = []
        for i in range(max([len(i) for i in perception])):
            perception_line = []
            for sensor in perception:
                perception_line.extend(sensor[i % len(sensor)])
            self.perception.append(perception_line)
            kwargs["perception"] = perception_line
            activation_value = self.calc_activation(**kwargs)
            if activation_value < self.threshold:
                activation_value = 0.0
            self.activation.append(activation_value)
            rospy.logdebug(self.type + " activation for " + self.ident + " = " + str(activation_value))
