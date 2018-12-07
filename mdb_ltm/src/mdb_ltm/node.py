"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import *
import rospy


class Node(object):
    """
    An MDB element.

    Attributes:

    """

    def __init__(self, ident, node_type, threshold=0.1, ltm=None, **kwargs):
        """Constructor."""
        self.ident = ident
        self.type = node_type
        self.activation = 0.0
        self.threshold = threshold
        self.neighbors = []
        self.ltm = ltm
        super(Node, self).__init__()

    @staticmethod
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition('.')
        node_module = __import__(module_string, fromlist=[class_string])
        node_class = getattr(node_module, class_string)
        return node_class

    def update_activation(self, **kwargs):
        """
        Calculate the new activation value.

        This method contains only the common stuff for every type of node.
        """
        if self.activation < self.threshold:
            self.activation = 0.0
        rospy.logdebug(self.type + ' activation for ' + self.ident + ' = ' + str(self.activation))
        super(Node, self).__init__(**kwargs)
