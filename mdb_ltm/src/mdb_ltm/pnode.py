"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library
from future.utils import text_to_native_str

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# Library imports
from numpy.lib.recfunctions import structured_to_unstructured

# MDB imports
from mdb_ltm.node import Node


class PNode(Node):
    """
    A subspace of the input space (sensorial or result of a redescription process).

    This subspace is linked to every node for which it is relevant,
    activating them when a new perception pertaining to this subspace occurs.
    """

    def __init__(self, space_class=None, space=None, **kwargs):
        """Initialize."""
        self.space = space if space else self.class_from_classname(space_class)(ident=kwargs.get("ident") + " space")
        super().__init__(**kwargs)

    def publish(self, message=None, first_time=False):
        """Publish node information."""
        message = self.node_message()
        if not isinstance(self.space.members, list):
            message.names = self.space.members.dtype.names
        else:
            message.names = []
        if first_time and not isinstance(message.names, list):
            point_message = self.data_message()
            point_message.command = text_to_native_str("new")
            point_message.id = self.ident
            point_array = structured_to_unstructured(self.space.members)
            confidence_array = structured_to_unstructured(self.space.memberships)
            for point, confidence in zip(point_array, confidence_array):
                point_message.point = point
                point_message.confidence = confidence
                self.data_publisher.publish(point_message)
        super().publish(message, first_time)

    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""
        added_point, added_point_confidence, delete_point, delete_point_confidence = self.space.add_point(
            perception, confidence
        )
        point_message = self.data_message()
        point_message.id = self.ident
        if delete_point:
            point_message.command = text_to_native_str("delete")
            point_message.point = structured_to_unstructured(delete_point)
            point_message.confidence = delete_point_confidence
            self.data_publisher.publish(point_message)
        if added_point:
            point_message.command = text_to_native_str("new")
            point_message.point = structured_to_unstructured(added_point)
            point_message.confidence = added_point_confidence
            self.data_publisher.publish(point_message)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        return self.space.get_probability(perception)
