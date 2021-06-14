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
    This class is migrating to be able to aggregate different input spaces.
    """

    def __init__(self, space_class=None, space=None, **kwargs):
        """Initialize."""
        self.spaces = [space if space else self.class_from_classname(space_class)(ident=kwargs.get("ident") + " space")]
        super().__init__(**kwargs)

    def publish(self, message=None, first_time=False):
        """Publish node information."""
        message = self.node_message()
        space = self.spaces[0]
        if not isinstance(space.members, list):
            message.names = space.members.dtype.names
        else:
            message.names = []
        if first_time and not isinstance(message.names, list):
            point_message = self.data_message()
            point_message.command = text_to_native_str("new")
            point_message.id = self.ident
            point_array = structured_to_unstructured(space.members)
            confidence_array = space.memberships
            for point, confidence in zip(point_array, confidence_array):
                point_message.point = point
                point_message.confidence = confidence
                self.data_publisher.publish(point_message)
        super().publish(message, first_time)

    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""
        added_point, added_point_confidence, delete_point, delete_point_confidence = self.get_space(
            perception
        ).add_point(perception, confidence)
        point_message = self.data_message()
        point_message.id = self.ident
        if delete_point is not None:
            point_message.command = text_to_native_str("delete")
            point_message.point = delete_point
            point_message.confidence = delete_point_confidence
            self.data_publisher.publish(point_message)
        if added_point is not None:
            point_message.command = text_to_native_str("new")
            point_message.point = added_point
            point_message.confidence = added_point_confidence
            self.data_publisher.publish(point_message)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        return self.get_space(perception).get_probability(perception)

    def get_space(self, perception):
        """Return the compatible space with perception."""
        # Ugly hack just to see if this works. In that case, everything need to be checked to reduce the number of
        # conversions between sensing, perception and space.
        temp_space = self.spaces[0].__class__()
        temp_space.add_point(perception, 1.0)
        for space in self.spaces:
            if (not space.size) or space.same_sensors(temp_space):
                return space
        return None