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
from numpy.lib.recfunctions import structured_to_unstructured
import pandas as pd
import tensorflow as tf
import rospy


class Space(object):
    """A n-dimensional state space."""

    def __init__(self, ident=None, **kwargs):
        """Init attributes when a new object is created."""
        self.ident = ident
        self.parent_space = None


class PointBasedSpace(Space):
    """A state space based on points."""

    def __init__(self, size=5000, **kwargs):
        """Init attributes when a new object is created."""
        self.real_size = size
        self.size = 0
        # These lists must be empty, not None, in order loops correctly operate with empty spaces.
        self.members = []
        self.memberships = []
        super().__init__(**kwargs)

    def create_structured_array(self, perception, base_dtype, size):
        """
        Create a structured array to store points.

        The key is what fields to use. There are three cases:
        - If base_dtype is specified, use the fields in perception that are also in base_dtype.
        - Otherwise, if this space is a specialization, use the fields in perception that are NOT in parent_space.
        - Otherwise, use every field in perception.

        """
        if getattr(perception, "dtype", None):
            if base_dtype:
                types = [(name, numpy.float) for name in perception.dtype.names if name in base_dtype.names]
            elif self.parent_space:
                types = [
                    (name, numpy.float)
                    for name in perception.dtype.names
                    if name not in self.parent_space.members.dtype.names
                ]
            else:
                types = perception.dtype
        else:
            if base_dtype:
                types = [
                    (sensor + "_" + attribute, numpy.float)
                    for sensor, attributes in perception.items()
                    for attribute in attributes
                    if sensor + "_" + attribute in base_dtype.names
                ]
            elif self.parent_space:
                types = [
                    (sensor + "_" + attribute, numpy.float)
                    for sensor, attributes in perception.items()
                    for attribute in attributes
                    if sensor + "_" + attribute not in self.parent_space.members.dtype.names
                ]
            else:
                types = [
                    (sensor + "_" + attribute, numpy.float)
                    for sensor, attributes in perception.items()
                    for attribute in attributes
                ]
        return numpy.zeros(size, dtype=types)

    @staticmethod
    def copy_perception(space, position, perception):
        """Copy a perception to a structured array."""
        if getattr(perception, "dtype", None):
            for name in perception.dtype.names:
                if name in space.dtype.names:
                    space[position][name] = perception[name]
        else:
            for sensor, attributes in perception.items():
                for attribute, value in attributes.items():
                    name = sensor + "_" + attribute
                    if name in space.dtype.names:
                        space[position][name] = value

    def specialize(self, space=None):
        """Return a new space with those fields that are in r"space" and not in r"self"."""
        new_space = type(self)()
        new_space.parent_space = self
        if space:
            new_space.add_point(space, 1.0)
        return new_space

    def add_point(self, perception, confidence):
        """Add a new point to the p-node."""
        added_point = added_point_confidence = delete_point = delete_point_confidence = None
        # Currently, we don't add the point if it is an anti-point and the space does not activate for it.
        if (confidence > 0.0) or (self.get_probability(perception) > 0.0):
            if self.parent_space:
                self.parent_space.add_point(perception, confidence)
            # Check if we need to initialize the structured numpy array for storing points
            if self.size == 0:
                # This first point's dtype sets the space's dtype
                # In order to relax this restriction, we will probably replace structured arrays with xarrays
                self.members = self.create_structured_array(perception, None, self.real_size)
                self.memberships = numpy.zeros(self.real_size)
            # Create a new structured array for the new perception
            candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
            # Check if the perception is compatible with this space
            if self.members.dtype != candidate_point.dtype:
                rospy.logerr(
                    "Trying to add a perception to a NOT compatible space!!!"
                    "Please, take into account that, at the present time, sensor order in perception matters!!!"
                )
                raise RuntimeError("LTM operation cannot continue :-(")
            else:
                # Copy the new perception on the structured array
                self.copy_perception(candidate_point, 0, perception)
                # Create views on the structured arrays so they can be used in calculations
                # This is for Numpy >= 1.16
                # The old way of doing this would be:
                # members = self.members[0 : self.size].view(numpy.float).reshape(-1, len(self.members[0]))
                members = structured_to_unstructured(self.members[0 : self.size])
                point = structured_to_unstructured(candidate_point)
                # Sanity check: checking if we are adding twice the same anti-point
                if self.size > 0:
                    pos_closest = numpy.argmin(numpy.linalg.norm(members - point, axis=1))
                    closest = members[pos_closest]
                    if numpy.linalg.norm(point - closest) == 0.0:
                        if confidence <= 0.0:
                            rospy.logwarn("Trying to add twice the same anti-point")
                # Store the new perception if there is a place for it
                if self.size < self.real_size:
                    self.members[self.size] = candidate_point
                    self.memberships[self.size] = confidence
                    self.size += 1
                else:
                    delete_point = structured_to_unstructured(self.members[pos_closest])[0]
                    delete_point_confidence = self.memberships[pos_closest]
                    self.members[pos_closest] = candidate_point
                    self.memberships[pos_closest] = confidence
                    rospy.logdebug(self.ident + " full!")
                added_point = point[0]
                added_point_confidence = confidence
        return added_point, added_point_confidence, delete_point, delete_point_confidence

    def get_probability(self, perception):
        """Calculate the new activation value."""
        raise NotImplementedError

    def contains(self, space, threshold=0.9):
        """
        Check if other space is contained inside this one.

        That happens if this space has a given value of probability for every point belonging to the other space.
        """
        contained = False
        if space.size:
            contained = True
            for point in space.members[0 : space.size]:
                if self.get_probability(point) < threshold:
                    contained = False
                    break
        return contained

    def same_sensors(self, space):
        """Check if other space has exactly the same sensors that this one."""
        answer = False
        if space.size:
            types = [name for name in space.members.dtype.names if name in self.members.dtype.names]
            if len(types) == len(self.members.dtype.names) == len(space.members.dtype.names):
                answer = True
        return answer


class ClosestPointBasedSpace(PointBasedSpace):
    """
    Calculate the new activation value.

    This activation value is for a given perception and it is calculated as follows:
    - Calculate the closest point to the new point.
    - If the closest point has a positive membership, the membership of the new point is that divided by the distance
    between them. Otherwise, the activation is -1.
    """

    def get_probability(self, perception):
        """Calculate the new activation value."""
        # Create a new structured array for the new perception
        candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
        # Copy the new perception on the structured array
        self.copy_perception(candidate_point, 0, perception)
        # Create views on the structured arrays so they can be used in calculations
        # Be ware, if candidate_point.dtype is not equal to self.members.dtype, members is a new array!!!
        members = structured_to_unstructured(self.members[0 : self.size][list(candidate_point.dtype.names)])
        point = structured_to_unstructured(candidate_point)
        memberships = self.memberships[0 : self.size]
        # Calculate the activation value
        distances = numpy.linalg.norm(members - point, axis=1)
        pos_closest = numpy.argmin(distances)
        if memberships[pos_closest] > 0.0:
            activation = memberships[pos_closest] / (distances[pos_closest] + 1.0)
        else:
            activation = -1
        return min(activation, self.parent_space.get_probability(perception)) if self.parent_space else activation


class CentroidPointBasedSpace(PointBasedSpace):
    """
    Calculate the new activation value.

    This activation value is for a given perception and it is calculated as follows:
    - Calculate the closest point to the new point.
    - If the closest point has a positive membership, the membership of the new point is that divided by the distance
    between them.
    - Otherwise:
        - Calculate the centroid of points with a positive membership.
        - If the distance from the new point to the centroid is less than the distance from the closest point to the
        centroid, then the activation is calculated as before but using the closest point with positive
        membership. Otherwise the activation is -1.
    """

    def get_probability(self, perception):
        """Calculate the new activation value."""
        # Create a new structured array for the new perception
        candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
        # Copy the new perception on the structured array
        self.copy_perception(candidate_point, 0, perception)
        # Create views on the structured arrays so they can be used in calculations
        # Be ware, if candidate_point.dtype is not equal to self.members.dtype, members is a new array!!!
        members = structured_to_unstructured(self.members[0 : self.size][list(candidate_point.dtype.names)])
        point = structured_to_unstructured(candidate_point)
        memberships = self.memberships[0 : self.size]
        # Calculate the activation value
        distances = numpy.linalg.norm(members - point, axis=1)
        pos_closest = numpy.argmin(distances)
        if memberships[pos_closest] > 0.0:
            activation = memberships[pos_closest] / (distances[pos_closest] + 1.0)
        else:
            centroid = numpy.mean(members[memberships > 0.0], axis=0)
            dist_antipoint_centroid = numpy.linalg.norm(members[pos_closest] - centroid)
            dist_newpoint_centroid = numpy.linalg.norm(point - centroid)
            if dist_newpoint_centroid + 0.000001 < dist_antipoint_centroid:
                distances = distances[memberships > 0.0]
                pos_closest = numpy.argmin(distances)
                memberships = memberships[memberships > 0.0]
                activation = memberships[pos_closest] / (distances[pos_closest] + 1.0)
            else:
                activation = -1
        return min(activation, self.parent_space.get_probability(perception)) if self.parent_space else activation


class NormalCentroidPointBasedSpace(PointBasedSpace):
    """
    Calculate the new activation value.

    This activation value is for a given perception and it is calculated as follows:
    - Calculate the closest point to the new point.
    - If the closest point has a positive membership, the membership of the new point is that divided by the distance
    between them.
    - Otherwise:
        - Calculate the centroid of points with a positive membership.
        - If the distance from the new point to the centroid is less than the distance from the closest point to the
        centroid, or the distance of the closest point to the line that goes from the new point to the centroid is high
        (see source code), then the activation is calculated as before but using the closest point with positive
        membership, otherwise the activation is -1.
    """

    def get_probability(self, perception):
        """Calculate the new activation value."""
        # Create a new structured array for the new perception
        candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
        # Copy the new perception on the structured array
        self.copy_perception(candidate_point, 0, perception)
        # Create views on the structured arrays so they can be used in calculations
        # Be ware, if candidate_point.dtype is not equal to self.members.dtype, members is a new array!!!
        members = structured_to_unstructured(self.members[0 : self.size][list(candidate_point.dtype.names)])
        point = structured_to_unstructured(candidate_point)
        memberships = self.memberships[0 : self.size]
        # Calculate the activation value
        distances = numpy.linalg.norm(members - point, axis=1)
        pos_closest = numpy.argmin(distances)
        if memberships[pos_closest] > 0.0:
            activation = memberships[pos_closest] / (distances[pos_closest] + 1.0)
        else:
            centroid = numpy.mean(members[memberships > 0.0], axis=0)
            v_antipoint_centroid = numpy.ravel(members[pos_closest] - centroid)
            v_newpoint_centroid = numpy.ravel(point - centroid)
            dist_antipoint_centroid = numpy.linalg.norm(v_antipoint_centroid)
            dist_newpoint_centroid = numpy.linalg.norm(v_newpoint_centroid)
            # https://en.wikipedia.org/wiki/Vector_projection
            separation = numpy.linalg.norm(
                v_antipoint_centroid
                - numpy.inner(
                    v_newpoint_centroid,
                    numpy.inner(v_antipoint_centroid, v_newpoint_centroid)
                    / numpy.inner(v_newpoint_centroid, v_newpoint_centroid),
                )
            )
            if (dist_newpoint_centroid < dist_antipoint_centroid) or (
                numpy.random.uniform() < dist_antipoint_centroid * separation / dist_newpoint_centroid
            ):
                distances = distances[memberships > 0.0]
                pos_closest = numpy.argmin(distances)
                memberships = memberships[memberships > 0.0]
                activation = memberships[pos_closest] / (distances[pos_closest] + 1.0)
            else:
                activation = -1
        return min(activation, self.parent_space.get_probability(perception)) if self.parent_space else activation


class DNNSpace(Space):
    """Calculate the new activation value using the output of a DNN."""

    def __init__(self, **kwargs):
        """Initialize."""
        self.headers = [
            "ball_in_right_hand",
            "ball_dist",
            "box_size",
            "ball_in_left_hand",
            "box_ang",
            "ball_ang",
            "box_dist",
            "ball_size",
        ]
        self.feature_columns = [tf.feature_column.numeric_column(i) for i in self.headers]
        self.net = tf.estimator.DNNClassifier(
            hidden_units=[20, 10, 5],
            feature_columns=self.feature_columns,
            model_dir="pnode00_03000",
            n_classes=2,
            optimizer="Adam",
            activation_fn=tf.nn.relu,
        )
        super(DNNSpace, self).__init__(**kwargs)

    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""

    def get_probability(self, perception):
        """Calculate the new activation value."""
        predict_data = pd.DataFrame([perception], columns=self.headers)
        predict_input_fn = tf.estimator.inputs.pandas_input_fn(
            x=predict_data, batch_size=1, num_epochs=1, shuffle=False, target_column="Confidence"
        )
        predictions = list(self.net.predict(input_fn=predict_input_fn))
        prediction = predictions[0]
        if prediction["class_ids"] == 1:
            activation = prediction["probabilities"][1]
        else:
            activation = -1.0
        return min(activation, self.parent_space.get_probability(perception)) if self.parent_space else activation
