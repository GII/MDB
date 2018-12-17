"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import numpy
import rospy
import pandas as pd
import tensorflow as tf
from mdb_ltm.node import Node


class PNode(Node):
    """
    A subspace of the input space (sensorial or result of a redescription process).

    This subspace is linked to every node for which it is relevant,
    activating them when a new perception pertaining to this subspace occurs.
    """

    def __init__(self, size=5000, n_perceptions=8, **kwargs):
        """Constructor."""
        self.real_size = size
        self.n_perceptions = n_perceptions
        self.members = numpy.matrix(numpy.zeros((self.n_perceptions, self.real_size)))
        self.memberships = numpy.zeros(self.real_size)
        self.size = 0
        super(PNode, self).__init__(**kwargs)

    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""
        newpoint = numpy.matrix(perception).T
        if self.size > 0:
            distances_to_newpoint = numpy.linalg.norm(self.members[:, 0 : self.size] - newpoint, axis=0)
            pos_closest = numpy.argmin(distances_to_newpoint)
            closest = self.members[:, pos_closest]
            if numpy.linalg.norm(newpoint - closest) == 0.0:
                if confidence <= 0.0:
                    rospy.logerr("Adding twice the same anti-point, this should not happen ever!!!")
        if self.size < self.real_size:
            self.members[:, self.size] = newpoint
            self.memberships[self.size] = confidence
            self.size += 1
        else:
            self.members[:, pos_closest] = newpoint
            self.memberships[pos_closest] = confidence
            rospy.logdebug("p-node " + self.ident + "full!")


class PNodeActClosest(PNode):
    """
    Calculate the new activation value.

    This activation value is for a given perception and it is calculated as follows:
    - Calculate the closest point to the new point.
    - If the closest point has a positive membership, the membership of the new point is that divided by the distance
    between them. Otherwise, the activation is -1.
    """

    def update_activation(self, perception=None, **kwargs):
        """Calculate the new activation value."""
        if perception is None:
            rospy.logerr("No data when updating the activation of a p-node. This should not happen!")
            self.activation = 0.0
        else:
            members = self.members[:, 0 : self.size]
            memberships = self.memberships[0 : self.size]
            newpoint = numpy.matrix(perception).T
            distances_to_newpoint = numpy.linalg.norm(members - newpoint, axis=0)
            pos_closest = numpy.argmin(distances_to_newpoint)
            if memberships[pos_closest] > 0.0:
                self.activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
            else:
                self.activation = -1
        super(PNodeActClosest, self).update_activation(**kwargs)


class PNodeActCentroid(PNode):
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

    def update_activation(self, perception=None, **kwargs):
        """Calculate the new activation value."""
        if perception is None:
            rospy.logerr("No data when updating the activation of a p-node. This should not happen!")
            self.activation = 0.0
        else:
            members = self.members[:, 0 : self.size]
            memberships = self.memberships[0 : self.size]
            newpoint = numpy.matrix(perception).T
            distances_to_newpoint = numpy.linalg.norm(members - newpoint, axis=0)
            pos_closest = numpy.argmin(distances_to_newpoint)
            closest = members[:, pos_closest]
            if memberships[pos_closest] > 0.0:
                self.activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
            else:
                points = members[:, memberships > 0.0]
                centroid = numpy.mean(points, axis=1)
                antipoint_centroid = closest - centroid
                newpoint_centroid = newpoint - centroid
                dist_antipoint_centroid = numpy.linalg.norm(antipoint_centroid)
                dist_newpoint_centroid = numpy.linalg.norm(newpoint_centroid)
                if dist_newpoint_centroid + 0.000001 < dist_antipoint_centroid:
                    distances_to_newpoint = distances_to_newpoint[memberships > 0.0]
                    pos_closest = numpy.argmin(distances_to_newpoint)
                    positive_memberships = memberships[memberships > 0.0]
                    self.activation = positive_memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
                else:
                    self.activation = -1
        super(PNodeActCentroid, self).update_activation(**kwargs)


class PNodeActCentroidAndNormal(PNode):
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

    # TODO: this is not efficient because we are operating with real_size instead of size
    # TODO: transpose matrices

    def update_activation(self, perception=None, **kwargs):
        """Calculate the new activation value."""
        if perception is None:
            rospy.logerr("No data when updating the activation of a p-node. This should not happen!")
            self.activation = 0.0
        else:
            members = self.members[:, 0 : self.size]
            memberships = self.memberships[0 : self.size]
            newpoint = numpy.matrix(perception).T
            distances_to_newpoint = numpy.linalg.norm(members - newpoint, axis=0)
            pos_closest = numpy.argmin(distances_to_newpoint)
            closest = members[:, pos_closest]
            if memberships[pos_closest] > 0.0:
                self.activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
            else:
                points = members[:, memberships > 0.0]
                centroid = numpy.mean(points, axis=1)
                v_antipoint_centroid = numpy.ravel(closest - centroid)
                v_newpoint_centroid = numpy.ravel(newpoint - centroid)
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
                ):  # yapf: disable
                    distances_to_newpoint = distances_to_newpoint[memberships > 0.0]
                    pos_closest = numpy.argmin(distances_to_newpoint)
                    positive_memberships = memberships[memberships > 0.0]
                    self.activation = positive_memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
                else:
                    self.activation = -1
        super(PNodeActCentroidAndNormal, self).update_activation(**kwargs)


class PNodeDNN(PNode):
    """Calculate the new activation value using the output of a DNN."""

    def __init__(self, **kwargs):
        """Constructor."""
        # TODO: almost everything is hardcoded for today
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
        super(PNodeDNN, self).__init__(**kwargs)

    def add_perception(self, perception, confidence):
        """Add a new point to the p-node."""
        pass

    def update_activation(self, perception=None, **kwargs):
        """Calculate the new activation value."""
        if perception is None:
            rospy.logerr("No data when updating the activation of a p-node. This should not happen!")
            self.activation = 0.0
        else:
            predict_data = pd.DataFrame([perception], columns=self.headers)
            predict_input_fn = tf.estimator.inputs.pandas_input_fn(
                x=predict_data, batch_size=1, num_epochs=1, shuffle=False, target_column="Confidence"
            )
            predictions = list(self.net.predict(input_fn=predict_input_fn))
            prediction = predictions[0]
            if prediction["class_ids"] == 1:
                self.activation = prediction["probabilities"][1]
            else:
                self.activation = -1.0
        super(PNodeDNN, self).update_activation(**kwargs)
