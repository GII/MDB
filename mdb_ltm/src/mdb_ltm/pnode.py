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
        super().__init__(**kwargs)

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

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        raise NotImplementedError

    def update_activation(self, **kwargs):
        """Calculate a new activation value for each perception."""
        kwargs["members"] = self.members[:, 0 : self.size]
        kwargs["memberships"] = self.memberships[0 : self.size]
        super().update_activation(**kwargs)


class PNodeActClosest(PNode):
    """
    Calculate the new activation value.

    This activation value is for a given perception and it is calculated as follows:
    - Calculate the closest point to the new point.
    - If the closest point has a positive membership, the membership of the new point is that divided by the distance
    between them. Otherwise, the activation is -1.
    """

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        members = kwargs.pop("members")
        memberships = kwargs.pop("memberships")
        newpoint = numpy.matrix(kwargs.pop("perception")).T
        distances_to_newpoint = numpy.linalg.norm(members - newpoint, axis=0)
        pos_closest = numpy.argmin(distances_to_newpoint)
        if memberships[pos_closest] > 0.0:
            activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
        else:
            activation = -1
        return activation


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

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        members = kwargs.pop("members")
        memberships = kwargs.pop("memberships")
        newpoint = numpy.matrix(kwargs.pop("perception")).T
        distances_to_newpoint = numpy.linalg.norm(members - newpoint, axis=0)
        pos_closest = numpy.argmin(distances_to_newpoint)
        if memberships[pos_closest] > 0.0:
            activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
        else:
            centroid = numpy.mean(members[:, memberships > 0.0], axis=1)
            dist_antipoint_centroid = numpy.linalg.norm(members[:, pos_closest] - centroid)
            dist_newpoint_centroid = numpy.linalg.norm(newpoint - centroid)
            if dist_newpoint_centroid + 0.000001 < dist_antipoint_centroid:
                distances_to_newpoint = distances_to_newpoint[memberships > 0.0]
                pos_closest = numpy.argmin(distances_to_newpoint)
                memberships = memberships[memberships > 0.0]
                activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
            else:
                activation = -1
        return activation


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

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        members = kwargs.pop("members")
        memberships = kwargs.pop("memberships")
        newpoint = numpy.matrix(kwargs.pop("perception")).T
        distances_to_newpoint = numpy.linalg.norm(members - newpoint, axis=0)
        pos_closest = numpy.argmin(distances_to_newpoint)
        if memberships[pos_closest] > 0.0:
            activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
        else:
            centroid = numpy.mean(members[:, memberships > 0.0], axis=1)
            v_antipoint_centroid = numpy.ravel(members[:, pos_closest] - centroid)
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
            ):
                distances_to_newpoint = distances_to_newpoint[memberships > 0.0]
                pos_closest = numpy.argmin(distances_to_newpoint)
                memberships = memberships[memberships > 0.0]
                activation = memberships[pos_closest] / (distances_to_newpoint[pos_closest] + 1.0)
            else:
                activation = -1
        return activation


class PNodeDNN(PNode):
    """Calculate the new activation value using the output of a DNN."""

    def __init__(self, **kwargs):
        """Constructor."""
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

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        perception = kwargs.pop("perception")
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
        return activation
