from math import isclose
import numpy
from numpy.lib.recfunctions import structured_to_unstructured, require_fields
import pandas as pd
from sklearn import svm
import tensorflow as tf
from rclpy.node import Node

class Space(object):
    """A n-dimensional state space."""

    def __init__(self, ident=None, **kwargs):
        """Init attributes when a new object is created

        :param ident: The name of the space
        :type ident: str
        """
        self.ident = ident
        self.parent_space = None


class PointBasedSpace(Space):
    """A state space based on points."""

    def __init__(self, size=5000, **kwargs):
        """
        Init attributes when a new object is created

        :param size: Maximum number of points that the space can contain, defaults to 5000
        :type size: int
        """
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

        :param perception: The perception that sizes the structured array
        :type perception: dict
        :param base_dtype: The dtype of the structured array
        :type base_dtype: numpy.dtype
        :param size: The size of the structured array
        :type size: int
        :return: The structured array, filled with zeros
        :rtype: numpy.ndarray
        """
        if getattr(perception, "dtype", None):
            if base_dtype:
                types = [(name, float) for name in perception.dtype.names if name in base_dtype.names]
            elif self.parent_space:
                types = [
                    (name, float)
                    for name in perception.dtype.names
                    if name not in self.parent_space.members.dtype.names
                ]
            else:
                types = perception.dtype
        else:
            if base_dtype:
                types = [
                    (sensor + "_" + attribute, float)
                    for sensor, attributes in perception.items()
                    for attribute in attributes
                    if sensor + "_" + attribute in base_dtype.names
                ]
            elif self.parent_space:
                types = [
                    (sensor + "_" + attribute, float)
                    for sensor, attributes in perception.items()
                    for attribute in attributes
                    if sensor + "_" + attribute not in self.parent_space.members.dtype.names
                ]
            else:
                types = [
                    (sensor + "_" + attribute, float)
                    for sensor, attributes in perception.items()
                    for attribute in attributes
                ]
        return numpy.zeros(size, dtype=types)

    @staticmethod
    def copy_perception(space, position, perception):
        """
        Copy a perception to a structured array.

        :param space: An structured array, filled with zeros
        :type space: numpy.ndarray
        :param position: Position of the array in which the perception is added
        :type position: int
        :param perception: The perception thah is copied in the structured array
        :type perception: dict
        """
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

    @staticmethod
    def get_closest_point_and_antipoint_info(members, memberships, foreigner):
        """
        Obtain info about the closest point and antipoint for a given foreigner

        :param members: Set of the points and antipoints
        :type members: numpy.ndarray
        :param memberships: The confidence of the points contained in members
        :type memberships: numpy.ndarray
        :param foreigner: The given foreigner point in order to obtain the info
        :type foreigner: numpy.ndarray
        :return: The position of in the members array the closest point and antipoins and 
        their distance with the foreigner point
        :rtype: int (position), float (distance)
        """
        distances = numpy.linalg.norm(members - foreigner, axis=1)
        closest_point_pos = None
        closest_point_dist = numpy.finfo(float).max
        closest_antipoint_pos = None
        closest_antipoint_dist = numpy.finfo(float).max
        for pos, _ in enumerate(members):
            if memberships[pos] > 0.0:
                if distances[pos] < closest_point_dist:
                    closest_point_pos = pos
                    closest_point_dist = distances[pos]
            else:
                if distances[pos] < closest_antipoint_dist:
                    closest_antipoint_pos = pos
                    closest_antipoint_dist = distances[pos]
        return closest_point_pos, closest_point_dist, closest_antipoint_pos, closest_antipoint_dist

    def specialize(self, space=None):
        """
        Return a new space with those fields that are in r"space" and not in r"self".

        :param space: Space used to specialize
        :type space: cognitive_nodes.space
        :return: The new space
        :rtype: cognitive_nodes.space
        """
        new_space = type(self)()
        new_space.parent_space = self
        if space:
            new_space.add_point(space, 1.0)
        return new_space

    def add_point(self, perception, confidence):
        """
        Add a new point to the p-node.

        :param perception: A given perception to add
        :type perception: dict
        :param confidence: The confidence of the added point that specifies if it is a point or an 
        antipoint
        :type confidence: float
        :raises RuntimeError: If LTM operation cannot continue
        :return: The position of the added point
        :rtype: int
        """
        added_point_pos = -1
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
                Node.get_logger().error(
                    "Trying to add a perception to a NOT compatible space!!!"
                    "Please, take into account that, at the present time, sensor order in perception matters!!!"
                    )
                raise RuntimeError("LTM operation cannot continue :-(")
            else:
                # Copy the new perception on the structured array
                self.copy_perception(candidate_point, 0, perception)
                # Store the new perception if there is a place for it
                if self.size < self.real_size:
                    self.members[self.size] = candidate_point
                    self.memberships[self.size] = confidence
                    added_point_pos = self.size
                    self.size += 1
                else:
                    # Points should be replaced when the P-node is full (may be some metric based on number of times
                    # involved in get_probability)
                    Node.get_logger().debug(self.ident + " full!")
                    raise RuntimeError("LTM operation cannot continue :-(")
        return added_point_pos

    def get_probability(self, perception):
        """
        Calculate the new activation value.

        :param perception: The given perception to calculate the activation
        :type perception: dict
        :raises NotImplementedError: The method has to be implemented in a child class
        """
        raise NotImplementedError

    def contains(self, space, threshold=0.9):
        """
        Check if other space is contained inside this one.

        That happens if this space has a given value of probability for every point belonging to the other space.

        :param space: Space that is checked if it is included
        :type space: cognitive_nodes.space
        :param threshold: Minimum probability value
        :type threshold: float
        :return: Indicates whether the space is contained or not
        :rtype: bool
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
        """
        Check if other space has exactly the same sensors that this one.

        :param space: The space to check
        :type space: cognitive_nodes.space
        :return: Indicates whether the space has the same sensors or not
        :rtype: bool
        """
        answer = False
        if self.size and space.size:
            types = [name for name in space.members.dtype.names if name in self.members.dtype.names]
            if len(types) == len(self.members.dtype.names) == len(space.members.dtype.names):
                answer = True
        return answer

    def prune(self, space):
        """
        Prune sensors that are present only in this space or in the space given for comparison.

        :param space: The given space
        :type space: cognitive_nodes.space
        """
        common_sensors = [(name, float) for name in self.members.dtype.names if name in space.members.dtype.names]
        self.members = require_fields(self.members, common_sensors)

    def aging(self):
        """
        Move towards zero the activation for every point or anti-point.
        """
        for i in range(self.size):
            if self.memberships[i] > 0.0:
                self.memberships[i] -= 0.001
            elif self.memberships[i] < 0.0:
                self.memberships[i] += 0.001
            # This is ugly as it can lead to holes (points that are not really points or antipoints any longer)
            # If this works well, an index structure to reuse these holes should be implemented.
            if numpy.isclose(self.memberships[i], 0.0):
                self.memberships[i] = 0.0


class ClosestPointBasedSpace(PointBasedSpace):
    """
    Calculate the new activation value.

    This activation value is for a given perception and it is calculated as follows:
    - Calculate the closest point to the new point.
    - If the closest point has a positive membership, the membership of the new point is that divided by the distance
    between them. Otherwise, the activation is -1.
    """

    def get_probability(self, perception):
        """
        Calculate the new activation value.

        :param perception: The given perception to calculate the activation
        :type perception: dict
        :return: The activation value
        :rtype: float
        """
        # Create a new structured array for the new perception
        candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
        # Copy the new perception on the structured array
        self.copy_perception(candidate_point, 0, perception)
        # Create views on the structured arrays so they can be used in calculations
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
        """
        Calculate the new activation value.

        :param perception: The given perception to calculate the activation
        :type perception: dict
        :return: The activation value
        :rtype: float
        """
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
        """
        Calculate the new activation value.

        :param perception: The given perception to calculate the activation
        :type perception: dict
        :return: The activation value
        :rtype: float
        """
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


class SVMSpace(PointBasedSpace):
    """
    Use a SVM to calculate activations.
    """

    def __init__(self, **kwargs):
        """
        Init attributes when a new object is created.
        """
        self.model = svm.SVC(kernel="poly", degree=32, max_iter=200000)
        super().__init__(**kwargs)

    def learnable(self):
        """
        Only antipoints are considered learnables

        :return: Return if the perception (point) is learnable or not
        :rtype: bool
        """
        for i in self.memberships[0 : self.size]:
            if numpy.isclose(i, -1.0):
                return True
        return False

    def prune_points(self, score, memberships):
        """
        Prune points depending on the model score obtained

        :param score: Score that determines the pruning
        :type score: float
        :param memberships: The confidence of the points
        :type memberships: numpy.ndarray
        """
        if numpy.isclose(score, 1.0):
            self.size = len(self.model.support_vectors_)
            for i, vector in zip(self.model.support_, self.model.support_vectors_):
                self.members[i] = tuple(vector)
                self.memberships[i] = memberships[i]

    def fit_and_score(self):
        """
        Fit and score the SVM Model

        :return: The score of the model
        :rtype: float
        """
        members = structured_to_unstructured(
            self.members[0 : self.size][list(self.members.dtype.names)]
        )
        memberships = self.memberships[0 : self.size].copy()
        memberships[memberships > 0] = 1
        memberships[memberships <= 0] = 0
        self.model.fit(members, memberships)
        score = self.model.score(members, memberships)
        Node.get_logger().logdebug(
            "SVM: iterations "
            + str(self.model.n_iter_)
            + " support vectors "
            + str(len(self.model.support_vectors_))
            + " score "
            + str(score)
            + " points "
            + str(len(members))
        )
        return score

    def remove_close_points(self):
        """
        Remove points that are too close in space
        """
        threshold = 0
        previous_size = self.size
        members = self.members[0 : self.size].copy()
        umembers = structured_to_unstructured(members[list(self.members.dtype.names)])
        memberships = self.memberships[0 : self.size].copy()
        score = 0.3
        while score < 1.0:
            threshold += 0.1
            distances = numpy.linalg.norm(umembers - umembers[previous_size - 1], axis=1)
            indexes = distances > threshold
            filtered_members = members[indexes]
            filtered_memberships = memberships[indexes]
            self.size = len(filtered_members)
            if self.size < previous_size - 1:
                for i in range(self.size):
                    self.members[i] = filtered_members[i]
                    self.memberships[i] = filtered_memberships[i]
                self.members[self.size] = members[previous_size - 1]
                self.memberships[self.size] = memberships[previous_size - 1]
                self.size += 1
                score = self.fit_and_score()
        Node.get_logger.logdebug(
            self.ident + ": throwing away " + str(previous_size - self.size) + " points."
        )

    def add_point(self, perception, confidence):
        """
        Add a new point to the p-node.

        :param perception: A given perception to add
        :type perception: dict
        :param confidence: The confidence of the added point that specifies if it is a point or an 
        antipoint
        :type confidence: float
        :return: The position of the added point
        :rtype: int
        """
        pos = super().add_point(perception, confidence)
        if self.learnable():
            self.fit_and_score()
        prediction = self.get_probability(perception)
        if ((confidence > 0.0) and (prediction <= 0.0)) or (
            (confidence <= 0.0) and (prediction > 0.0)
        ):
            if self.fit_and_score() < 1.0:
                self.remove_close_points()
        return pos

    def get_probability(self, perception):
        """
        Calculate the new activation value.

        :param perception: The given perception to calculate the activation
        :type perception: dict
        :return: The activation value
        :rtype: float
        """
        # Create a new structured array for the new perception
        candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
        # Copy the new perception on the structured array
        self.copy_perception(candidate_point, 0, perception)
        # Create views on the structured arrays so they can be used in calculations
        # Beware, if candidate_point.dtype is not equal to self.members.dtype, members is a new array!
        point = structured_to_unstructured(candidate_point)
        # Calculate the activation value
        if self.learnable():
            act = min(2.0, self.model.decision_function(point)[0]) / 2.0
        else:
            act = 1.0
        return min(act, self.parent_space.get_probability(perception)) if self.parent_space else act


class ANNSpace(PointBasedSpace):
    """
    Use and train a Neural Network to calculate the activations
    """
    def __init__(self, **kwargs):
        """
        Init attributes when a new object is created.
        """

        #Define train values
        output_activation = 'sigmoid'
        optimizer = tf.optimizers.Adam()
        loss = tf.losses.BinaryCrossentropy()
        metrics=['accuracy']
        # self.n_splits = 5
        self.batch_size = 50
        self.epochs = 50
        self.max_data = 400
        self.first_data = 0
        #Define the Neural Network's model
        self.model = tf.keras.Sequential([
            tf.keras.layers.Dense(128, activation='relu', input_shape =(8,)),
            tf.keras.layers.Dense(64, activation='relu'),
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dense(1, activation=output_activation)
        ])

        #Compile the model
        self.model.compile(optimizer=optimizer, loss=loss, metrics=metrics)

        #Initialize variables
        self.there_are_points = False
        self.there_are_antipoints = False
        super().__init__(**kwargs)

    def add_point(self, perception, confidence):
        """
        Add a new point to the p-node.

        :param perception: A given perception to add
        :type perception: dict
        :param confidence: The confidence of the added point that specifies if it is a point or an 
        antipoint
        :type confidence: float
        :return: The position of the added point
        :rtype: int
        """
        pos = None

        if confidence > 0.0:
            self.there_are_points = True
        else:
            self.there_are_antipoints = True

        if self.there_are_points and self.there_are_antipoints:
            candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
            self.copy_perception(candidate_point, 0, perception)
            point = tf.convert_to_tensor(structured_to_unstructured(candidate_point))
            prediction = self.model.call(point)[0][0]
            pos = super().add_point(perception, confidence)
            
            members = structured_to_unstructured(self.members[0 : self.size][list(self.members.dtype.names)])
            memberships = self.memberships[0 : self.size].copy()
            memberships[memberships > 0] = 1.0
            memberships[memberships <= 0] = 0.0

            prediction = 1.0 if prediction >= 0.5 else 0.0

            if self.size >= self.max_data:
                self.first_data = self.size - self.max_data
        
            if (((confidence <= 0 and prediction == 1.0) or (confidence >= 0 and prediction == 0.0))):
                Node.get_logger().logdebug(f"Training... {self.ident}")
                X = members[self.first_data : self.size]
                Y = memberships[self.first_data : self.size]
                n_0 = int(len(Y[Y == 0.0]))
                n_1 = int(len(Y[Y == 1.0]))
                weight_for_0 = (1 / n_0) * ((self.size-self.first_data) / 2.0) if n_0 != 0 else 1.0
                weight_for_1 = (1 / n_1) * ((self.size-self.first_data) / 2.0) if n_1 != 0 else 1.0
                class_weight = {0: weight_for_0, 1: weight_for_1}
                self.model.fit(x=X, y=Y, batch_size=self.batch_size, epochs=self.epochs, verbose = 0, class_weight=class_weight)

        else:
            pos = super().add_point(perception, confidence)
        
        return pos
        
    def get_probability(self, perception):
        """
        Calculate the new activation value.

        :param perception: The given perception to calculate the activation
        :type perception: dict
        :return: The activation value
        :rtype: float
        """
        candidate_point = self.create_structured_array(perception, self.members.dtype, 1)
        self.copy_perception(candidate_point, 0, perception)
        point = tf.convert_to_tensor(structured_to_unstructured(candidate_point))
        if self.there_are_points:
            if self.there_are_antipoints:
                act = self.model.call(point)[0][0]
                act = 1.0 if act >= 0.5 else 0.0
            else:
                act = 1.0
        else:
            act = 0.0
        return min(act, self.parent_space.get_probability(perception)) if self.parent_space else act   

