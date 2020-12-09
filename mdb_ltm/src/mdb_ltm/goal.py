"""
MDB.

https://github.com/GII/MDB
"""

# Python 2 compatibility imports
from __future__ import absolute_import, division, print_function, unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *  # noqa pylint: disable=unused-wildcard-import,wildcard-import

# Standard imports
import threading

# Library imports
import numpy
import rospy

# MDB imports
from mdb_simulator.simulator import LTMSim
from mdb_ltm.node import Node


class Goal(Node):
    """A subspace in some input space (sensorial or the result of a redescription) where utility was obtained."""

    def __init__(self, data=None, space_class=None, space=None, **kwargs):
        """Initialize."""
        super().__init__(**kwargs)
        self.new_activation = self.reward = 0.0
        self.embedded = set()
        self.value_functions = set()
        self.start = self.end = self.period = None
        if data:
            self.new_from_configuration_file(data)
        else:
            self.space = (
                space if space else self.class_from_classname(space_class)(ident=kwargs.get("ident") + " space")
            )

    def new_from_configuration_file(self, data):
        """Init attributes with values read from a configuration file."""
        self.space = self.class_from_classname(data.get("space"))(ident=self.ident + " space")
        self.start = data.get("start")
        self.end = data.get("end")
        self.period = data.get("period")
        for point in data.get("points", []):
            self.space.add_point(point, 1.0)

    @property
    def max_activation(self):
        """Return the maximum value of all activations."""
        return numpy.max(self.activation)

    @max_activation.setter
    def max_activation(self, max_activation):
        """Set the same activation value for every perception."""
        self.activation = [max_activation] * len(self.activation)

    def add_value_function(self, path):
        """Create a new value function."""
        self.value_functions.add(tuple(path))

    def exec_value_functions(self, perception=None):
        """
        Execute the value functions.

        This is not in calc_activation() because a goal can affect another goal's activation through its value
        functions, so we need to calculate the activation in two stages, first endogenous activation, then value
        function's generated activation.
        """
        newly_activated_goals = set()
        for value_function in self.value_functions:
            n_goals = len(value_function)
            step = 1 / n_goals
            activation = step
            for goal in value_function:
                if goal.max_activation < activation:
                    if goal.max_activation < goal.threshold:
                        newly_activated_goals.add(goal)
                    goal.max_activation = activation
                activation += step
        return newly_activated_goals

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        if self.end:
            if (self.ltm.iteration % self.period >= self.start) and (self.ltm.iteration % self.period <= self.end):
                self.new_activation = 1.0
            else:
                self.new_activation = 0.0
        return self.new_activation

    def update_embedded(self, p_nodes):
        """Recalculate the list of P-nodes that embed this goal."""
        self.embedded = set()
        for p_node in p_nodes:
            if p_node.space.contains(self.space, threshold=0.5):
                self.embedded.add(p_node)

    def update_success(self):
        """
        Calculate the reward for the current sensor values.

        Currently, this method is not used, as points are only used to check if a goal is inside
        a P-node, and rules are used to check reward. Using points to check reward is very
        problematic. We probably should use sensor ranges.
        """
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for perception, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    reward = activation * self.space.get_probability(perception)
                    if reward > self.reward:
                        self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward

    @staticmethod
    def object_in_close_box(perceptions):
        """Check if there is an object inside of a box."""
        inside = False
        for box in perceptions["boxes"].raw.data:
            if not LTMSim.object_too_far(box.distance, box.angle):
                for cylinder in perceptions["cylinders"].raw.data:
                    inside = (abs(box.distance - cylinder.distance) < 0.05) and (abs(box.angle - cylinder.angle) < 0.05)
                    if inside:
                        break
        return inside

    @staticmethod
    def object_in_far_box(perceptions):
        """Check if there is an object inside of a box."""
        inside = False
        for box in perceptions["boxes"].raw.data:
            if LTMSim.object_too_far(box.distance, box.angle):
                for cylinder in perceptions["cylinders"].raw.data:
                    inside = (abs(box.distance - cylinder.distance) < 0.05) and (abs(box.angle - cylinder.angle) < 0.05)
                    if inside:
                        break
        return inside

    @staticmethod
    def object_with_robot(perceptions):
        """Check if there is an object adjacent to the robot."""
        together = False
        if not Goal.object_held(perceptions):
            for cylinder in perceptions["cylinders"].raw.data:
                dist_near, ang_near = LTMSim.calculate_closest_position(cylinder.angle)
                together = (abs(cylinder.distance - dist_near) < 0.05) and (abs(cylinder.angle - ang_near) < 0.05)
                if together:
                    break
        return together

    @staticmethod
    def object_held_with_left_hand(perceptions):
        """Check if an object is held with the left hand."""
        return perceptions["ball_in_left_hand"].raw.data

    @staticmethod
    def object_held_with_right_hand(perceptions):
        """Check if an object is held with the right hand."""
        return perceptions["ball_in_right_hand"].raw.data

    @classmethod
    def object_held(cls, perceptions):
        """Check if an object is held with one hand."""
        return cls.object_held_with_left_hand(perceptions) or cls.object_held_with_right_hand(perceptions)

    @staticmethod
    def object_held_before(perceptions):
        """Check if an object was held with one hand."""
        return perceptions["ball_in_left_hand"].old_raw.data or perceptions["ball_in_right_hand"].old_raw.data

    @staticmethod
    def object_held_with_two_hands(perceptions):
        """Check if an object is held with two hands."""
        return perceptions["ball_in_left_hand"].raw.data and perceptions["ball_in_right_hand"].raw.data

    @staticmethod
    def ball_and_box_on_the_same_side(perceptions):
        """Check if an object and a box are on the same side."""
        same_side = False
        for box in perceptions["boxes"].raw.data:
            same_side = (perceptions["ball_in_left_hand"].raw.data and box.angle > 0) or (
                perceptions["ball_in_right_hand"].raw.data and box.angle <= 0
            )
            if same_side:
                break
        return same_side

    @staticmethod
    def object_pickable_withtwohands(perceptions):
        """Check if an object can be hold with two hands."""
        pickable = False
        for cylinder in perceptions["cylinders"].raw.data:
            pickable = (not Goal.object_held(perceptions)) and LTMSim.object_pickable_withtwohands(
                cylinder.distance, cylinder.angle
            )
            if pickable:
                break
        return pickable

    @staticmethod
    def object_was_approximated(perceptions):
        """Check if an object was moved towards the robot's reachable area."""
        approximated = False
        for old, cur in zip(perceptions["cylinders"].old_raw.data, perceptions["cylinders"].raw.data):
            approximated = not LTMSim.object_too_far(cur.distance, cur.angle) and LTMSim.object_too_far(
                old.distance, old.angle
            )
            if approximated:
                break
        return approximated

    @staticmethod
    def hand_was_changed(perceptions):
        """Check if the held object changed from one hand to another."""
        return (
            (perceptions["ball_in_left_hand"].raw.data and (not perceptions["ball_in_right_hand"].raw.data))
            and ((not perceptions["ball_in_left_hand"].old_raw.data) and perceptions["ball_in_right_hand"].old_raw.data)
        ) or (
            ((not perceptions["ball_in_left_hand"].raw.data) and perceptions["ball_in_right_hand"].raw.data)
            and (perceptions["ball_in_left_hand"].old_raw.data and (not perceptions["ball_in_right_hand"].old_raw.data))
        )

    @staticmethod
    def food_in_skillet(perceptions):
        """Check if all the needed food is inside the skillet."""
        carrot_inside = eggplant_inside = cabbage_inside = False
        for box in perceptions["boxes"].raw.data:
            if box.color == "skillet":
                for cylinder in perceptions["cylinders"].raw.data:
                    inside = (abs(box.distance - cylinder.distance) < 0.05) and (abs(box.angle - cylinder.angle) < 0.05)
                    if inside:
                        if cylinder.color == "carrot":
                            carrot_inside = True
                        elif cylinder.color == "eggplant":
                            eggplant_inside = True
                        elif cylinder.color == "cabbage":
                            cabbage_inside = True
        return carrot_inside and eggplant_inside and cabbage_inside


class GoalMotiven(Goal):
    """Goal generated (and managed) by MOTIVEN."""

    def __init__(self, **kwargs):
        """Initialize."""
        self.new_activation_event = None
        self.new_reward_event = None
        self.init_threading()
        self.activation_topic = None
        self.activation_message = None
        self.ok_topic = None
        self.ok_message = None
        super().__init__(**kwargs)

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = super().__getstate__()
        del state["new_activation_event"]
        del state["new_reward_event"]
        return state

    def init_threading(self):
        """Create needed stuff to synchronize threads."""
        self.new_activation_event = threading.Event()
        self.new_reward_event = threading.Event()

    def init_ros(self):
        """Create publishers and make subscriptions."""
        super().init_ros()
        # Ugly hack
        if self.ros_data_prefix is None:
            self.ros_data_prefix = "/mdb/goal"
        self.activation_topic = rospy.get_param(self.ros_data_prefix + "_activation_topic")
        self.activation_message = self.class_from_classname(rospy.get_param(self.ros_data_prefix + "_activation_msg"))
        self.ok_topic = rospy.get_param(self.ros_data_prefix + "_ok_topic")
        self.ok_message = self.class_from_classname(rospy.get_param(self.ros_data_prefix + "_ok_msg"))
        rospy.logdebug("Subscribing to %s...", self.activation_topic)
        rospy.Subscriber(self.activation_topic, self.activation_message, callback=self.update_activation_callback)
        rospy.logdebug("Subscribing to %s...", self.ok_topic)
        rospy.Subscriber(self.ok_topic, self.ok_message, callback=self.update_success_callback)

    def update_activation_callback(self, data):
        """Calculate the new activation value."""
        if self.ident == data.id:
            rospy.logdebug("Reading goal activation for " + data.id + " = " + str(data.activation))
            self.new_activation = data.activation
            self.new_activation_event.set()

    def update_success_callback(self, data):
        """Calculate the value for the current sensor values."""
        if self.ident == data.id:
            rospy.logdebug("Reading goal success for " + data.id + " = " + str(data.ok))
            self.reward = data.ok
            self.new_reward_event.set()

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        return self.new_activation

    def update_activation(self, **kwargs):
        """Calculate the new activation value."""
        self.new_activation_event.wait()
        super().update_activation(**kwargs)
        self.new_activation_event.clear()

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.new_reward_event.wait()
        self.new_reward_event.clear()
        return self.reward


# There is a problem with all these classes, as they are calling methods that use perception objects,
# instead of perception values in the way they are stored in the goal objects.
# This needs to be "fixed" as soon as possible.


class GoalObjectHeldLeftHand(Goal):
    """Goal representing a grasped object with the left hand."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_held_with_left_hand(self.ltm.perceptions) and (
                        not Goal.object_held_with_right_hand(self.ltm.perceptions)
                    ):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectHeldRightHand(Goal):
    """Goal representing a grasped object with the right hand."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if (not Goal.object_held_with_left_hand(self.ltm.perceptions)) and Goal.object_held_with_right_hand(
                        self.ltm.perceptions
                    ):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectHeld(Goal):
    """Goal representing a grasped object with one hand."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_held(self.ltm.perceptions) and not Goal.object_held_with_two_hands(
                        self.ltm.perceptions
                    ):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectHeldWithTwoHands(Goal):
    """Goal representing a grasped object with two hands."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_held_with_two_hands(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalChangedHands(Goal):
    """Goal representing a change of the hand that holds an object."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.hand_was_changed(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalFrontalObject(Goal):
    """Goal representing an object in front of the robot."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if (
                        Goal.object_pickable_withtwohands(self.ltm.perceptions)
                        and (not Goal.object_in_close_box(self.ltm.perceptions))
                        and (not Goal.object_with_robot(self.ltm.perceptions))
                    ):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectInCloseBox(Goal):
    """Goal representing an object inside a box (that was reachable)."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_in_close_box(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectWithRobot(Goal):
    """Goal representing an object as close to the robot as possible."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_with_robot(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectInFarBox(Goal):
    """Goal representing an object inside a box (that was out of reach)."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_in_far_box(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalApproximatedObject(Goal):
    """Goal representing an reachable object (that was not reachable previously)."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.object_was_approximated(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalVegetablesInSkillet(Goal):
    """Goal representing three different vegetables in a skillet in front of the robot."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        if self.ltm.sensorial_changes():
            for _, activation in zip(self.perception, self.activation):
                if activation > self.threshold:
                    if Goal.food_in_skillet(self.ltm.perceptions):
                        reward = activation
                        if reward > self.reward:
                            self.reward = reward
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectInBoxStandalone(Goal):
    """Goal representing the desire of putting an object in a box."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        # This is not coherent at all. I need to change it...
        # Or self.activation is not a list any longer...
        # or perceptions should be flattened
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_in_close_box(perceptions) or Goal.object_in_far_box(perceptions):
                    self.reward = 1.0
                elif Goal.object_held(perceptions):
                    if Goal.object_held_with_two_hands(perceptions):
                        self.reward = 0.6
                    elif Goal.ball_and_box_on_the_same_side(perceptions):
                        self.reward = 0.6
                    elif not Goal.object_held_before(perceptions):
                        self.reward = 0.3
                elif not Goal.object_held_before(perceptions):
                    if Goal.object_pickable_withtwohands(perceptions):
                        self.reward = 0.3
                    elif Goal.object_was_approximated(perceptions):
                        self.reward = 0.2
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectWithRobotStandalone(Goal):
    """Goal representing the desire of bringing an object as close as possible to the robot."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_with_robot(perceptions):
                    self.reward = 1.0
                elif Goal.object_held(perceptions):
                    if not Goal.object_held_before(perceptions):
                        self.reward = 0.6
                elif not Goal.object_held_before(perceptions):
                    if Goal.object_pickable_withtwohands(perceptions):
                        self.reward = 0.3
                    elif Goal.object_was_approximated(perceptions):
                        self.reward = 0.2
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward
