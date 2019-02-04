"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import threading
import rospy
from mdb_simulator.simulator import LTMSim
from mdb_ltm.node import Node


class Goal(Node):
    """A subspace in some input space (sensorial or the result of a redescription) where utility was obtained."""

    def __init__(self, data=None, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.new_activation = 0.0
        self.reward = 0.0
        if data is not None:
            self.start = data[0]
            self.end = data[1]
            self.period = data[2]

    def calc_activation(self, **kwargs):
        """Calculate the new activation value."""
        if (self.ltm.iteration % self.period >= self.start) and (self.ltm.iteration % self.period <= self.end):
            self.new_activation = 1.0
        else:
            self.new_activation = 0.0
        return self.new_activation

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
    def object_held(perceptions):
        """Check if an object is held with one hand."""
        return perceptions["ball_in_left_hand"].raw.data or perceptions["ball_in_right_hand"].raw.data

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
        for idx, _ in enumerate(perceptions["cylinders"].raw.data):
            approximated = not LTMSim.object_too_far(
                perceptions["cylinders"].raw.data[idx].distance, perceptions["cylinders"].raw.data[idx].angle
            ) and LTMSim.object_too_far(
                perceptions["cylinders"].old_raw.data[idx].distance, perceptions["cylinders"].old_raw.data[idx].angle
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


class GoalMotiven(Goal):
    """Goal generated (and managed) by MOTIVEN."""

    def __init__(self, ros_name_prefix=None, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.new_activation_event = None
        self.new_reward_event = None
        self.init_threading()
        self.activation_topic = None
        self.activation_message = None
        self.ok_topic = None
        self.ok_message = None
        self.init_ros(ros_name_prefix)

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["new_activation_event"]
        del state["new_reward_event"]
        return state

    def init_threading(self):
        """Create needed stuff to synchronize threads."""
        self.new_activation_event = threading.Event()
        self.new_reward_event = threading.Event()

    def init_ros(self, ros_name_prefix=None):
        """Create publishers and make subscriptions."""
        # Ugly hack
        if ros_name_prefix is None:
            ros_name_prefix = "/mdb/goal"
        self.activation_topic = rospy.get_param(ros_name_prefix + "_activation_topic")
        self.activation_message = self.class_from_classname(rospy.get_param(ros_name_prefix + "_activation_msg"))
        self.ok_topic = rospy.get_param(ros_name_prefix + "_ok_topic")
        self.ok_message = self.class_from_classname(rospy.get_param(ros_name_prefix + "_ok_msg"))
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

    def calc_activation(self, **kwargs):
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


class GoalObjectHeld(Goal):
    """Goal representing a grasped object with one hand."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_held(perceptions) and not Goal.object_held_with_two_hands(perceptions):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectHeldWithTwoHands(Goal):
    """Goal representing a grasped object with two hands."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_held_with_two_hands(perceptions):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalChangedHands(Goal):
    """Goal representing a change of the hand that holds an object."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.hand_was_changed(perceptions):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalFrontalObject(Goal):
    """Goal representing an object in front of the robot."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if (
                    Goal.object_pickable_withtwohands(perceptions)
                    and (not Goal.object_in_close_box(perceptions))
                    and (not Goal.object_with_robot(perceptions))
                ):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectInCloseBox(Goal):
    """Goal representing an object inside a box (that was reachable)."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_in_close_box(perceptions):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectWithRobot(Goal):
    """Goal representing an object as close to the robot as possible."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_with_robot(perceptions):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalObjectInFarBox(Goal):
    """Goal representing an object inside a box (that was out of reach)."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_in_far_box(perceptions):
                    self.reward = 1.0
                    break
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalApproximatedObject(Goal):
    """Goal representing an reachable object (that was not reachable previously)."""

    def update_success(self):
        """Calculate the reward for the current sensor values."""
        self.reward = 0.0
        perceptions = self.ltm.perceptions
        for activation in self.activation:
            if (self.ltm.sensorial_changes()) and (activation == 1.0):
                if Goal.object_was_approximated(perceptions):
                    self.reward = 1.0
                    break
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
