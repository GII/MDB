"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

import threading
import rospy
from mdb_ltm.node import Node
from mdb_simulator.ltm import LTMSim


class Goal(Node):
    """A subspace in some input space (sensorial or the result of a redescription) where utility was obtained."""

    def __init__(self, data=None, **kwargs):
        """Constructor."""
        super(Goal, self).__init__(**kwargs)
        self.reward = 0.0
        self.new_activation_value = False
        self.new_reward_value = False
        if data is not None:
            self.activation = data[0]
            self.period = data[1]


class GoalMotiven(Goal):
    """Goal generated (and managed) by MOTIVEN."""

    def __init__(self, ros_name_prefix=None, **kwargs):
        """Constructor."""
        super(GoalMotiven, self).__init__(**kwargs)
        self.new_activation = None
        self.new_reward = None
        self.init_threading()
        self.activation_topic = rospy.get_param(ros_name_prefix + "_activation_topic")
        self.activation_message = self.class_from_classname(rospy.get_param(ros_name_prefix + "_activation_msg"))
        self.ok_topic = rospy.get_param(ros_name_prefix + "_ok_topic")
        self.ok_message = self.class_from_classname(rospy.get_param(ros_name_prefix + "_ok_msg"))
        self.init_ros()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["new_activation"]
        del state["new_reward"]
        return state

    def init_threading(self):
        """Create needed stuff to synchronize threads."""
        self.new_activation = threading.Event()
        self.new_reward = threading.Event()

    def init_ros(self):
        """Create publishers and make subscriptions."""
        rospy.logdebug("Subscribing to %s...", self.activation_topic)
        rospy.Subscriber(self.activation_topic, self.activation_message, callback=self.update_activation_callback)
        rospy.logdebug("Subscribing to %s...", self.ok_topic)
        rospy.Subscriber(self.ok_topic, self.ok_message, callback=self.update_success_callback)

    def update_activation_callback(self, data):
        """Calculate the new activation value."""
        if self.ident == data.id:
            rospy.logdebug("Reading goal activation for " + data.id + " = " + str(data.activation))
            self.activation = data.activation
            self.new_activation.set()

    def update_success_callback(self, data):
        """Calculate the value for the current sensor values."""
        if self.ident == data.id:
            rospy.logdebug("Reading goal success for " + data.id + " = " + str(data.ok))
            self.reward = data.ok
            self.new_reward.set()

    def update_activation(self, **kwargs):
        """Calculate the new activation value."""
        self.new_activation.wait()
        self.new_activation.clear()
        super(GoalMotiven, self).update_activation(**kwargs)

    def update_success(self, **kwargs):
        """Calculate the value for the current sensor values."""
        self.new_reward.wait()
        self.new_reward.clear()
        return self.reward


class GoalBallInBox(Goal):
    """Goal representing the desire of putting a ball in a box."""

    def update_activation(self, **kwargs):
        """Calculate the new activation value."""
        if (self.ltm.iteration > 0) and (self.ltm.iteration % self.period == 0):
            if self.activation == 0.0:
                self.activation = 1.0
            else:
                self.activation = 0.0
        super(GoalBallInBox, self).update_activation(**kwargs)

    def update_success(self, perceptions=None):
        """Calculate the value for the current sensor values."""
        self.reward = 0.0
        if perceptions is None:
            perceptions = self.ltm.perceptions
        if (self.ltm.sensorial_changes()) and (self.activation == 1.0):
            if (abs(perceptions["ball_dist"].raw - perceptions["box_dist"].raw) < 0.12) and (
                abs(perceptions["ball_ang"].raw - perceptions["box_ang"].raw) < 0.12
            ):
                self.reward = 1.0
            elif perceptions["ball_in_left_hand"].raw or perceptions["ball_in_right_hand"].raw:
                if perceptions["ball_in_left_hand"].raw and perceptions["ball_in_right_hand"].raw:
                    self.reward = 0.6
                elif (perceptions["ball_in_left_hand"].raw and perceptions["box_ang"].raw > 0) or (
                    perceptions["ball_in_right_hand"].raw and perceptions["box_ang"].raw <= 0
                ):
                    self.reward = 0.6
                elif perceptions["ball_in_left_hand"].raw and perceptions["box_ang"].raw <= 0:
                    if (not perceptions["ball_in_left_hand"].old_raw) and (perceptions["ball_in_right_hand"].old_raw):
                        self.reward = 0.0
                    else:
                        self.reward = 0.3
                elif perceptions["ball_in_right_hand"].raw and perceptions["box_ang"].raw > 0:
                    if (perceptions["ball_in_left_hand"].old_raw) and (not perceptions["ball_in_right_hand"].old_raw):
                        self.reward = 0.0
                    else:
                        self.reward = 0.3
            elif (
                (not perceptions["ball_in_left_hand"].old_raw)
                and (not perceptions["ball_in_right_hand"].old_raw)
                and (abs(perceptions["ball_ang"].raw) < 0.05)
                and (LTMSim.object_pickable_withtwohands(perceptions["ball_dist"].raw, perceptions["ball_ang"].raw))
            ):
                self.reward = 0.3
            elif (not LTMSim.object_too_far(perceptions["ball_dist"].raw, perceptions["ball_ang"].raw)) and (
                LTMSim.object_too_far(perceptions["ball_dist"].old_raw, perceptions["ball_ang"].old_raw)
            ):
                self.reward = 0.2
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward


class GoalBallWithRobot(Goal):
    """Goal representing the desire of bringing a ball as close as possible to the robot."""

    def update_activation(self, **kwargs):
        """Calculate the new activation value."""
        if (self.ltm.iteration > 0) and (self.ltm.iteration % self.period == 0):
            if self.activation == 0.0:
                self.activation = 1.0
            else:
                self.activation = 0.0
        super(GoalBallWithRobot, self).update_activation(**kwargs)

    def update_success(self, perceptions=None):
        """Calculate the value for the current sensor values."""
        self.reward = 0.0
        if perceptions is None:
            perceptions = self.ltm.perceptions
        if (self.ltm.sensorial_changes()) and (self.activation == 1.0):
            perceptions = self.ltm.perceptions
            dist_near, ang_near = LTMSim.calculate_closest_position(perceptions["ball_ang"].raw)
            if (
                (perceptions["ball_dist"].raw - dist_near < 0.05)
                and (perceptions["ball_ang"].raw - ang_near < 0.05)
                and (not perceptions["ball_in_left_hand"].raw)
                and (not perceptions["ball_in_right_hand"].raw)
            ):
                self.reward = 1.0
            elif (
                (perceptions["ball_in_left_hand"].raw or perceptions["ball_in_right_hand"].raw)
                and (not perceptions["ball_in_left_hand"].old_raw)
                and (not perceptions["ball_in_right_hand"].old_raw)
            ):
                self.reward = 0.6
            elif (
                (not perceptions["ball_in_left_hand"].old_raw)
                and (not perceptions["ball_in_right_hand"].old_raw)
                and (not perceptions["ball_in_left_hand"].raw)
                and (not perceptions["ball_in_right_hand"].raw)
                and (abs(perceptions["ball_ang"].raw) < 0.05)
                and (LTMSim.object_pickable_withtwohands(perceptions["ball_dist"].raw, perceptions["ball_ang"].raw))
            ):
                self.reward = 0.3
            elif (not LTMSim.object_too_far(perceptions["ball_dist"].raw, perceptions["ball_ang"].raw)) and (
                LTMSim.object_too_far(perceptions["ball_dist"].old_raw, perceptions["ball_ang"].old_raw)
            ):
                self.reward = 0.2
        rospy.loginfo("Obtaining reward from " + self.ident + " => " + str(self.reward))
        return self.reward
