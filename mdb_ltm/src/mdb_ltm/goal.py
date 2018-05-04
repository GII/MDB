"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

import threading
import time
import rospy
from mdb_ltm.node import Node
from mdb_simulator.ltm import LTMSim

class Goal(Node):
    """
    A subspace in some input space (sensorial or the result of a redescription) where utility was obtained.

    Attributes:

    """

    def __init__(self, data=None, **kwargs):
        """Constructor."""
        super(Goal, self).__init__(**kwargs)
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
        self.reward = 0.0
        self.new_activation = threading.Event()
        topic = rospy.get_param(ros_name_prefix + '_activation_topic')
        message = self.class_from_classname(rospy.get_param(ros_name_prefix + '_activation_msg'))
        rospy.logdebug('Subscribing to %s...', topic)
        rospy.Subscriber(topic, message, callback=self.update_activation_callback)
        self.new_reward = threading.Event()
        topic = rospy.get_param(ros_name_prefix + '_ok_topic')
        message = self.class_from_classname(rospy.get_param(ros_name_prefix + '_ok_msg'))
        rospy.logdebug('Subscribing to %s...', topic)
        rospy.Subscriber(topic, message, callback=self.get_reward_callback)

    def update_activation_callback(self, data):
        """Calculate the new activation value."""
        if self.ident == data.id:
            self.activation = data.activation
            self.new_activation.set()

    def get_reward_callback(self, data):
        """Calculate the value for the current sensor values."""
        if self.ident == data.id:
            self.reward = data.ok
            self.new_reward.set()

    def update_activation(self, **kwargs):
        """Calculate the new activation value."""
        self.new_activation.wait()
        self.new_activation.clear()
        super(GoalMotiven, self).update_activation(**kwargs)

    def get_reward(self, perceptions=None):
        """Calculate the value for the current sensor values."""
        self.new_reward.wait()
        self.new_reward.clear()
        rospy.loginfo('Obtaining reward from ' + self.ident + ' => ' + str(self.reward))
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
        
    def get_reward(self, perceptions=None):
        """Calculate the value for the current sensor values."""
        reward = 0.0
        if perceptions is None:
            perceptions = self.ltm.perceptions
        if (self.ltm.sensorial_changes()) and (self.activation == 1.0):
            if (
                    (abs(perceptions['ball_dist'].raw - perceptions['box_dist'].raw) < 0.05) and
                    (abs(perceptions['ball_ang'].raw - perceptions['box_ang'].raw) < 0.05)
                ): # yapf: disable
                reward = 1.0
            elif perceptions['ball_in_left_hand'].raw or perceptions['ball_in_right_hand'].raw:
                if perceptions['ball_in_left_hand'].raw and perceptions['ball_in_right_hand'].raw:
                    reward = 0.6
                elif (
                        (perceptions['ball_in_left_hand'].raw and perceptions['box_ang'].raw <= 0) or
                        (perceptions['ball_in_right_hand'].raw and perceptions['box_ang'].raw > 0)
                    ): # yapf: disable
                    reward = 0.6
                elif perceptions['ball_in_left_hand'].raw and perceptions['box_ang'].raw > 0:
                    if (
                            (not perceptions['ball_in_left_hand'].old_raw) and
                            (perceptions['ball_in_right_hand'].old_raw)
                        ): # yapf: disable
                        reward = 0.0
                    else:
                        reward = 0.3
                elif perceptions['ball_in_right_hand'].raw and perceptions['box_ang'].raw <= 0:
                    if (
                            (perceptions['ball_in_left_hand'].old_raw) and
                            (not perceptions['ball_in_right_hand'].old_raw)
                        ): # yapf: disable
                        reward = 0.0
                    else:
                        reward = 0.3
            elif (
                    (abs(perceptions['ball_ang'].raw) < 0.05) and
                    (not perceptions['ball_in_left_hand'].old_raw) and
                    (not perceptions['ball_in_right_hand'].old_raw)
                ): # yapf: disable
                reward = 0.3
            elif (
                    (not LTMSim.object_too_far(perceptions['ball_dist'].raw, perceptions['ball_ang'].raw)) and
                    (LTMSim.object_too_far(perceptions['ball_dist'].old_raw, perceptions['ball_ang'].old_raw))
                ): # yapf: disable
                reward = 0.2
        rospy.loginfo('Obtaining reward from ' + self.ident + ' => ' + str(reward))
        return reward


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

    def get_reward(self, perceptions=None):
        """Calculate the value for the current sensor values."""
        reward = 0.0
        if perceptions is None:
            perceptions = self.ltm.perceptions
        if (self.ltm.sensorial_changes()) and (self.activation == 1.0):
            perceptions = self.ltm.perceptions
            dist_near, ang_near = LTMSim.calculate_closest_position(perceptions['ball_ang'].raw)
            if (
                    (perceptions['ball_dist'].raw - dist_near < 0.05) and
                    (perceptions['ball_ang'].raw - ang_near < 0.05) and
                    (not perceptions['ball_in_left_hand'].raw) and
                    (not perceptions['ball_in_right_hand'].raw)
                ): # yapf: disable
                reward = 1.0
            elif (
                    (perceptions['ball_in_left_hand'].raw or perceptions['ball_in_right_hand'].raw) and
                    (not perceptions['ball_in_left_hand'].old_raw) and
                    (not perceptions['ball_in_right_hand'].old_raw)
                ): # yapf: disable
                reward = 0.6
            elif (
                    (abs(perceptions['ball_ang'].raw) < 0.05) and
                    (abs(perceptions['ball_ang'].old_raw) >= 0.05) and
                    (not perceptions['ball_in_left_hand'].raw) and
                    (not perceptions['ball_in_right_hand'].raw)
                ): # yapf: disable
                reward = 0.3
            elif (
                    (not LTMSim.object_too_far(perceptions['ball_dist'].raw, perceptions['ball_ang'].raw)) and
                    (LTMSim.object_too_far(perceptions['ball_dist'].old_raw, perceptions['ball_ang'].old_raw))
                ): # yapf: disable
                reward = 0.2
        rospy.loginfo('Obtaining reward from ' + self.ident + ' => ' + str(reward))
        return reward
