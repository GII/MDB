"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
from enum import Enum
import sys
import os.path
import math
import numpy
import yaml
import yamlloader
import rospy


class World(Enum):
    """Worlds to be simulated."""

    gripper_and_low_friction = 1
    no_gripper_and_high_friction = 2
    gripper_and_low_friction_two_boxes = 3
    kitchen = 4


class Robot(object):
    """ROS node to provide robot sensorization to MDB."""

    inner = numpy.poly1d(numpy.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [0.45, 0.47, 0.525, 0.65, 0.9], 3))
    outer = numpy.poly1d(numpy.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [1.15, 1.25, 1.325, 1.375, 1.375], 3))

    def __init__(self, **kwargs):
        """Constructor."""
        self.ident = None
        self.world = None
        self.base_messages = {}
        self.perceptions = {}
        self.publishers = {}
        self.catched_object = None

    @staticmethod
    def class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        if sys.version_info < (3, 0):
            node_module = __import__(module_string, fromlist=[bytes(class_string, "utf-8")])
        else:
            node_module = __import__(module_string, fromlist=[class_string])
        # node_module = importlib.import_module('.' + class_string, package=module_string)
        node_class = getattr(node_module, class_string)
        return node_class

    @classmethod
    def object_too_far(cls, dist, ang):
        """Return True if the object is out of range of the robot."""
        return dist > cls.outer(abs(ang))

    @classmethod
    def calculate_closest_position(cls, ang):
        """Calculate the closest feasible position for an object taking into account the angle."""
        dist = cls.inner(abs(ang))
        y_coord = dist * math.sin(ang)
        x_coord = dist * math.cos(ang)
        if y_coord < -0.38:
            y_coord = -0.38
            x_coord = 0.35
        elif y_coord > 0.38:
            y_coord = 0.38
            x_coord = 0.35
        new_dist = 0.01 + numpy.linalg.norm([y_coord, x_coord])
        new_ang = numpy.arctan2(y_coord, x_coord)
        return new_dist, new_ang

    def reward_ball_in_box(self):
        """Reward for object in box goal."""
        self.perceptions["ball_in_box"].data = False
        for cylinder in self.perceptions["cylinders"].data:
            for box in self.perceptions["boxes"].data:
                if (cylinder.distance == box.distance) and (cylinder.angle == box.angle):
                    self.perceptions["ball_in_box"].data = True
                    return True
        return False

    def reward_ball_with_robot(self):
        """Reward for object with robot goal."""
        self.perceptions["ball_with_robot"].data = False
        for cylinder in self.perceptions["cylinders"].data:
            dist, ang = self.calculate_closest_position(cylinder.angle)
            if (cylinder.distance == dist) and (cylinder.angle == ang):
                self.perceptions["ball_with_robot"].data = True
                return True
        return False

    def reward_clean_area(self):
        """Reward for cleaning the table goal."""
        self.perceptions["clean_area"].data = False
        for cylinder in self.perceptions["cylinders"].data:
            for box in self.perceptions["boxes"].data:
                if (
                    (cylinder.distance == box.distance)
                    and (cylinder.angle == box.angle)
                    and Robot.object_too_far(box.distance, box.angle)
                ):
                    self.perceptions["clean_area"].data = True
                    return True
        return False

    def update_reward_sensor(self):
        """Update goal sensors' values."""
        for sensor in self.perceptions:
            reward_method = getattr(self, "reward_" + sensor, None)
            if callable(reward_method):
                reward_method()

    def random_perceptions(self):
        """Randomize the state of the environment."""
        raise NotImplementedError

    def new_command_callback(self, data):
        """Process a command."""
        rospy.logdebug("Command received...")
        self.world = World[data.world]
        self.random_perceptions()
        for ident, publisher in self.publishers.items():
            rospy.logdebug("Publishing " + ident + " = " + str(self.perceptions[ident].data))
            publisher.publish(self.perceptions[ident])
        if (not self.catched_object) and (
            self.perceptions["ball_in_left_hand"].data or self.perceptions["ball_in_right_hand"].data
        ):
            rospy.logerr("Critical error: catched_object is empty and it should not!!!")

    def new_action_callback(self, data):
        """Execute a policy and publish new perceptions."""
        rospy.logdebug("Executing policy %s...", data.data)
        getattr(self, data.data + "_policy")()
        self.update_reward_sensor()
        for ident, publisher in self.publishers.items():
            rospy.logdebug("Publishing " + ident + " = " + str(self.perceptions[ident].data))
            publisher.publish(self.perceptions[ident])
        if (not self.catched_object) and (
            self.perceptions["ball_in_left_hand"].data or self.perceptions["ball_in_right_hand"].data
        ):
            rospy.logerr("Critical error: catched_object is empty and it should not!!!")

    def setup_perceptions(self, perceptions):
        """Configure the ROS publishers where publish perception values."""
        for perception in perceptions:
            sid = perception["id"]
            prefix = perception["ros_name_prefix"]
            topic = rospy.get_param(prefix + "_topic")
            classname = rospy.get_param(prefix + "_msg")
            message = self.class_from_classname(classname)
            self.perceptions[sid] = message()
            if "List" in classname:
                self.perceptions[sid].data = []
                self.base_messages[sid] = self.class_from_classname(classname.replace("List", ""))
            else:
                self.perceptions[sid].data = 0
            rospy.logdebug("I will publish to %s...", topic)
            self.publishers[sid] = rospy.Publisher(topic, message, latch=True, queue_size=0)

    def setup_sensors(self, sensors):
        """Configure the ROS subscribers to read the real sensor values."""
        for sensor in sensors:
            prefix = sensor["ros_name_prefix"]
            topic = rospy.get_param(prefix + "_topic")
            message = self.class_from_classname(rospy.get_param(prefix + "_msg"))
            callback = getattr(self, "new_" + sensor["id"] + "_value_callback")
            rospy.logdebug("Subscribing to %s...", topic)
            rospy.Subscriber(topic, message, callback=callback)

    def setup_control_channel(self, control):
        """Configure the ROS topic where listen for commands to be executed."""
        topic = rospy.get_param(control["ros_name_prefix"] + "_topic")
        message = self.class_from_classname(rospy.get_param(control["ros_name_prefix"] + "_msg"))
        rospy.logdebug("Subscribing to %s...", topic)
        rospy.Subscriber(topic, message, callback=self.new_command_callback)
        topic = rospy.get_param(control["executed_policy_prefix"] + "_topic")
        message = self.class_from_classname(rospy.get_param(control["executed_policy_prefix"] + "_msg"))
        rospy.logdebug("Subscribing to %s...", topic)
        rospy.Subscriber(topic, message, callback=self.new_action_callback)

    def load_configuration(self, log_level, config_file):
        """Load configuration from a file."""
        rospy.init_node("mdb_" + self.ident.lower(), log_level=getattr(rospy, log_level))
        if config_file is None:
            rospy.logerr("No configuration file for " + self.ident + " sensorization specified!")
        else:
            if not os.path.isfile(config_file):
                rospy.logerr(config_file + " does not exist!")
            else:
                rospy.loginfo("Loading configuration from %s...", config_file)
                config = yaml.load(open(config_file, "r", encoding="utf-8"), Loader=yamlloader.ordereddict.CLoader)
                self.setup_perceptions(config[self.ident]["Perceptions"])
                self.setup_sensors(config[self.ident]["Sensors"])
                # Be ware, we can not subscribe to control channel before creating all perception publishers.
                self.setup_control_channel(config["Control"])

    def run(self, log_level="INFO", config_file=None):
        """Start the robot interface."""
        self.load_configuration(log_level, config_file)
        rospy.loginfo("Starting " + self.ident + " interface...")
        rospy.spin()
        rospy.loginfo("Ending " + self.ident + " interface...")
