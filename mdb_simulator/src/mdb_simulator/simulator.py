"""
The shiny, all new, MDB 3.

Available from https://github.com/robotsthatdream/mdb_ltm
Copyright 2017-18 Richard J. Duro, Jose A. Becerra.
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
from enum import Enum, auto
import os.path
import math
import yaml
import numpy
import rospy


class World(Enum):
    """Worlds to be simulated."""

    gripper_and_low_friction = auto()
    no_gripper_and_high_friction = auto()
    gripper_and_low_friction_two_boxes = auto()


class LTMSim(object):
    """A very simple events-based simulator for LTM experiments."""

    inner = numpy.poly1d(numpy.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [0.45, 0.47, 0.525, 0.65, 0.9], 3))
    outer = numpy.poly1d(numpy.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [1.15, 1.25, 1.325, 1.375, 1.375], 3))

    def __init__(self):
        """Constructor."""
        self.ident = None
        self.world = None
        self.messages = {}
        self.base_messages = {}
        self.perceptions = {}
        self.publishers = {}
        self.catched_object = None

    @staticmethod
    def __class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition(".")
        node_module = __import__(module_string, fromlist=[class_string])
        node_class = getattr(node_module, class_string)
        return node_class

    @classmethod
    def object_too_close(cls, dist, ang):
        """Return True if the object is too close to the robot to be caught."""
        return dist < cls.inner(abs(ang))

    @classmethod
    def object_too_far(cls, dist, ang):
        """Return True if the object is out of range of the robot."""
        return dist > cls.outer(abs(ang))

    @classmethod
    def object_outside_table(cls, dist, ang):
        """Return True if the object is outside the table. This is used in some scripts..."""
        object_y = numpy.sin(ang) * dist
        object_x = numpy.cos(ang) * dist
        return not (-1.07 <= object_y <= 1.07 and 0.35 <= object_x <= 1.27)

    @classmethod
    def object_is_small(cls, rad):
        """Return True if the ball is small, False if it is big. Right now, small is 0.03 and big 0.07."""
        return rad <= 0.05

    @classmethod
    def object_pickable_withtwohands(cls, dist, ang):
        """Return True if the object is in a place where it can be picked with two hands."""
        return abs(ang) <= 0.3925 and 0.46 <= dist <= 0.75

    @classmethod
    def __send_object_twohandsreachable(cls, dist):
        """Calculate the coordinates of the object when moving it to a place where it can be picked with two hands."""
        x_coord = 0
        y_coord = dist
        if y_coord < 0.46:
            y_coord = 0.46
        elif y_coord > 0.75:
            y_coord = 0.75
        new_dist = numpy.linalg.norm([y_coord, x_coord])
        return new_dist, 0

    @classmethod
    def __send_object_outofreach(cls, ang):
        """Calculate the coordinates of the object when moving it out of reach."""
        dist = cls.outer(abs(ang))
        y_coord = dist * math.sin(ang)
        x_coord = dist * math.cos(ang)
        if y_coord < -1.07:
            y_coord = -1.07
            x_coord = 0.84
        elif y_coord > 1.07:
            y_coord = 1.07
            x_coord = 0.84
        new_dist = 0.01 + numpy.linalg.norm([y_coord, x_coord])
        new_ang = numpy.arctan2(y_coord, x_coord)
        return new_dist, new_ang

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

    def __object_in_close_box(self, dist, ang):
        """Check if there is an object inside of a box."""
        inside = False
        for box in self.perceptions["boxes"].data:
            if not LTMSim.object_too_far(box.distance, box.angle):
                inside = (abs(box.distance - dist) < 0.05) and (abs(box.angle - ang) < 0.05)
                if inside:
                    break
        return inside

    def __object_in_far_box(self, dist, ang):
        """Check if there is an object inside of a box."""
        inside = False
        for box in self.perceptions["boxes"].data:
            if LTMSim.object_too_far(box.distance, box.angle):
                inside = (abs(box.distance - dist) < 0.05) and (abs(box.angle - ang) < 0.05)
                if inside:
                    break
        return inside

    def __object_with_robot(self, dist, ang):
        """Check if there is an object adjacent to the robot."""
        together = False
        if (not self.perceptions["ball_in_left_hand"].data) and (not self.perceptions["ball_in_right_hand"].data):
            dist_near, ang_near = LTMSim.calculate_closest_position(ang)
            together = (abs(dist - dist_near) < 0.05) and (abs(ang - ang_near) < 0.05)
        return together

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
                    and LTMSim.object_too_far(box.distance, box.angle)
                ):
                    self.perceptions["clean_area"].data = True
                    return True
        return False

    def __update_reward_sensor(self):
        """Update goal sensors' values."""
        for sensor in self.perceptions:
            reward_method = getattr(self, "reward_" + sensor, None)
            if callable(reward_method):
                reward_method()

    def __random_position(self, in_valid=True, out_valid=True):
        """Return a random position in the table."""
        valid = False
        while not valid:
            object_y = numpy.random.uniform(low=-1.07, high=1.07)
            object_x = numpy.random.uniform(low=0.37, high=1.29)
            distance = numpy.linalg.norm([object_y, object_x])
            angle = numpy.arctan2(object_y, object_x)
            valid = not self.object_too_close(distance, angle)
            if not in_valid:
                valid = valid and self.object_too_far(distance, angle)
            if not out_valid:
                valid = valid and not self.object_too_far(distance, angle)
            if valid:
                for box in self.perceptions["boxes"].data:
                    if (abs(box.distance - distance) < 0.15) and (abs(box.angle - angle) < 0.15):
                        valid = False
                        break
            if valid:
                for cylinder in self.perceptions["cylinders"].data:
                    if (abs(cylinder.distance - distance) < 0.15) and (abs(cylinder.angle - angle) < 0.15):
                        valid = False
                        break
        return distance, angle

    def __random_perceptions(self):
        """Randomize the state of the environment."""
        # Objects
        self.catched_object = None
        if self.world == World.gripper_and_low_friction or self.world == World.no_gripper_and_high_friction:
            self.perceptions["boxes"].data = []
            distance, angle = self.__random_position(in_valid=True, out_valid=True)
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[0].distance = distance
            self.perceptions["boxes"].data[0].angle = angle
            self.perceptions["boxes"].data[0].diameter = 0.12
            self.perceptions["cylinders"].data = []
            distance, angle = self.__random_position(in_valid=True, out_valid=True)
            self.perceptions["cylinders"].data.append(self.base_messages["cylinders"]())
            self.perceptions["cylinders"].data[0].distance = distance
            self.perceptions["cylinders"].data[0].angle = angle
            if numpy.random.uniform() > 0.5:
                self.perceptions["cylinders"].data[0].diameter = 0.03
            else:
                self.perceptions["cylinders"].data[0].diameter = 0.07
            self.perceptions["ball_in_left_hand"].data = False
            self.perceptions["ball_in_right_hand"].data = False
            object_distance = self.perceptions["cylinders"].data[0].distance
            object_angle = self.perceptions["cylinders"].data[0].angle
            if (
                (World.gripper_and_low_friction.name in self.world.name)
                and self.object_is_small(object_distance)
                and (not self.object_too_far(object_distance, object_angle))
                and (numpy.random.uniform() > 0.5)
            ):
                self.catched_object = self.perceptions["cylinders"].data[0]
                if object_angle > 0:
                    self.perceptions["ball_in_left_hand"].data = True
                    self.perceptions["ball_in_right_hand"].data = False
                else:
                    self.perceptions["ball_in_left_hand"].data = False
                    self.perceptions["ball_in_right_hand"].data = True
            if self.object_pickable_withtwohands(distance, angle) and (numpy.random.uniform() > 0.5):
                self.catched_object = self.perceptions["cylinders"].data[0]
                self.perceptions["ball_in_left_hand"].data = True
                self.perceptions["ball_in_right_hand"].data = True
        elif self.world == World.gripper_and_low_friction_two_boxes:
            self.perceptions["boxes"].data = []
            distance, angle = self.__random_position(in_valid=True, out_valid=False)
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[0].distance = distance
            self.perceptions["boxes"].data[0].angle = angle
            self.perceptions["boxes"].data[0].diameter = 0.12
            distance, angle = self.__random_position(in_valid=False, out_valid=True)
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[1].distance = distance
            self.perceptions["boxes"].data[1].angle = angle
            self.perceptions["boxes"].data[1].diameter = 0.12
            self.perceptions["cylinders"].data = []
            distance, angle = self.__random_position(in_valid=True, out_valid=False)
            self.perceptions["cylinders"].data.append(self.base_messages["cylinders"]())
            self.perceptions["cylinders"].data[0].distance = distance
            self.perceptions["cylinders"].data[0].angle = angle
            if numpy.random.uniform() > 0.5:
                self.perceptions["cylinders"].data[0].diameter = 0.03
            else:
                self.perceptions["cylinders"].data[0].diameter = 0.07
            self.perceptions["ball_in_left_hand"].data = False
            self.perceptions["ball_in_right_hand"].data = False
        else:
            rospy.logerr("Unknown world received in simulator!")
        # Goal sensors
        self.__update_reward_sensor()

    def grasp_object_policy(self):
        """Grasp an object with a gripper."""
        if not self.catched_object:
            for cylinder in self.perceptions["cylinders"].data:
                if (
                    World.gripper_and_low_friction.name in self.world.name
                    and (not self.object_too_far(cylinder.distance, cylinder.angle))
                    and self.object_is_small(cylinder.diameter)
                ):
                    if cylinder.angle > 0.0:
                        self.perceptions["ball_in_left_hand"].data = True
                    else:
                        self.perceptions["ball_in_right_hand"].data = True
                    self.catched_object = cylinder
                    break

    def grasp_with_two_hands_policy(self):
        """Grasp an object using both arms."""
        if not self.catched_object:
            for cylinder in self.perceptions["cylinders"].data:
                if (
                    (self.object_pickable_withtwohands(cylinder.distance, cylinder.angle)) and
                    (
                        (World.no_gripper_and_high_friction.name in self.world.name) or
                        (not self.object_is_small(cylinder.diameter))
                    )
                ):
                    self.perceptions["ball_in_left_hand"].data = True
                    self.perceptions["ball_in_right_hand"].data = True
                    self.catched_object = cylinder
                    break

    def change_hands_policy(self):
        """Exchange an object from one hand to the other one."""
        if self.perceptions["ball_in_left_hand"].data and (not self.perceptions["ball_in_right_hand"].data):
            self.perceptions["ball_in_left_hand"].data = False
            self.perceptions["ball_in_right_hand"].data = True
            self.catched_object.angle = -self.catched_object.angle
        elif (not self.perceptions["ball_in_left_hand"].data) and self.perceptions["ball_in_right_hand"].data:
            self.perceptions["ball_in_left_hand"].data = True
            self.perceptions["ball_in_right_hand"].data = False
            self.catched_object.angle = -self.catched_object.angle

    def sweep_object_policy(self):
        """Sweep an object to the front of the robot."""
        if not self.catched_object:
            for cylinder in self.perceptions["cylinders"].data:
                if not self.object_too_far(cylinder.distance, cylinder.angle):
                    sign = numpy.sign(cylinder.angle) # pylint: disable=E1111
                    cylinder.distance, cylinder.angle = self.__send_object_twohandsreachable(cylinder.distance)
                    if ((World.gripper_and_low_friction.name in self.world.name) and
                        self.object_is_small(cylinder.diameter)
                    ):
                        cylinder.angle = sign * 0.4
                    # This is necessary so sweep never puts the object close to the robot or colliding with a box.
                    # This is not realistic, it needs to be improved.
                    if self.__object_with_robot(cylinder.distance, cylinder.angle) or self.__object_in_close_box(
                        cylinder.distance, cylinder.angle
                    ):
                        if cylinder.distance > 0.65:
                            cylinder.distance -= 0.10
                        else:
                            cylinder.distance += 0.10
                    break

    def put_object_in_box_policy(self):
        """Put an object into the box."""
        if self.catched_object:
            for box in self.perceptions["boxes"].data:
                if (not self.object_too_far(box.distance, box.angle)) and (
                    ((box.angle > 0.0) and self.perceptions["ball_in_left_hand"].data)
                    or ((box.angle <= 0.0) and self.perceptions["ball_in_right_hand"].data)
                ):
                    self.catched_object.distance = box.distance
                    self.catched_object.angle = box.angle
                    self.perceptions["ball_in_left_hand"].data = False
                    self.perceptions["ball_in_right_hand"].data = False
                    self.catched_object = None
                    break

    def put_object_with_robot_policy(self):
        """Put an object as close to the robot as possible."""
        if self.catched_object:
            self.catched_object.distance, self.catched_object.angle = self.calculate_closest_position(
                self.catched_object.angle
            )
            # Box and cylinder collide.
            # This is not realistic, it needs to be improved.
            if self.__object_in_close_box(self.catched_object.distance, self.catched_object.angle):
                self.catched_object.angle += 0.10
            self.perceptions["ball_in_left_hand"].data = False
            self.perceptions["ball_in_right_hand"].data = False
            self.catched_object = None

    def throw_policy(self):
        """Throw an object."""
        if self.catched_object:
            for box in self.perceptions["boxes"].data:
                if self.object_too_far(box.distance, box.angle) and (
                    (box.angle > 0.0 and self.perceptions["ball_in_left_hand"].data)
                    or (box.angle <= 0.0 and self.perceptions["ball_in_right_hand"].data)
                ):
                    self.catched_object.distance = box.distance
                    self.catched_object.angle = box.angle
                    # else:
                    #     self.catched_object.distance, self.catched_object.angle = self.__send_object_outofreach(
                    #         self.catched_object.angle
                    #     )
                    self.perceptions["ball_in_left_hand"].data = False
                    self.perceptions["ball_in_right_hand"].data = False
                    self.catched_object = None
                    break

    def ask_nicely_policy(self):
        """Ask someone to bring the object closer to us."""
        if not self.catched_object:
            for cylinder in self.perceptions["cylinders"].data:
                if self.object_too_far(cylinder.distance, cylinder.angle):
                    cylinder.distance = 1.13
                    break

    def __new_command_callback(self, data):
        """Process a command."""
        rospy.logdebug("Command received...")
        self.world = World[data.world]
        self.__random_perceptions()
        for ident, publisher in self.publishers.items():
            rospy.logdebug("Publishing " + ident + " = " + str(self.perceptions[ident].data))
            publisher.publish(self.perceptions[ident])
        if (not self.catched_object) and (
            self.perceptions["ball_in_left_hand"].data or self.perceptions["ball_in_right_hand"].data
        ):
            rospy.logerr("Critical error: catched_object is empty and it should not!!!")

    def __new_action_callback(self, data):
        """Execute a policy and publish new perceptions."""
        rospy.logdebug("Executing policy %s...", data.data)
        getattr(self, data.data + "_policy")()
        self.__update_reward_sensor()
        for ident, publisher in self.publishers.items():
            rospy.logdebug("Publishing " + ident + " = " + str(self.perceptions[ident].data))
            publisher.publish(self.perceptions[ident])
        if (not self.catched_object) and (
            self.perceptions["ball_in_left_hand"].data or self.perceptions["ball_in_right_hand"].data
        ):
            rospy.logerr("Critical error: catched_object is empty and it should not!!!")

    def __configure_sensors(self, sensors):
        """Configure the ROS publishers where publish sensor values."""
        for sensor in sensors:
            sid = sensor["id"]
            prefix = sensor["ros_name_prefix"]
            topic = rospy.get_param(prefix + "_topic")
            classname = rospy.get_param(prefix + "_msg")
            self.messages[sid] = self.__class_from_classname(classname)
            self.perceptions[sid] = self.messages[sid]()
            if "List" in classname:
                self.perceptions[sid].data = []
                self.base_messages[sid] = self.__class_from_classname(classname.replace("List", ""))
            else:
                self.perceptions[sid].data = 0
            rospy.logdebug("I will publish to %s...", topic)
            self.publishers[sid] = rospy.Publisher(topic, self.messages[sid], latch=True, queue_size=0)

    def __configure_simulation(self, simulation):
        """Configure the ROS topic where listen for commands to be executed."""
        self.ident = simulation["id"]
        topic = rospy.get_param(simulation["ros_name_prefix"] + "_topic")
        message = self.__class_from_classname(rospy.get_param(simulation["ros_name_prefix"] + "_msg"))
        rospy.logdebug("Subscribing to %s...", topic)
        rospy.Subscriber(topic, message, callback=self.__new_command_callback)
        topic = rospy.get_param(simulation["executed_policy_prefix"] + "_topic")
        message = self.__class_from_classname(rospy.get_param(simulation["executed_policy_prefix"] + "_msg"))
        rospy.logdebug("Subscribing to %s...", topic)
        rospy.Subscriber(topic, message, callback=self.__new_action_callback)

    def __load_configuration(self, log_level, config_file):
        """Load configuration from a file."""
        rospy.init_node("ltm_simulator", log_level=getattr(rospy, log_level))
        if config_file is None:
            rospy.logerr("No configuration file for the LTM simulator specified!")
        else:
            if not os.path.isfile(config_file):
                rospy.logerr(config_file + " does not exist!")
            else:
                rospy.loginfo("Loading configuration from %s...", config_file)
                config = yaml.load(open(config_file, "r"), Loader=yaml.CLoader)
                self.__configure_sensors(config["Simulator"]["Sensors"])
                # Be ware, we can not subscribe to control channel before creating all sensor publishers.
                self.__configure_simulation(config["Control"])

    def run(self, log_level="INFO", config_file=None):
        """Start the LTM simulator."""
        self.__load_configuration(log_level, config_file)
        rospy.loginfo("Starting LTM Simulator...")
        rospy.spin()
        rospy.loginfo("Ending LTM Simulator...")
