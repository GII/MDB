"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import math
import numpy
import rospy
from mdb_robots.robot import World, Robot


class SimulatedBaxter(Robot):
    """A very simple events-based simulator for LTM experiments."""

    def __init__(self, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.ident = "SimulatedBaxter"

    @classmethod
    def object_too_close(cls, dist, ang):
        """Return True if the object is too close to the robot to be caught."""
        return dist < cls.inner(abs(ang))

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
    def send_object_twohandsreachable(cls, dist):
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
    def send_object_outofreach(cls, ang):
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

    def object_in_close_box(self, dist, ang):
        """Check if there is an object inside of a box."""
        inside = False
        for box in self.perceptions["boxes"].data:
            if not SimulatedBaxter.object_too_far(box.distance, box.angle):
                inside = (abs(box.distance - dist) < 0.05) and (abs(box.angle - ang) < 0.05)
                if inside:
                    break
        return inside

    def object_in_far_box(self, dist, ang):
        """Check if there is an object inside of a box."""
        inside = False
        for box in self.perceptions["boxes"].data:
            if SimulatedBaxter.object_too_far(box.distance, box.angle):
                inside = (abs(box.distance - dist) < 0.05) and (abs(box.angle - ang) < 0.05)
                if inside:
                    break
        return inside

    def object_with_robot(self, dist, ang):
        """Check if there is an object adjacent to the robot."""
        together = False
        if (not self.perceptions["ball_in_left_hand"].data) and (not self.perceptions["ball_in_right_hand"].data):
            dist_near, ang_near = SimulatedBaxter.calculate_closest_position(ang)
            together = (abs(dist - dist_near) < 0.05) and (abs(ang - ang_near) < 0.05)
        return together

    def avoid_reward_by_chance(self, distance, angle):
        """Avoid a reward situation obtained by chance."""
        # This is necessary so sweep never puts the object close to the robot or colliding with a box.
        # This is not realistic, it needs to be improved.
        while (
            self.object_with_robot(distance, angle)
            or self.object_in_close_box(distance, angle)
            or self.object_in_far_box(distance, angle)
        ):
            if distance > 0.65:
                distance -= 0.10
            else:
                distance += 0.10
        return distance

    def random_position(self, in_valid=True, out_valid=True):
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

    def random_perceptions(self):
        """Randomize the state of the environment."""
        # Objects
        self.catched_object = None
        if self.world == World.gripper_and_low_friction or self.world == World.no_gripper_and_high_friction:
            self.perceptions["boxes"].data = []
            distance, angle = self.random_position(in_valid=True, out_valid=True)
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[0].distance = distance
            self.perceptions["boxes"].data[0].angle = angle
            self.perceptions["boxes"].data[0].diameter = 0.12
            self.perceptions["cylinders"].data = []
            distance, angle = self.random_position(in_valid=True, out_valid=True)
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
            distance, angle = self.random_position(in_valid=True, out_valid=False)
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[0].distance = distance
            self.perceptions["boxes"].data[0].angle = angle
            self.perceptions["boxes"].data[0].diameter = 0.12
            distance, angle = self.random_position(in_valid=False, out_valid=True)
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[1].distance = distance
            self.perceptions["boxes"].data[1].angle = angle
            self.perceptions["boxes"].data[1].diameter = 0.12
            self.perceptions["cylinders"].data = []
            distance, angle = self.random_position(in_valid=True, out_valid=False)
            self.perceptions["cylinders"].data.append(self.base_messages["cylinders"]())
            self.perceptions["cylinders"].data[0].distance = distance
            self.perceptions["cylinders"].data[0].angle = angle
            if numpy.random.uniform() > 0.5:
                self.perceptions["cylinders"].data[0].diameter = 0.03
            else:
                self.perceptions["cylinders"].data[0].diameter = 0.07
            self.perceptions["ball_in_left_hand"].data = False
            self.perceptions["ball_in_right_hand"].data = False
        elif self.world == World.kitchen:
            self.perceptions["boxes"].data = []
            self.perceptions["boxes"].data.append(self.base_messages["boxes"]())
            self.perceptions["boxes"].data[0].distance = 0.605
            self.perceptions["boxes"].data[0].angle = 0.0
            self.perceptions["boxes"].data[0].diameter = 0.12
            self.perceptions["boxes"].data[0].color = "skillet"
            self.perceptions["cylinders"].data = []
            distance, angle = self.random_position(in_valid=True, out_valid=True)
            self.perceptions["cylinders"].data.append(self.base_messages["cylinders"]())
            self.perceptions["cylinders"].data[0].distance = distance
            self.perceptions["cylinders"].data[0].angle = angle
            self.perceptions["cylinders"].data[0].diameter = 0.03
            self.perceptions["cylinders"].data[0].color = "carrot"
            distance, angle = self.random_position(in_valid=True, out_valid=True)
            self.perceptions["cylinders"].data.append(self.base_messages["cylinders"]())
            self.perceptions["cylinders"].data[1].distance = distance
            self.perceptions["cylinders"].data[1].angle = angle
            self.perceptions["cylinders"].data[1].diameter = 0.03
            self.perceptions["cylinders"].data[1].color = "eggplant"
            distance, angle = self.random_position(in_valid=True, out_valid=True)
            self.perceptions["cylinders"].data.append(self.base_messages["cylinders"]())
            self.perceptions["cylinders"].data[2].distance = distance
            self.perceptions["cylinders"].data[2].angle = angle
            self.perceptions["cylinders"].data[2].diameter = 0.03
            self.perceptions["cylinders"].data[2].color = "cabbage"
            self.perceptions["ball_in_left_hand"].data = False
            self.perceptions["ball_in_right_hand"].data = False
        else:
            rospy.logerr("Unknown world received in simulator!")
        # Goal sensors
        self.update_reward_sensor()

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
                if (self.object_pickable_withtwohands(cylinder.distance, cylinder.angle)) and (
                    (World.no_gripper_and_high_friction.name in self.world.name)
                    or (not self.object_is_small(cylinder.diameter))
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
            self.catched_object.distance = self.avoid_reward_by_chance(
                self.catched_object.distance, self.catched_object.angle
            )
        elif (not self.perceptions["ball_in_left_hand"].data) and self.perceptions["ball_in_right_hand"].data:
            self.perceptions["ball_in_left_hand"].data = True
            self.perceptions["ball_in_right_hand"].data = False
            self.catched_object.angle = -self.catched_object.angle
            self.catched_object.distance = self.avoid_reward_by_chance(
                self.catched_object.distance, self.catched_object.angle
            )

    def sweep_object_policy(self):
        """Sweep an object to the front of the robot."""
        if not self.catched_object:
            for cylinder in self.perceptions["cylinders"].data:
                if not self.object_too_far(cylinder.distance, cylinder.angle):
                    sign = numpy.sign(cylinder.angle)  # pylint: disable=E1111
                    cylinder.distance, cylinder.angle = self.send_object_twohandsreachable(cylinder.distance)
                    if (World.gripper_and_low_friction.name in self.world.name) and self.object_is_small(
                        cylinder.diameter
                    ):
                        cylinder.angle = sign * 0.4
                    cylinder.distance = self.avoid_reward_by_chance(cylinder.distance, cylinder.angle)
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
            if self.object_in_close_box(self.catched_object.distance, self.catched_object.angle):
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
                    #     self.catched_object.distance, self.catched_object.angle = self.send_object_outofreach(
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
                    cylinder.distance = self.avoid_reward_by_chance(1.13, cylinder.angle)
                    break
