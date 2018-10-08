"""
The shiny, all new, MDB 3.

Available from https://github.com/robotsthatdream/mdb_ltm
Copyright 2017-18 Richard J. Duro, Jose A. Becerra.
Distributed under GPLv3.
"""

import os.path
import math
import pdb
import yaml
import numpy
import rospy

class LTMSim(object):
    """A very simple events-based simulator for LTM experiments."""

    inner = numpy.poly1d(numpy.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [0.45, 0.47, 0.525, 0.65, 0.9], 3))
    outer = numpy.poly1d(numpy.polyfit([0.0, 0.3925, 0.785, 1.1775, 1.57], [1.15, 1.25, 1.325, 1.375, 1.375], 3))

    def __init__(self):
        """Constructor."""
        self.ident = None
        self.world = None
        self.perceptions = {}
        self.publishers = {}

    @staticmethod
    def __class_from_classname(class_name):
        """Return a class object from a class name."""
        module_string, _, class_string = class_name.rpartition('.')
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
        return object_y < -1.07 or object_y > 1.07 or object_x < 0.35 or object_x > 1.27

    @classmethod
    def ball_is_small(cls, rad):
        """Return True if the ball is small, False if it is big. Right now, small is 0.03 and big 0.07."""
        return rad <= 0.05

    @classmethod
    def object_pickable_withtwohands(cls, dist, ang):
        """Return True if the object is in a place where it can be picked with two hands."""
        return abs(ang) <= 0.3925 and dist >= 0.46 and dist <= 0.75

    @classmethod
    def __send_object_twohandsreachable(cls, dist):
        """Calculate the coordinates of the object when moving it to a place where it can be picked with two hands."""
        x_coord = 0
        y_coord = dist
        if y_coord < 0.47:
            y_coord = 0.47
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

    def __update_goal_sensors(self):
        """Update goal sensors' values."""
        if (self.perceptions['ball_dist'] == self.perceptions['box_dist']) and (self.perceptions['ball_ang'] == self.perceptions['box_ang']):
            self.perceptions['ball_in_box'] = True
        else:
            self.perceptions['ball_in_box'] = False
        dist, ang = self.calculate_closest_position(self.perceptions['ball_ang'])
        if (self.perceptions['ball_dist'] == dist) and (self.perceptions['ball_ang'] == ang):
            self.perceptions['ball_with_robot'] = True
        else:
            self.perceptions['ball_with_robot'] = False

    def __random_perceptions(self):
        """Randomize the state of the environment."""
        # Sizes
        if numpy.random.uniform() > 0.3:
            self.perceptions['ball_size'] = 0.03
            self.perceptions['box_size'] = 0.12
        else:
            self.perceptions['ball_size'] = 0.07
            self.perceptions['box_size'] = 0.12
        # Ball position
        valid = False
        while not valid:
            ball_y = numpy.random.uniform(low=-1.07, high=1.07)
            ball_x = numpy.random.uniform(low=0.37, high=1.29)
            self.perceptions['ball_dist'] = numpy.linalg.norm([ball_y, ball_x])
            self.perceptions['ball_ang'] = numpy.arctan2(ball_y, ball_x)
            valid = not self.object_too_close(self.perceptions['ball_dist'], self.perceptions['ball_ang'])
        # Box position
        valid = False
        while not valid:
            box_y = numpy.random.uniform(low=-1.07, high=1.07)
            box_x = numpy.random.uniform(low=0.37, high=1.29)
            self.perceptions['box_dist'] = numpy.linalg.norm([box_y, box_x])
            self.perceptions['box_ang'] = numpy.arctan2(box_y, box_x)
            valid = not self.object_too_close(self.perceptions['box_dist'], self.perceptions['box_ang'])
        # Hands
        self.perceptions['ball_in_left_hand'] = False
        self.perceptions['ball_in_right_hand'] = False
        # Goal sensors
        self.__update_goal_sensors()

    def grasp_object_policy(self):
        """Grasp an object with a gripper."""
        if (
                self.world == 'gripper_and_low_friction' and
                (not self.object_too_far(self.perceptions['ball_dist'], self.perceptions['ball_ang'])) and
                self.ball_is_small(self.perceptions['ball_size']) and
                (not self.perceptions['ball_in_left_hand']) and
                (not self.perceptions['ball_in_right_hand'])
            ): # yapf: disable
            if self.perceptions['ball_ang'] > 0.0:
                self.perceptions['ball_in_left_hand'] = True
            else:
                self.perceptions['ball_in_right_hand'] = True

    def grasp_with_two_hands_policy(self):
        """Grasp an object using both arms."""
        if (
                self.object_pickable_withtwohands(self.perceptions['ball_dist'], self.perceptions['ball_ang']) and
                (
                    (self.world == 'no_gripper_and_high_friction') or
                    (not self.ball_is_small(self.perceptions['ball_size']))
                ) and
                (not self.perceptions['ball_in_left_hand']) and
                (not self.perceptions['ball_in_right_hand'])
            ): # yapf: disable
            self.perceptions['ball_in_left_hand'] = True
            self.perceptions['ball_in_right_hand'] = True

    def change_hands_policy(self):
        """Exchange an object from one hand to the other one."""
        if self.perceptions['ball_in_left_hand'] and (not self.perceptions['ball_in_right_hand']):
            self.perceptions['ball_in_left_hand'] = False
            self.perceptions['ball_in_right_hand'] = True
            self.perceptions['ball_ang'] = -self.perceptions['ball_ang']
        elif (not self.perceptions['ball_in_left_hand']) and self.perceptions['ball_in_right_hand']:
            self.perceptions['ball_in_left_hand'] = True
            self.perceptions['ball_in_right_hand'] = False
            self.perceptions['ball_ang'] = -self.perceptions['ball_ang']

    def sweep_object_policy(self):
        """Sweep an object to the front of the robot."""
        if (
                (not self.object_too_far(self.perceptions['ball_dist'], self.perceptions['ball_ang'])) and
                (not self.perceptions['ball_in_left_hand']) and
                (not self.perceptions['ball_in_right_hand'])
            ): # yapf: disable
            if (
                    (self.world == 'no_gripper_and_high_friction') or
                    (not self.ball_is_small(self.perceptions['ball_size']))
                ): # yapf: disable
                self.perceptions['ball_dist'], self.perceptions['ball_ang'] = self.__send_object_twohandsreachable(self.perceptions['ball_dist'])
            else:
                self.perceptions['ball_ang'] = -numpy.sign(self.perceptions['ball_ang']) * 0.4

    def put_object_in_box_policy(self):
        """Put an object into the box."""
        if (
                (not self.object_too_far(self.perceptions['box_dist'], self.perceptions['box_ang'])) and
                (
                    ((self.perceptions['box_ang'] > 0.0) and self.perceptions['ball_in_left_hand']) or
                    ((self.perceptions['box_ang'] <= 0.0) and self.perceptions['ball_in_right_hand'])
                )
            ): # yapf: disable
            self.perceptions['ball_dist'] = self.perceptions['box_dist']
            self.perceptions['ball_ang'] = self.perceptions['box_ang']
            self.perceptions['ball_in_left_hand'] = False
            self.perceptions['ball_in_right_hand'] = False

    def put_object_in_robot_policy(self):
        """Put an object as close to the robot as possible."""
        if self.perceptions['ball_in_left_hand'] or self.perceptions['ball_in_right_hand']:
            self.perceptions['ball_dist'], self.perceptions['ball_ang'] = self.calculate_closest_position(self.perceptions['ball_ang'])
            self.perceptions['ball_in_left_hand'] = False
            self.perceptions['ball_in_right_hand'] = False

    def throw_policy(self):
        """Throw an object."""
        if self.perceptions['ball_in_left_hand'] or self.perceptions['ball_in_right_hand']:
            if (
                    self.object_too_far(self.perceptions['box_dist'], self.perceptions['box_ang']) and
                    (
                        (self.perceptions['box_ang'] > 0.0 and self.perceptions['ball_in_left_hand']) or
                        (self.perceptions['box_ang'] <= 0.0 and self.perceptions['ball_in_right_hand'])
                    )
                ): # yapf: disable
                self.perceptions['ball_dist'] = self.perceptions['box_dist']
                self.perceptions['ball_ang'] = self.perceptions['box_ang']
            else:
                self.perceptions['ball_dist'], self.perceptions['ball_ang'] = self.__send_object_outofreach(self.perceptions['ball_ang'])
            self.perceptions['ball_in_left_hand'] = False
            self.perceptions['ball_in_right_hand'] = False

    def ask_nicely_policy(self):
        """Ask someone to bring the object closer to us."""
        if self.object_too_far(self.perceptions['ball_dist'], self.perceptions['ball_ang']):
            self.perceptions['ball_dist'] = 1.13

    def __new_command_callback(self, data):
        """Process a command."""
        rospy.logdebug('Command received...')
        self.world = data.world
        self.__random_perceptions()
        for ident, publisher in self.publishers.iteritems():
            rospy.logdebug('Publishing ' + ident + ' = ' + str(self.perceptions[ident]))
            publisher.publish(self.perceptions[ident])

    def __new_action_callback(self, data):
        """Whenever a policy is selected to be executed by the LTM, the simulator executes it and publishes the new perceptions."""
        rospy.logdebug('Executing policy %s...', data.data)
        getattr(self, data.data + '_policy')()
        self.__update_goal_sensors()
        for ident, publisher in self.publishers.iteritems():
            rospy.logdebug('Publishing ' + ident + ' = ' + str(self.perceptions[ident]))
            # pdb.set_trace()
            publisher.publish(self.perceptions[ident])

    def __configure_sensors(self, sensors):
        """Configure the ROS publishers where publish sensor values."""
        for sensor in sensors:
            self.perceptions[sensor['id']] = 0
            topic = rospy.get_param(sensor['ros_name_prefix'] + '_topic')
            message = self.__class_from_classname(rospy.get_param(sensor['ros_name_prefix'] + '_msg'))
            rospy.logdebug('I will publish to %s...', topic)
            self.publishers[sensor['id']] = rospy.Publisher(topic, message, latch=True, queue_size=None)

    def __configure_simulation(self, simulation):
        """Configure the ROS topic where listen for commands to be executed."""
        self.ident = simulation['id']
        topic = rospy.get_param(simulation['ros_name_prefix'] + '_topic')
        message = self.__class_from_classname(rospy.get_param(simulation['ros_name_prefix'] + '_msg'))
        rospy.logdebug('Subscribing to %s...', topic)
        rospy.Subscriber(topic, message, callback=self.__new_command_callback)
        topic = rospy.get_param(simulation['executed_policy_prefix'] + '_topic')
        message = self.__class_from_classname(rospy.get_param(simulation['executed_policy_prefix'] + '_msg'))
        rospy.logdebug('Subscribing to %s...', topic)
        rospy.Subscriber(topic, message, callback=self.__new_action_callback)

    def __load_configuration(self, log_level, config_file):
        """Load configuration from a file."""
        rospy.init_node('ltm_simulator', log_level=getattr(rospy, log_level))
        if config_file is None:
            rospy.logerr('No configuration file for the LTM simulator specified!')
        else:
            if not os.path.isfile(config_file):
                rospy.logerr(config_file + ' does not exist!')
            else:
                rospy.loginfo('Loading configuration from %s...', config_file)
                config = yaml.load(open(config_file, 'r'), Loader=yaml.CLoader)
                self.__configure_sensors(config['Simulator']['Sensors'])
                # Be ware, we can not subscribe to control channel before creating all sensor publishers.
                self.__configure_simulation(config['Control'])

    def run(self, log_level='INFO', config_file=None, **kwargs):
        """Start the LTM simulator."""
        self.__load_configuration(log_level, config_file)
        rospy.loginfo('Starting LTM Simulator...')
        rospy.spin()
        rospy.loginfo('Ending LTM Simulator...')
