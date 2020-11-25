"""
The shiny, all new, MDB 3.0.

Available from (we are still thinking about this...)
Copyright 2017 Richard J. Duro, Jose A. Becerra.
Distributed under the (yes, we are still thinking about this too...).
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import (  # noqa pylint: disable=unused-import
    bytes,
    dict,
    int,
    list,
    object,
    range,
    str,
    ascii,
    chr,
    hex,
    input,
    next,
    oct,
    open,
    pow,
    round,
    super,
    filter,
    map,
    zip,
)
import threading
from collections import OrderedDict
import rospy
from mdb_ltm.node import Node


class Perception(Node):
    """A perception. Its content cames from a sensor or a redescription and it is stored in a memory."""

    def __init__(self, **kwargs):
        """Init attributes when a new object is created."""
        super(Perception, self).__init__(**kwargs)
        # Init data storage attributes
        self.old_raw = 0.0
        self.raw = 0.0
        self.old_value = 0.0
        self.value = 0.0
        # Init thread syncronizing stuff
        self.semaphore = None
        self.flag = None
        self.init_threading()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = super().__getstate__()
        del state["semaphore"]
        del state["flag"]
        return state

    def init_threading(self):
        """Create needed stuff to synchronize threads."""
        self.semaphore = threading.Semaphore()
        self.flag = threading.Event()

    def init_ros(self):
        """Create publishers and make subscriptions."""
        super().init_ros()
        rospy.logdebug("Subscribing to %s...", self.data_topic)
        rospy.Subscriber(self.data_topic, self.data_message, callback=self.read_callback)

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        rospy.logerr("Someone call calc_activation on a perception, this should not happen!!!")

    def read_callback(self, reading):
        """Get sensor data from ROS topic."""
        self.semaphore.acquire()
        rospy.logdebug("Receiving " + self.ident + " = " + str(reading))
        self.old_raw = self.raw
        self.raw = reading
        self.old_value = self.value
        self.process_reading()
        self.flag.set()
        self.semaphore.release()

    def process_reading(self):
        """Process the new sensor reading."""
        self.value = []
        self.value.append(OrderedDict(data=self.raw.data))

    def read(self):
        """Obtain a new value for the sensor / redescription."""
        self.flag.wait()
        self.flag.clear()
        return self.value


class ObjectListPerception(Perception):
    """A perception corresponding with a list of objects."""

    def __init__(self, data=None, **kwargs):
        """Init attributes when a new object is created."""
        self.normalize_values = data
        super(ObjectListPerception, self).__init__(**kwargs)

    def process_reading(self):
        """Process the new sensor reading."""
        self.value = []
        for perception in self.raw.data:
            distance = (perception.distance - self.normalize_values["distance_min"]) / (
                self.normalize_values["distance_max"] - self.normalize_values["distance_min"]
            )
            angle = (perception.angle - self.normalize_values["angle_min"]) / (
                self.normalize_values["angle_max"] - self.normalize_values["angle_min"]
            )
            diameter = (perception.diameter - self.normalize_values["diameter_min"]) / (
                self.normalize_values["diameter_max"] - self.normalize_values["diameter_min"]
            )
            self.value.append(OrderedDict(distance=distance, angle=angle, diameter=diameter, id=perception.id))
