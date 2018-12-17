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
from mdb_ltm.node import Node


class Perception(Node):
    """A perception. Its content cames from a sensor or a redescription and it is stored in a memory."""

    def __init__(self, data=None, ros_name_prefix=None, **kwargs):
        """Constructor."""
        super(Perception, self).__init__(**kwargs)
        if data is not None:
            self.min = data[0]
            self.max = data[1]
        else:
            self.min = None
            self.max = None
        self.old_raw = 0.0
        self.raw = 0.0
        self.old_value = 0.0
        self.value = 0.0
        self.sensor_semaphore = None
        self.new_value = None
        self.init_threading()
        self.topic = rospy.get_param(ros_name_prefix + "_topic")
        self.message = self.class_from_classname(rospy.get_param(ros_name_prefix + "_msg"))
        self.init_ros()

    def __getstate__(self):
        """Return the object to be serialize with PyYAML as the result of removing the unpicklable entries."""
        state = self.__dict__.copy()
        del state["sensor_semaphore"]
        del state["new_value"]
        return state

    def init_threading(self):
        """Create needed stuff to synchronize threads."""
        self.sensor_semaphore = threading.Semaphore()
        self.new_value = threading.Event()

    def init_ros(self):
        """Create publishers and make subscriptions."""
        rospy.logdebug("Subscribing to %s...", self.topic)
        rospy.Subscriber(self.topic, self.message, callback=self.read_callback)

    def read_callback(self, reading):
        """Get sensor data from ROS topic."""
        self.sensor_semaphore.acquire()
        self.old_raw = self.raw
        self.raw = reading.data
        self.old_value = self.value
        if self.max is not None and self.min is not None:
            self.value = (self.raw - self.min) / (self.max - self.min)
        else:
            self.value = self.raw
        self.new_value.set()
        self.sensor_semaphore.release()

    def read(self):
        """Obtain a new value for the sensor / redescription."""
        self.new_value.wait()
        self.new_value.clear()
        return self.value
