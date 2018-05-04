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
        self.new_value = threading.Event()
        topic = rospy.get_param(ros_name_prefix + '_topic')
        message = self.class_from_classname(rospy.get_param(ros_name_prefix + '_msg'))
        rospy.logdebug('Subscribing to %s...', topic)
        rospy.Subscriber(topic, message, callback=self.read_callback)

    def read_callback(self, reading):
        """Get sensor data from ROS topic."""
        self.old_raw = self.raw
        self.raw = reading.data
        self.old_value = self.value
        if self.max is not None and self.min is not None:
            self.value = (self.raw - self.min) / (self.max - self.min)
        else:
            self.value = self.raw
        self.new_value.set()

    def read(self):
        """Obtain a new value for the sensor / redescription."""
        self.new_value.wait()
        self.new_value.clear()
        return self.value
