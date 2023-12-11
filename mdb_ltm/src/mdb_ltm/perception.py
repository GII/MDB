"""
MDB.

https://github.com/GII/MDB
"""

# Standard imports
import threading

# Library imports
import rospy

# MDB imports
from mdb_ltm.node import Node


class Perception(Node):
    """A perception. Its content cames from a sensor or a redescription and it is stored in a memory."""

    def __init__(self, data=None, **kwargs):
        """Init attributes when a new object is created."""
        super(Perception, self).__init__(**kwargs)
        # Init data storage attributes
        self.old_raw = 0.0
        self.raw = 0.0
        self.old_value = 0.0
        self.value = 0.0
        self.normalize_values = data
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
        rospy.Subscriber(
            self.data_topic, self.data_message, callback=self.read_callback
        )

    def calc_activation(self, perception=None):
        """Calculate the new activation value."""
        rospy.logerr(
            "Someone call calc_activation on a perception, this should not happen!!!"
        )

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
        if isinstance(self.raw.data, list):
            for perception in self.raw.data:
                distance = (
                    perception.distance - self.normalize_values["distance_min"]
                ) / (
                    self.normalize_values["distance_max"]
                    - self.normalize_values["distance_min"]
                )
                angle = (perception.angle - self.normalize_values["angle_min"]) / (
                    self.normalize_values["angle_max"]
                    - self.normalize_values["angle_min"]
                )
                diameter = (
                    perception.diameter - self.normalize_values["diameter_min"]
                ) / (
                    self.normalize_values["diameter_max"]
                    - self.normalize_values["diameter_min"]
                )
                self.value.append(
                    dict(
                        distance=distance,
                        angle=angle,
                        diameter=diameter,
                        # id=perception.id,
                    )
                )
        else:
            self.value.append(dict(data=self.raw.data))

    def read(self):
        """Obtain a new value for the sensor / redescription."""
        self.flag.wait()
        self.flag.clear()
        return self.value
