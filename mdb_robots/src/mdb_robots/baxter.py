"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import numpy
import skimage
import skimage.morphology # it doesn't work in Ubuntu 14.04 without this!!!
import skimage.feature
import rospy
from mdb_robots.robot import Robot


class Baxter(Robot):
    """ROS node to provide Baxter sensorization to MDB."""

    def __init__(self, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.ident = "Baxter"

    def random_perceptions(self):
        """Randomize the state of the environment."""
        # Wait for the human been...

    @staticmethod
    def detect_objects(image):
        """Try to find colour objects in a white table."""
        # RGB => Gray
        gray_image = skimage.color.rgb2gray(image)
        # Binarize using a threshold
        binarized_image = (gray_image < skimage.filters.threshold_mean(gray_image)).astype(float)
        # Dilate the image to remove artifacts and fill gaps
        binarized_image = skimage.morphology.dilation(binarized_image, selem=skimage.morphology.star(5))
        # Paint the space outside the table black
        dim = binarized_image.shape
        # Be ware! Due to flood_fill, this needs scikit-image 0.15.0
        #binarized_image = skimage.morphology.flood_fill(binarized_image, seed_point=(0, 0), new_value=0)
        #binarized_image = skimage.morphology.flood_fill(
        #    binarized_image, seed_point=(dim[0] - 1, dim[1] - 1), new_value=0
        #)
        # Look for blobs. This works better with the gray image than with the binarized image
        blobs = skimage.feature.blob_doh(gray_image, threshold=0.006)
        # Paint red circles around the blobs
        undesired_blobs = []
        for idx, blob in enumerate(blobs):
            row, col, radius = [int(round(x)) for x in blob]
            if binarized_image[row, col]:
                rows, cols = skimage.draw.circle_perimeter(row, col, radius, shape=image.shape)
                image[rows, cols] = (255, 0, 0)
            else:
                undesired_blobs.append(idx)
        return numpy.delete(blobs, undesired_blobs, 0), image

    def update_objects(self, blobs):
        """Update detected objects."""
        self.perceptions["cylinders"].data = []
        self.perceptions["boxes"].data = []
        for blob in blobs:
            row, col, radius = [int(round(x)) for x in blob]
            if radius > 0:  # TODO
                detected_object = self.base_messages["boxes"]()
                detected_object.distance = 0  # TODO
                detected_object.angle = 0  # TODO
                detected_object.diameter = 0  # TODO
                self.perception["boxes"].data.append(detected_object)
            else:
                detected_object = self.base_messages["cylinders"]()
                detected_object.distance = 0  # TODO
                detected_object.angle = 0  # TODO
                detected_object.diameter = 0  # TODO
                self.perception["cylinders"].data.append(detected_object)

    def update_monitor_image(self, image):
        """Update stored image (contains circles around detected objects)."""
        self.perceptions["monitor_image"] = image

    def new_camera_value_callback(self, data):
        """Process a new sensor reading."""
        rospy.logdebug("Camera image received...")
        start_time = rospy.get_time()
        blobs, image = self.detect_objects(
            numpy.fromstring(data.data, dtype=numpy.uint8).reshape(data.height, data.width, 3)
        )
        data.data = numpy.array_str(image.reshape(data.width)).strip("[]").replace("\n", "")
        self.update_objects(blobs)
        self.update_monitor_image(data)
        rospy.logdebug("Camera image processed (" + str(rospy.get_time() - start_time) + " seconds)")
