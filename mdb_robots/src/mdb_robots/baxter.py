"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import numpy
from skimage import color, filters, morphology, feature, draw, io, util, transform
from scipy import ndimage
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

    @classmethod
    def detect_objects(cls, image):
        """Try to find colour objects in a white table."""
        # RGB => Gray
        gray_image = color.rgb2gray(image)
        # Improve contrast using a local method
        enhanced_image = filters.rank.enhance_contrast(util.img_as_ubyte(gray_image), selem=morphology.disk(2))
        # Detect edges using Canny + closing + fill holes (it misses part of the table's edge)
        edges = feature.canny(enhanced_image, sigma=2.3)
        closed_edges = ndimage.binary_closing(edges, iterations=5)
        binarized_image = ndimage.binary_fill_holes(closed_edges)
        io.imshow(binarized_image)
        io.show()
        # Detect edges using gradient + Otsu global thresholding (it misses some small objects)
        gradient = filters.rank.gradient(enhanced_image, selem=morphology.disk(3))
        binarized_image_2 = gradient > filters.threshold_otsu(gradient)
        io.imshow(binarized_image_2)
        io.show()
        # Combine both edge images
        combined_binarized_image = numpy.maximum(binarized_image, binarized_image_2)
        # Paint the space outside the table black. Due to flood_fill(), this needs scikit-image 0.15.0
        dim = combined_binarized_image.shape
        combined_binarized_image = morphology.flood_fill(combined_binarized_image, seed_point=(0, 0), new_value=255)
        combined_binarized_image = morphology.flood_fill(
            combined_binarized_image, seed_point=(dim[0] - 1, dim[1] - 1), new_value=255
        )
        io.imshow(combined_binarized_image)
        io.show()
        # Look for blobs. This works better with the gray image than with the binarized image
        blobs = feature.blob_dog(combined_binarized_image, threshold=1.0, exclude_border=1)
        # Paint red circles around the blobs
        for blob in blobs:
            row, col, sigma = [int(round(x)) for x in blob]
            rows, cols = draw.circle_perimeter(row, col, 7, shape=image.shape)
            image[rows, cols] = (255, 0, 0)
            lbp_code_red = feature.multiblock_lbp(transform.integral_image(image[:, :, 0]), row - 7, col - 7, 5, 5)
            lbp_code_green = feature.multiblock_lbp(transform.integral_image(image[:, :, 1]), row - 7, col - 7, 5, 5)
            lbp_code_blue = feature.multiblock_lbp(transform.integral_image(image[:, :, 2]), row - 7, col - 7, 5, 5)
            print(
                str(row)
                + "\t"
                + str(col)
                + "\t"
                + str(lbp_code_red)
                + "\t"
                + str(lbp_code_green)
                + "\t"
                + str(lbp_code_blue)
            )
        return blobs, image

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
