"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import numpy
from skimage import color, filters, morphology, feature, draw, io, util, transform
from scipy import stats, ndimage
import pandas
import rospy
from mdb_robots.robot import Robot


class Baxter(Robot):
    """ROS node to provide Baxter sensorization to MDB."""

    def __init__(self, objects_database=None, **kwargs):
        """Constructor."""
        super().__init__(**kwargs)
        self.ident = "Baxter"
        if objects_database:
            self.objects = pandas.read_csv(objects_database, delim_whitespace=True, header=None)

    def random_perceptions(self):
        """Randomize the state of the environment."""
        # Wait for the human been...

    @staticmethod
    def multiblock_lbp(image, blobs):
        red = image[:, :, 0]
        green = image[:, :, 1]
        blue = image[:, :, 2]
        integral_red = transform.integral_image(red)
        integral_green = transform.integral_image(green)
        integral_blue = transform.integral_image(blue)
        for blob in blobs:
            row, col, sigma = [int(round(x)) for x in blob]
            rows, cols = draw.circle_perimeter(row, col, sigma, shape=image.shape)
            image[rows, cols] = (255, 0, 0)
            lbp_red = feature.multiblock_lbp(integral_red, row - 7, col - 7, 5, 5)
            lbp_green = feature.multiblock_lbp(integral_green, row - 7, col - 7, 5, 5)
            lbp_blue = feature.multiblock_lbp(integral_blue, row - 7, col - 7, 5, 5)
            print(str(row) + "\t" + str(col) + "\t" + str(lbp_red) + "\t" + str(lbp_green) + "\t" + str(lbp_blue))

    @staticmethod
    def lbp_histogram(image):
        lbp = feature.local_binary_pattern(image, P=24, R=3, method="uniform")
        n_bins = int(lbp.max()) + 1
        histogram, _ = numpy.histogram(lbp, bins=n_bins, range=(0, n_bins), density=True)
        return histogram

    @classmethod
    def lbp(cls, image, blobs):
        red = image[:, :, 0]
        green = image[:, :, 1]
        blue = image[:, :, 2]
        object_features = []
        for blob in blobs:
            row, col, sigma = [int(round(x)) for x in blob]
            rows, cols = draw.circle_perimeter(row, col, sigma, shape=image.shape)
            image[rows, cols] = (255, 0, 0)
            red_hist = cls.lbp_histogram(red[row - sigma : row + sigma + 1, col - sigma : col + sigma + 1])
            green_hist = cls.lbp_histogram(green[row - sigma : row + sigma + 1, col - sigma : col + sigma + 1])
            blue_hist = cls.lbp_histogram(blue[row - sigma : row + sigma + 1, col - sigma : col + sigma + 1])
            object_features.append(pandas.DataFrame([numpy.concatenate((blob[0:2], red_hist, green_hist, blue_hist))]))
        features = pandas.concat(object_features, ignore_index=True)
        return features

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
        # Detect edges using gradient + Otsu global thresholding (it misses some small objects)
        gradient = filters.rank.gradient(enhanced_image, selem=morphology.disk(3))
        binarized_image_2 = gradient > filters.threshold_otsu(gradient)
        # Combine both edge images
        combined_binarized_image = numpy.maximum(binarized_image, binarized_image_2)
        # Paint the space outside the table white. Due to flood_fill(), this needs scikit-image 0.15.0
        dim = combined_binarized_image.shape
        combined_binarized_image = morphology.flood_fill(combined_binarized_image, seed_point=(0, 0), new_value=255)
        combined_binarized_image = morphology.flood_fill(
            combined_binarized_image, seed_point=(dim[0] - 1, dim[1] - 1), new_value=255
        )
        # Look for blobs.
        blobs = feature.blob_dog(combined_binarized_image, threshold=1.0, exclude_border=1)
        # Paint red circles around the blobs and extract features
        features = cls.lbp(image, blobs)
        return image, blobs, features

    def classify_objects(self, image):
        image, _, features = self.detect_objects(image)
        n_color_attributes = features.shape[1]
        for _, detected_object in features.iterrows():
            red = detected_object[2 : 2 + n_color_attributes].to_list()
            green = detected_object[2 + n_color_attributes : 2 + 2 * n_color_attributes].to_list()
            blue = detected_object[2 + 2 * n_color_attributes : 2 + 3 * n_color_attributes].to_list()
            recognized_object = None
            score = float("inf")
            for _, object_type in self.objects.iterrows():
                reference_red = object_type[1 : 1 + n_color_attributes].to_list()
                reference_green = object_type[1 + n_color_attributes : 1 + 2 * n_color_attributes].to_list()
                reference_blue = object_type[1 + 2 * n_color_attributes : 1 + 3 * n_color_attributes].to_list()
                kullback_leibler_divergence_sum = (
                    stats.entropy(pk=reference_red, qk=red)
                    + stats.entropy(pk=reference_green, qk=green)
                    + stats.entropy(pk=reference_blue, qk=blue)
                )
                recognized_object, score = (
                    (object_type[0], kullback_leibler_divergence_sum)
                    if kullback_leibler_divergence_sum < score
                    else (recognized_object, score)
                )
            print(f"{recognized_object} {detected_object[0]} {detected_object[1]}")
        return image

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
