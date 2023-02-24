"""
MDB.

https://github.com/GII/MDB
"""

# Library imports
import numpy
from skimage import color, filters, morphology, feature, draw, io, util, transform
from scipy import ndimage
import pandas
import rospy

# MDB imports
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
        features_list = []
        max_sigma = int(round(numpy.max(blobs[:, 2])))
        for blob in blobs:
            row, col, _ = [int(round(x)) for x in blob]
            if row < 400 and col > 100:
                rows, cols = draw.circle_perimeter(row, col, max_sigma, shape=image.shape)
                image[rows, cols] = (255, 0, 0)
                red_hist = cls.lbp_histogram(
                    red[row - max_sigma : row + max_sigma + 1, col - max_sigma : col + max_sigma + 1]
                )
                green_hist = cls.lbp_histogram(
                    green[row - max_sigma : row + max_sigma + 1, col - max_sigma : col + max_sigma + 1]
                )
                blue_hist = cls.lbp_histogram(
                    blue[row - max_sigma : row + max_sigma + 1, col - max_sigma : col + max_sigma + 1]
                )
                features_list.append(
                    pandas.DataFrame([numpy.concatenate((blob[0:2], red_hist, green_hist, blue_hist))])
                )
        features = pandas.concat(features_list, ignore_index=True)
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
        io.imshow(combined_binarized_image)
        io.show()
        # Look for blobs.
        blobs = feature.blob_dog(combined_binarized_image, threshold=1.7, exclude_border=1)
        # Paint red circles around the blobs and extract features
        image = filters.gaussian(image, sigma=1.5)
        features = cls.lbp(image, blobs)
        return image, blobs, features

    def kullback_leibler_symmetrised_divergence(self, pk, qk):
        """
        Calculate Kullback-Leibler symmetrised divergence.

        Notes:
        -----
        - scipy.stats.entropy() is not used because if qk contains zeros the result is 'inf'!

        """
        non_zeros = numpy.logical_and(pk != 0, qk != 0)
        dkl_p_q = numpy.sum(pk[non_zeros] * numpy.log(pk[non_zeros] / qk[non_zeros]))
        dkl_q_p = numpy.sum(qk[non_zeros] * numpy.log(qk[non_zeros] / pk[non_zeros]))
        return dkl_p_q + dkl_q_p

    def classify_objects(self, image):
        """
        - infer_objects() or astype() is needed when a pandas.Series has originally columns with different types,
        what happens with pandas.read_csv(). Otherwise, dtypes are objects and numpy.log() fails.

        """
        image, _, features = self.detect_objects(image)
        features.sort_values(by=[0, 1], inplace=True)
        n_color_attributes = features.shape[1]
        for _, detected_object in features.iterrows():
            red = detected_object[2 : 2 + n_color_attributes].to_numpy()
            green = detected_object[2 + n_color_attributes : 2 + 2 * n_color_attributes].to_numpy()
            blue = detected_object[2 + 2 * n_color_attributes : 2 + 3 * n_color_attributes].to_numpy()
            recognized_object = None
            score = float("inf")
            for _, object_type in self.objects.iterrows():
                reference_red = object_type[1 : 1 + n_color_attributes]
                reference_red = reference_red.infer_objects().to_numpy()
                reference_green = object_type[1 + n_color_attributes : 1 + 2 * n_color_attributes]
                reference_green = reference_green.infer_objects().to_numpy()
                reference_blue = object_type[1 + 2 * n_color_attributes : 1 + 3 * n_color_attributes]
                reference_blue = reference_blue.infer_objects().to_numpy()
                kullback_leibler_symmetrised_divergence = (
                    self.kullback_leibler_symmetrised_divergence(pk=red, qk=reference_red)
                    + self.kullback_leibler_symmetrised_divergence(pk=green, qk=reference_green)
                    + self.kullback_leibler_symmetrised_divergence(pk=blue, qk=reference_blue)
                )
                recognized_object, score = (
                    (object_type[0], kullback_leibler_symmetrised_divergence)
                    if kullback_leibler_symmetrised_divergence < score
                    else (recognized_object, score)
                )
            print(
                recognized_object
                + "\t"
                + str(int(detected_object[0]))
                + "\t"
                + str(int(detected_object[1]))
                + "\t"
                + str(score)
            )
        return image

    def update_objects(self, blobs):
        """Update detected objects."""
        self.perceptions["cylinders"].data = []
        self.perceptions["boxes"].data = []
        for blob in blobs:
            _, _, radius = [int(round(x)) for x in blob]
            if radius > 0:  # TODO
                detected_object = self.base_messages["boxes"]()
                detected_object.distance = 0  # TODO
                detected_object.angle = 0  # TODO
                detected_object.diameter = 0  # TODO
                self.perceptions["boxes"].data.append(detected_object)
            else:
                detected_object = self.base_messages["cylinders"]()
                detected_object.distance = 0  # TODO
                detected_object.angle = 0  # TODO
                detected_object.diameter = 0  # TODO
                self.perceptions["cylinders"].data.append(detected_object)

    def update_monitor_image(self, image):
        """Update stored image (contains circles around detected objects)."""
        self.perceptions["monitor_image"] = image

    def new_camera_value_callback(self, data):
        """Process a new sensor reading."""
        rospy.logdebug("Camera image received...")
        start_time = rospy.get_time()
        image, blobs, _ = self.detect_objects(
            numpy.fromstring(data.data, dtype=numpy.uint8).reshape(data.height, data.width, 3)
        )
        data.data = numpy.array_str(image.reshape(data.width)).strip("[]").replace("\n", "")
        self.update_objects(blobs)
        self.update_monitor_image(data)
        rospy.logdebug("Camera image processed (" + str(rospy.get_time() - start_time) + " seconds)")
