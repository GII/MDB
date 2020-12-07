"""
MDB.

https://github.com/GII/MDB
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals
from future import standard_library

standard_library.install_aliases()
from builtins import *
from builtins import object
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg


class display(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()

        self.display_pub = rospy.Publisher("/robot/xdisplay", Image, queue_size=1)

        self.exp_dict = {}
        self.read_expressions()

    def image_cv2ros(self, image):
        try:
            imageros = self.bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        return imageros

    def read_expressions(self):
        path_expressions = self.rospack.get_path("mdb_robots_policies") + "/face/"

        self.exp_dict["normal"] = cv2.imread(path_expressions + "face_normal_d.png", 1)
        self.exp_dict["focus"] = cv2.imread(path_expressions + "face_focused_d.png", 1)
        self.exp_dict["giveme"] = cv2.imread(path_expressions + "face_giveme_new.png", 1)
        self.exp_dict["horizon"] = cv2.imread(path_expressions + "face_horizon_d.png", 1)
        self.exp_dict["horizon2"] = cv2.imread(path_expressions + "face_horizon_2_d.png", 1)
        self.exp_dict["rewardb"] = cv2.imread(path_expressions + "rewardb.png", 1)
        self.exp_dict["rewardw"] = cv2.imread(path_expressions + "rewardw.png", 1)
        self.exp_dict["noreward"] = cv2.imread(path_expressions + "noreward2.png", 1)
        self.exp_dict["automatic"] = cv2.imread(path_expressions + "face_automatic_b.png", 1)
        self.exp_dict["confused"] = cv2.imread(path_expressions + "face_confused.png", 1)
        self.exp_dict["bigobj"] = cv2.imread(path_expressions + "big_obj.png", 1)
        self.exp_dict["smallobj"] = cv2.imread(path_expressions + "small_obj.png", 1)
        self.exp_dict["big_obj_grip"] = cv2.imread(path_expressions + "big_obj_grip_new.png", 1)
        self.exp_dict["big_obj_nogrip"] = cv2.imread(path_expressions + "big_obj_nogrip_new.png", 1)
        self.exp_dict["small_obj_grip"] = cv2.imread(path_expressions + "small_obj_grip_new.png", 1)
        self.exp_dict["small_obj_nogrip"] = cv2.imread(path_expressions + "small_obj_nogrip_new.png", 1)

    def changeDisplay(self, expression):
        self.display_pub.publish(self.image_cv2ros(self.exp_dict[expression]))
