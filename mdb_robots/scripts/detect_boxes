#!/usr/bin/env python3
"""
MDB.

Available from https://github.com/robotsthatdream/MDB
Distributed under GPLv3.
"""

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import *  # noqa
import sys
import skimage
from mdb_robots.baxter import Baxter


if __name__ == "__main__":
    _, image = Baxter.detect_objects(skimage.io.imread(sys.argv[1]))
    skimage.io.imshow(image)
    skimage.io.show()
