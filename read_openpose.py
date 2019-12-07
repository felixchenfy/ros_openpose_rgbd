import numpy as np
import time
import cv2

if True:  # Add project root
    import sys
    import os
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/'


class Timer(object):
    def __init__(self):
        self.t0 = time.time()

    def report_time(self, str_msg):
        t = time.time() - self.t0
        t = "{:.3f}".format(t)
        print("'{}' takes {} seconds".format(str_msg, t))

    def report_time_and_reset(self, str_msg):
        self.report_time(str_msg)
        self.reset()

    def reset(self):
        self.t0 = time.time()


''' -------------------------------------- Settings -------------------------------------- '''

SRC_FOLDER = ROOT + "output/"
SRC_POSE_FILE = SRC_FOLDER + "body_joints.npy"
SRC_HAND_FILE = SRC_FOLDER + "hand_joints.npy"


def read_joints_of_an_image():
    body_joints = np.load(SRC_POSE_FILE)
    hand_joints = np.load(SRC_HAND_FILE)
    return body_joints, hand_joints


def test_data_reading_speed():
    t0 = time.time()
    test_times = 100
    for i in range(test_times):
        body_joints, hand_joints = read_joints_of_an_image()
        # print("body_joints: ", body_joints)
        img = cv2.imread(
            "data/two_images/COCO_val2014_000000000328.jpg", cv2.IMREAD_COLOR)
    print("Read one image data of 3 people takes {}s.".format(
        (time.time()-t0)/test_times))

test_data_reading_speed()
