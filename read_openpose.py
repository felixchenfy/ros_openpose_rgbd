import numpy as np
import time
import cv2
import rospy

if True:  # Add project root
    import sys
    import os
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/'


class RgbdImage(object):
    def __init__(self, depth, camera_info, color=None):
        self.depth = depth
        self.camera_info = camera_info
        self.color = color

    def get_3d_pos(self, x, y):
        '''
        Get the 3d position of the pixel (y, x) from depth image.
        The pixel is at yth row and xth column.
        '''

class Body(object):
    def __init__(self, joints):
        self.set_joints(joints)

    def set_joints(self, joints):
        '''
        joints {2D array. 18x3}:
            Each row represents a joint. See `doc/keypoints_pose_coco_18.png`.
            The three columns are: x, y, confidence.
        '''
        pass


class Hand(object):
    def __init__(self, joints):
        self.set_joints(joints)

    def set_joints(self, joints):
        '''
        joints {2D array. 18x3}:
            Each row represents a joint. See `doc/keypoints_pose_coco_18.png`.
            The three columns are: x, y, confidence.
        '''
        pass


class Human(object):
    _cnt_all_humans = 0

    def __init__(self, body_joints, hand_joints):
        Human._cnt_all_humans += 1
        self._id = Human._cnt_all_humans
        self.set_joints(body_joints, hand_joints)
        pass

    def set_joints(self, body_joints, hand_joints):
        self._body = Body(self._id, body_joints)
        self._left_hand = Hand(self._id, hand_joints[0])
        self._right_hand = Hand(self._id, hand_joints[1])

        # Store all the above into a list.
        self._parts = [self._body, self._left_hand, self._right_hand]
        self._is_displayed = False

    def draw_rviz(self):
        for part in self._parts:
            part.draw_rviz()
        self._is_displayed = True

    def delete_rviz(self):
        if self._is_displayed:
            for part in self._parts:
                part.delete_rviz()
        self._is_displayed = False

    def __del__(self):
        self.delete_rviz()

    def get_Pointer_directions(self):
        ''' Get where each hand is pointing to. '''
        return [self._Pointer(self.left_hand), self._Pointer(self.right_hand))

    def draw_Pointer_direction(self):
        ''' Draw a line onto rviz. '''
        left_pointer, right_pointer = self.get_Pointer_directions()
        left_pointer.draw_rviz()
        right_pointer.draw_rviz()

    def check_pointintg(self, objects):
        pass
        for obj in objects:
            if obj.is_pointed():
                obj.draw_rviz(True)
            else:
                obj.draw_rviz(False)

    class _Pointer(object):
        def __init__(self, hand):
            pass

        def draw_rviz(self):
            ''' Draw the pointing onto rviz. '''
            pass


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


def main():
    rate = rospy.Rate(1)
    prev_humans = []
    while not rospy.is_shutdown():
        body_joints, hand_joints = read_joints_of_an_image()
        N_people = len(body_joints)
        humans = [Human(body_joints[i], hand_joints[i])
                  for i in range(N_people)]
        for human in humans:
            human.draw_rviz()
        for human in prev_humans:
            human.delete_rviz()
        prev_humans = humans
        rate.sleep()
