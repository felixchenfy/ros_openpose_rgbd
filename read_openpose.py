import numpy as np
import time
import cv2
import rospy

from geometry_msgs.msg import Point
if True:  # Add project root
    import sys
    import os
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

    from utils.lib_rgbd import CameraInfo

''' ------------------------------- Settings ------------------------------- '''
CAMERA_INFO_FILE_PATH = ROOT + "config/cam_params_realsense.json"
CAMERA_INFO = CameraInfo(CAMERA_INFO_FILE_PATH)

''' ------------------------------- Classes ------------------------------- '''


class RgbdImage(object):
    def __init__(self, depth, camera_info=CAMERA_INFO, depth_unit=0.001, color=None):
        '''
        Arguments:
            depth {image}: depth image of type np.uint16.
            camera_info {CameraInfo}:
            depth_unit {float}: depth[i,j]*depth_unit is it's real depth in meters.
            color {image}.
        '''
        self._depth = depth.astype(np.float32) * depth_unit
        self._camera_info = camera_info
        self._color = color
        self._row, self._col, self._fx, self._fy, self._cx, self._cy = camera_info.get_cam_params()



    def get_3d_pos(self, x, y):
        '''
        Get the 3d position of the pixel (y, x) from depth image.
        The pixel is at yth row and xth column.
        '''
        row, col = self._01xy_to_row_col(x, y)
        d = self._depth[row, col]
        xyz = [
            (col - self._cx)*d/self._fx,
            (row - self._cy)*d/self._fy,
            d]
        return xyz

    def is_depth_valid(self, x, y):
        row, col = self._01xy_to_row_col(x, y)
        return self._depth[row, col] >= 0.00001

    def _01xy_to_row_col(self, x, y):
        row, col = round(self._row * y), round(self._col * x)
        return row, col

class Body(object):
    def __init__(self, id, rgbd, joints):
        '''
        Arguments:
            joints {2D array. 18x3}:
                Each row represents a joint. See `doc/keypoints_pose_coco_18.png`.
                The three columns are: x, y, confidence.
        '''
        pass


class Hand(object):
    def __init__(self, id, rgbd, joints):
        '''
        Arguments:
            joints {2D array. 18x3}:
                Each row represents a joint. See `doc/keypoints_pose_coco_18.png`.
                The three columns are: x, y, confidence.
        '''
        pass


class Human(object):
    _cnt_all_humans = 0

    def __init__(self, rgbd, body_joints, hand_joints):
        Human._cnt_all_humans += 1
        self._id = Human._cnt_all_humans
        self.set_joints(rgbd, body_joints, hand_joints)
        pass

    def set_joints(self, rgbd, body_joints, hand_joints):
        self._body = Body(self._id, rgbd, body_joints)
        self._left_hand = Hand(self._id, rgbd, hand_joints[0])
        self._right_hand = Hand(self._id, rgbd, hand_joints[1])

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
        return [self._Pointer(self.left_hand), self._Pointer(self.right_hand))]

    def draw_Pointer_direction(self):
        ''' Draw a line onto rviz. '''
        left_pointer, right_pointer=self.get_Pointer_directions()
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
        self.t0=time.time()

    def report_time(self, str_msg):
        t=time.time() - self.t0
        t="{:.3f}".format(t)
        print("'{}' takes {} seconds".format(str_msg, t))

    def report_time_and_reset(self, str_msg):
        self.report_time(str_msg)
        self.reset()

    def reset(self):
        self.t0=time.time()


''' -------------------------------------- Settings -------------------------------------- '''

SRC_FOLDER=ROOT + "output/"
SRC_POSE_FILE=SRC_FOLDER + "body_joints.npy"
SRC_HAND_FILE=SRC_FOLDER + "hand_joints.npy"


def read_joints_of_an_image():
    body_joints=np.load(SRC_POSE_FILE)
    hand_joints=np.load(SRC_HAND_FILE)
    return body_joints, hand_joints


def test_data_reading_speed():
    t0=time.time()
    test_times=100
    for i in range(test_times):
        body_joints, hand_joints=read_joints_of_an_image()
        # print("body_joints: ", body_joints)
        img=cv2.imread(
            "data/two_images/COCO_val2014_000000000328.jpg", cv2.IMREAD_COLOR)
    print("Read one image data of 3 people takes {}s.".format(
        (time.time()-t0)/test_times))


def read_next_data():
    body_joints, hand_joints=read_joints_of_an_image()
    color=cv2.imread(
        "data/two_images/COCO_val2014_000000000328.jpg", cv2.IMREAD_COLOR)
    depth=np.zeros(color.shape[0:2], np.uint16) + 1000  # 1m
    rgbd=RgbdImage(depth, color = color)
    return rgbd, body_joints, hand_joints


def main():
    rate=rospy.Rate(1)
    prev_humans=[]
    while not rospy.is_shutdown():
        rgbd, body_joints, hand_joints=read_next_data()
        N_people=len(body_joints)
        humans=[Human(rgbd, body_joints[i], hand_joints[i])
                  for i in range(N_people)]
        for human in humans:
            human.draw_rviz()
        for human in prev_humans:
            human.delete_rviz()
        prev_humans= humans
        rate.sleep()
