'''
Visualize multiple people's 3D joints in rviz.

Method:
1. Read in 2d human joints from Openpose.
2. Read in a depth image.
3. Create 3d joints.
4. Visualize them in rviz by ROS markers.
'''

import numpy as np
import time
import cv2
import rospy
import tf
from tf.transformations import quaternion_from_matrix

from abc import ABCMeta, abstractmethod  # Abstract class.
from geometry_msgs.msg import Point, Quaternion, Pose

if True:  # Add project root
    import sys
    import os
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

    from utils.lib_rgbd import RgbdImage, MyCameraInfo
    from utils.lib_rviz_marker import RvizMarker

''' ------------------------------- DEBUG SETTINGS ------------------------------- '''
IS_DRAW_DOTS = False  # This slows down rviz and makes it unstable.

''' ------------------------------- Classes ------------------------------- '''


class Link(object):
    ''' A link connects to two joints. '''

    def __init__(self, xyz1, xyz2):
        self.xyz1, self.xyz2 = xyz1, xyz2


class AbstractPart(object):
    ''' Base class for `class Body` and `class Hand`. '''
    __metaclass__ = ABCMeta
    _N_LINKS = 0
    _LINKS_TABLE = []

    def __init__(self, id, rgbd, joints):
        '''
        Arguments:
            joints {2D array}:
                shape = (_N_LINKS, 3)
                Each row represents a joint.
                See `doc/keypoints_pose_coco_18.png` or `doc/keypoints_hand.png`.
        '''
        self._id = id
        self._rgbd = rgbd
        self._links = self._create_links(joints)
        self._marker_ids = []

    def draw_rviz(self):
        curr_id = self._id
        for i, link in enumerate(self._links):
            RvizMarker.draw_links(curr_id, link)
            self._marker_ids.append(curr_id)
            curr_id += 1
        if IS_DRAW_DOTS:
            for i, link in enumerate(self._links):
                RvizMarker.draw_dots(curr_id, link)
                self._marker_ids.append(curr_id)
                curr_id += 1

    def delete_rviz(self):
        for markder_id in self._marker_ids:
            RvizMarker.delete_marker(markder_id)

    def _create_links(self, joints_2d):
        '''
        Return:
            valid_links {list}:
                Each element is a link.
                A link is a list of joint positions.
        '''
        joints_xyz_in_world, joints_validity = self._create_3d_joints(
            joints_2d)
        valid_links = []
        for joints_indices in self._LINKS_TABLE:
            ith_link = []
            for joint_idx in joints_indices:
                if joints_validity[joint_idx]:
                    ith_link.append(joints_xyz_in_world[joint_idx])
                else:
                    break
            if len(ith_link) >= 2:
                valid_links.append(ith_link)
        return valid_links

    def _create_3d_joints(self, joints_2d):
        joints_xyz_in_camera = [self._rgbd.get_3d_pos(joint_2d_x, joint_2d_y)
                                for joint_2d_x, joint_2d_y, confidence in joints_2d]  # Nx3

        def change_joints_coordinate_to_world():
            T_camera = self._rgbd.camera_pose()  # 4x4
            P_joints = np.hstack(  # Nx3 --> Nx4
                (np.array(joints_xyz_in_camera),
                 np.ones((len(joints_2d), 1), np.float64))).T
            return (T_camera.dot(P_joints)).T[:, 0:3]  # Nx3
        joints_xyz_in_world = change_joints_coordinate_to_world()

        EPS = 0.0001

        def is_valid(i, joint_2d_x, joint_2d_y):
            valid_depth = joints_xyz_in_camera[i][-1] >= EPS
            valid_row_col = joint_2d_x >= EPS or joint_2d_y >= EPS
            return valid_depth and valid_row_col

        joints_validity = [is_valid(i, col_row_conf[0], col_row_conf[1])
                           for i, col_row_conf in enumerate(joints_2d)]

        return joints_xyz_in_world, joints_validity


class Body(AbstractPart):
    _N_LINKS = 18
    _LINKS_TABLE = [
        [0, 1, 2, 3, 4],
        [1, 5, 6, 7],
        [1, 8, 9, 10],
        [1, 11, 12, 13],
        [0, 14, 16],
        [0, 15, 17],
    ]
    _LINKS_TABLE_SINGLE = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 4],
        [1, 5],
        [5, 6],
        [6, 7],
        [1, 8],
        [8, 9],
        [9, 10],
        [1, 11],
        [11, 12],
        [12, 13],
        [0, 14],
        [14, 16],
        [0, 15],
        [15, 17],
    ]

    def __init__(self, *args):
        super(Body, self).__init__(*args)


class Hand(AbstractPart):
    _N_LINKS = 21
    _LINKS_TABLE = [
        [0, 1, 2, 3, 4],
        [0, 5, 6, 7, 8],
        [0, 9, 10, 11, 12],
        [0, 13, 14, 15, 16],
        [0, 17, 18, 19, 20]
    ]
    _LINKS_TABLE_SINGLE = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 4],
        [0, 5],
        [5, 6],
        [6, 7],
        [7, 8],
        [0, 9],
        [9, 10],
        [10, 11],
        [11, 12],
        [0, 13],
        [13, 14],
        [14, 15],
        [15, 16],
        [0, 17],
        [17, 18],
        [18, 19],
        [19, 20],
    ]

    def __init__(self, *args):
        super(Hand, self).__init__(*args)


class Human(object):
    _cnt_all_humans = 0

    def __init__(self, rgbd, body_joints, hand_joints):
        Human._cnt_all_humans += 1
        self._id = Human._cnt_all_humans
        self._has_displayed = False
        self.set_joints(rgbd, body_joints, hand_joints)
        pass

    def set_joints(self, rgbd, body_joints, hand_joints):

        GAP = 100  # Make id different in order to draw in rviz.
        id_body = self._id * GAP + 0
        id_l_hand = self._id * GAP + 30
        id_r_hand = self._id * GAP + 60
        self._body = Body(id_body, rgbd, body_joints)
        if hand_joints is not None:
            self._left_hand = Hand(id_l_hand, rgbd, hand_joints[0])
            self._right_hand = Hand(id_r_hand, rgbd, hand_joints[1])

        # Store all the above into a list.
        self._parts = [self._body]
        if hand_joints is not None:
            self._parts.extend([self._left_hand, self._right_hand])

        return

    def draw_rviz(self):
        for part in self._parts:
            part.draw_rviz()
            rospy.sleep(0.0001)
        self._has_displayed = True

    def delete_rviz(self):
        if self._has_displayed:
            for part in self._parts:
                part.delete_rviz()
        self._has_displayed = False

    # def __del__(self):
    #     self.delete_rviz()

    # def get_Pointer_directions(self):
    #     ''' Get where each hand is pointing to. '''
    #     return [self._Pointer(self.left_hand), self._Pointer(self.right_hand)]

    # def draw_Pointer_direction(self):
    #     ''' Draw a line onto rviz. '''
    #     left_pointer, right_pointer=self.get_Pointer_directions()
    #     left_pointer.draw_rviz()
    #     right_pointer.draw_rviz()

    # def check_pointintg(self, objects):
    #     pass
    #     for obj in objects:
    #         if obj.is_pointed():
    #             obj.draw_rviz(True)
    #         else:
    #             obj.draw_rviz(False)

    # class _Pointer(object):
    #     def __init__(self, hand):
    #         pass

    #     def draw_rviz(self):
    #         ''' Draw the pointing onto rviz. '''
    #         pass


''' -------------------------------------- Helper Functions -------------------------------------- '''


class CameraPosePublisher(object):
    def __init__(self, frame_id_camera, frame_id_world,
                 T4x4_world_to_cam=np.identity(4)):
        self._br = tf.TransformBroadcaster()
        self.set_pose(T4x4_world_to_cam)
        self._frame_id_camera = frame_id_camera
        self._frame_id_world = frame_id_world

    def set_pose(self, T4x4_world_to_cam):
        self._p = T4x4_world_to_cam[0:3, 3]
        self._q = quaternion_from_matrix(T4x4_world_to_cam)

    def publish(self, T4x4_world_to_cam=None):
        if T4x4_world_to_cam is not None:
            self.set_pose(T4x4_world_to_cam)
        self._br.sendTransform(self._p, self._q,
                               rospy.Time.now(),
                               self._frame_id_camera,
                               self._frame_id_world)


def set_default_params():
    ''' Set default parameters and class instances for testing,
    including: (1) camera pose, (2) camera pose publisher, (3) RvizMarker.
    '''
    # Camera pose.
    cam_pose = np.array([  # let camera faces +x direction.
        [0.,  0.,  1., 0.],
        [-1., 0.,  0., 0.],
        [0.,  -1., 0., 1.],
        [0.,  0., 0., 1.],
    ])

    # # To make the skeleton allign with point cloud,
    # # let's make the camera pose at origin.
    # # cam_pose = np.identity(4)

    # Camera pose publisher.
    base_frame = "base"
    cam_pose_pub = CameraPosePublisher(
        frame_id_camera="camera_link",
        frame_id_world=base_frame,
        T4x4_world_to_cam=cam_pose)

    # Rviz drawer.
    RvizMarker.init(frame_id=base_frame, topic_name="visualization_marker")
    RvizMarker.set_dot(size=0.1, color='r')
    RvizMarker.set_link(size=0.01, color='g')

    # Return.
    return cam_pose, cam_pose_pub


''' -------------------------------------- Unit Test -------------------------------------- '''


def test_visualize_3d_joints():
    '''
    1. Read in 2d human joints from Openpose.
    2. Read in a depth image.
    3. Create 3d joints.
    4. Visualize them in rviz by ROS markers.
    '''

    ''' -------------------- Settings -------------------- '''

    cam_pose, cam_pose_pub = set_default_params()

    def read_next_data():
        ''' Read color/depth image,
            camera info,
            and human body/hand joints.
        '''

        body_joints = np.load(ROOT+"data/image1/body_joints.npy")
        hand_joints = np.load(ROOT+"data/image1/hand_joints.npy")

        def read_img(p):
            return cv2.imread(p, cv2.IMREAD_UNCHANGED)
        color = read_img(ROOT+"data/image1/color/00083.png")
        depth = read_img(ROOT+"data/image1/depth/00083.png")

        camera_info = MyCameraInfo(
            camera_info_file_path=ROOT+"data/image1/cam_params_realsense.json")
        rgbd = RgbdImage(color, depth,
                         camera_info,
                         camera_pose=cam_pose,
                         depth_unit=0.001)
        return rgbd, body_joints, hand_joints

    ''' -------------------- Read data and draw -------------------- '''

    rate = rospy.Rate(1.0)
    prev_humans = []

    while not rospy.is_shutdown():

        rgbd, body_joints, hand_joints = read_next_data()
        N_people = len(body_joints)

        for human in prev_humans:
            # If I put delete after drawing new markders,
            # The delete doesn't work. I don't know why.
            human.delete_rviz()
        humans = []
        for i in range(N_people):
            human = Human(rgbd, body_joints[i, :, :],
                          hand_joints[:, i, :, :])
            human.draw_rviz()
            rospy.loginfo("Drawing {}th person on rviz.".format(human._id))
            humans.append(human)

        prev_humans = humans

        cam_pose_pub.publish()
        rate.sleep()

    # -- Delete marker (Which doens't work. I don't know why.)
    for human in prev_humans:
        human.delete_rviz()
    for human in humans:
        human.delete_rviz()
    rospy.sleep(2.0)


if __name__ == '__main__':
    node_name = "test_visualize_3d_joints"
    rospy.init_node(node_name)
    rospy.sleep(0.1)
    test_visualize_3d_joints()
    rospy.logwarn("Node `{}` stops.".format(node_name))
