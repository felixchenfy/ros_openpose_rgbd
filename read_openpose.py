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

    from utils.lib_rgbd import RgbdImage, CameraInfo
    from utils.lib_ros_marker import MarkerDrawer

''' ------------------------------- Settings ------------------------------- '''


# -- Input human joints.
SRC_FOLDER = ROOT + "output/"
SRC_POSE_FILE = SRC_FOLDER + "body_joints.npy"
SRC_HAND_FILE = SRC_FOLDER + "hand_joints.npy"

# -- Input camera info and depth image.
CAMERA_INFO_FILE_PATH = ROOT + "config/cam_params_realsense.json"
CAMERA_INFO = CameraInfo(CAMERA_INFO_FILE_PATH)
DEPTH_UNIT = 0.001  # Unit: meter.

# -- Tf frames.
BASE_FRAME = "base"
CAMERA_FRAME = "camera_link"
CAMERA_POSE = np.array([  # 3x4 Transformation matrix.
    [0.,  0.,  1., 0.],  # Camera faces +x direction.
    [-1., 0.,  0., 0.],
    [0.,  -1., 0., 0.],
    [0.,  0., 0., 1.],
])

# -- Visualization in RVIZ.
rviz_drawer = MarkerDrawer(
    frame_id=BASE_FRAME, topic_name="visualization_marker")
LINK_SIZE = 0.003
LINK_COLOR = 'g'
IS_DRAW_DOTS = False  # This slows down rviz and makes it unstable.
DOT_SIZE = 0.01
DOT_COLOR = 'r'

''' ------------------------------- Classes ------------------------------- '''


class Link(object):
    def __init__(self, xyz1, xyz2):
        self.xyz1, self.xyz2 = xyz1, xyz2


class AbstractPart(object):
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
            rviz_drawer.draw_links(
                curr_id, link, size=LINK_SIZE, color=LINK_COLOR)
            curr_id += 1
            self._marker_ids.append(curr_id)
        if IS_DRAW_DOTS:
            for i, link in enumerate(self._links):
                rviz_drawer.draw_dots(
                    curr_id, link, size=DOT_SIZE, color=DOT_COLOR)
                curr_id += 1
                self._marker_ids.append(curr_id)

    def delete_rviz(self):
        for markder_id in self._marker_ids:
            rviz_drawer.delete_marker(markder_id)

    def _create_3d_joints(self, joints_2d):
        joints_xyz_in_camera = [self._rgbd.get_3d_pos(joint_2d_x, joint_2d_y)
                                for joint_2d_x, joint_2d_y, confidence in joints_2d]  # Nx3
        joints_xyz_in_world = CAMERA_POSE.dot(np.hstack(
            (np.array(joints_xyz_in_camera),
             np.ones((len(joints_2d), 1), np.float64))).T).T[:, 0:3]  # Nx3

        EPS = 0.0001

        def is_valid(i, joint_2d_x, joint_2d_y):
            valid_depth = joints_xyz_in_camera[i][-1] >= EPS
            valid_row_col = joint_2d_x >= EPS or joint_2d_y >= EPS
            return valid_depth and valid_row_col

        joints_validity = [is_valid(i, col_row_conf[0], col_row_conf[1])
                           for i, col_row_conf in enumerate(joints_2d)]

        return joints_xyz_in_world, joints_validity

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
        self._left_hand = Hand(id_l_hand, rgbd, hand_joints[0])
        self._right_hand = Hand(id_r_hand, rgbd, hand_joints[1])

        # Store all the above into a list.
        self._parts = [self._body, self._left_hand, self._right_hand]

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


def read_next_data():
    body_joints, hand_joints = read_joints_of_an_image()
    color = cv2.imread(
        "data/two_images/COCO_val2014_000000000328.jpg", cv2.IMREAD_COLOR)
    depth = np.zeros(color.shape[0:2], np.uint16) + 1000  # 1m
    rgbd = RgbdImage(depth, CAMERA_INFO, DEPTH_UNIT, color)
    return rgbd, body_joints, hand_joints


class CameraPosePublisher(object):
    def __init__(self):
        self._br = tf.TransformBroadcaster()
        self._p = CAMERA_POSE[0:3, 3]
        self._q = quaternion_from_matrix(CAMERA_POSE)
        # self._q = Quaternion(q[0], q[1], q[2], q[3])
        self.publish()

    def publish(self):
        self._br.sendTransform(self._p, self._q,
                               rospy.Time.now(),
                               CAMERA_FRAME,
                               BASE_FRAME)


''' -------------------------------------- Main -------------------------------------- '''


def main():

    camera_pose_publisher = CameraPosePublisher()
    rate = rospy.Rate(1.0)
    prev_humans = []
    while not rospy.is_shutdown():
        rgbd, body_joints, hand_joints = read_next_data()
        N_people = len(body_joints)

        humans = []
        for i in range(N_people):
            human = Human(rgbd, body_joints[i, :, :],
                          hand_joints[:, i, :, :])
            human.draw_rviz()
            rospy.loginfo("Drawing {}th person on rviz.".format(human._id))
            humans.append(human)

        for human in prev_humans:
            human.delete_rviz()
        prev_humans = humans
        camera_pose_publisher.publish()
        rate.sleep()


if __name__ == '__main__':
    node_name = "sub_rgbd_and_cloud"
    rospy.init_node(node_name)
    rospy.sleep(0.1)
    main()
    rospy.logwarn("Node `{}` stops.".format(node_name))
