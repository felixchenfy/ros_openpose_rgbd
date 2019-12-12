#!/usr/bin/env python
'''
A wrapper class `RvizMarker` for drawing dots or links on Rviz.

Unit test:
    python utils/lib_rviz_marker.py
    roslaunch ros_openpose_rgbd run_rviz.launch 
'''

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import rospy
import copy
import math
from geometry_msgs.msg import Point

COLORS = {  # [r, g, b, a]
    'r': ColorRGBA(1., 0., 0., 1.),  # red
    'g': ColorRGBA(0., 1., 0., 1.),  # green
    'b': ColorRGBA(0., 0., 1., 1.),  # blue
    'k': ColorRGBA(0., 0., 0., 1.),  # black
    'y': ColorRGBA(1., 1., 0., 1.),  # yellow
}


class VizProperty(object):
    ''' Visualization property of a marker. '''

    def __init__(self, size, color, lifetime):
        self.size, self.color, self.lifetime = \
            size, color, lifetime

    def set_params(self, size=None, color=None, lifetime=None):
        if size is not None:
            self.size = size
        if color is not None:
            self.color = color
        if lifetime is not None:
            self.lifetime = lifetime

    def get_params(self):
        return self.size, self.color, self.lifetime


class RvizMarker(object):

    # Reference: http://wiki.ros.org/rviz/DisplayTypes/Marker

    # -- Initialization
    _MARKER_TEMPLATE = None
    _pub = None  # Marker publisher.

    # -- Default display settings.
    _V_DOT = VizProperty(size=0.1, color='r', lifetime=-1)
    _V_LINK = VizProperty(size=0.01, color='g', lifetime=-1)

    # -- Functions.
    @staticmethod
    def init(frame_id="base",
             topic_name="visualization_marker"):

        RvizMarker._MARKER_TEMPLATE = RvizMarker._create_template_marker()
        RvizMarker._MARKER_TEMPLATE.header.frame_id = frame_id
        RvizMarker._pub = rospy.Publisher(topic_name, Marker, queue_size=400)
        rospy.loginfo("RvizMarker.init(): frame_id={}, topic_name={}".format(
            frame_id, topic_name))

    @staticmethod
    def set_dot(size=None, color=None, lifetime=None):
        RvizMarker._V_DOT.set_params(size, color, lifetime)

    @staticmethod
    def set_link(size=None, color=None, lifetime=None):
        RvizMarker._V_LINK.set_params(size, color, lifetime)

    @staticmethod
    def draw_dot(id, xyz):
        marker = copy.deepcopy(RvizMarker._MARKER_TEMPLATE)
        size, color, lifetime = RvizMarker._V_DOT.get_params()
        marker.id = id
        marker.type = marker.SPHERE
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color = COLORS[color]
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        RvizMarker._pub.publish(marker)

    @staticmethod
    def draw_link(id, xyz1, xyz2, _color=''):
        marker = copy.deepcopy(RvizMarker._MARKER_TEMPLATE)
        size, color, lifetime = RvizMarker._V_LINK.get_params()
        marker.id = id
        marker.type = marker.LINE_LIST
        marker.header.stamp = rospy.Time.now()
        marker.points = [Point(*xyz1), Point(*xyz2)]
        marker.scale.x = size
        marker.color = COLORS[_color] if _color else COLORS[color]
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        RvizMarker._pub.publish(marker)

    @staticmethod
    def draw_links(id, list_xyz):
        ''' If list_xyz=[p0, p1, p2, p3],
        then the links are [(p0, p1), (p2, p3)].
        '''
        marker = copy.deepcopy(RvizMarker._MARKER_TEMPLATE)
        size, color, lifetime = RvizMarker._V_LINK.get_params()
        marker.id = id
        marker.type = marker.LINE_LIST
        marker.header.stamp = rospy.Time.now()
        marker.points = [Point(*xyz) for xyz in list_xyz]
        marker.scale.x = size
        marker.color = COLORS[color]
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        RvizMarker._pub.publish(marker)

    @staticmethod
    def draw_single_strand_links(id, list_xyz):
        ''' If list_xyz=[p0, p1, p2, p3],
        then the links are [(p0, p1), (p1, p2), (p2, p3)].
        '''
        marker = copy.deepcopy(RvizMarker._MARKER_TEMPLATE)
        size, color, lifetime = RvizMarker._V_LINK.get_params()
        marker.id = id
        marker.type = marker.LINE_STRIP
        marker.header.stamp = rospy.Time.now()
        marker.points = [Point(*xyz) for xyz in list_xyz]
        marker.scale.x = size
        marker.color = COLORS[color]
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        RvizMarker._pub.publish(marker)

    @staticmethod
    def draw_dots(id, list_xyz):
        marker = copy.deepcopy(RvizMarker._MARKER_TEMPLATE)
        size, color, lifetime = RvizMarker._V_DOT.get_params()
        marker.id = id
        marker.type = marker.SPHERE_LIST
        marker.header.stamp = rospy.Time.now()
        marker.points = [Point(*xyz) for xyz in list_xyz]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color = COLORS[color]
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        RvizMarker._pub.publish(marker)

    @staticmethod
    def delete_marker(id):
        ''' WARNING: This works in this script's test case,
        but sometimes doesn't work in my main program.
        '''
        marker = copy.deepcopy(RvizMarker._MARKER_TEMPLATE)
        marker.id = id
        marker.action = marker.DELETE
        RvizMarker._pub.publish(marker)
        # rospy.loginfo("Delete marker with id = {}".format(id))

    # @staticmethod
    # def deleteAllMarkers():
    #     RvizMarker._pub.deleteAllMarkers()

    @staticmethod
    def _check_initialization():
        if (RvizMarker._MARKER_TEMPLATE is None) or \
                (RvizMarker._pub is None):
            raise RuntimeError("Please use RvizMarker.init() "
                               "to initialize the class.")

    @staticmethod
    def _create_template_marker():
        marker = Marker()
        marker.header = Header()
        marker.action = marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0
        marker.scale.y = 0
        marker.scale.z = 0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker


if __name__ == '__main__':
    ''' Example usage of RvizMarker. '''

    rospy.init_node('rviz_a_ball_rolling_in_a_circle')

    # -- Set the moving speed of the ball.
    # The time period of the end-efforctor sweeping one circle
    T = rospy.get_param("T", 5.0)
    f = 10.0  # How many points to publish per second.
    rate = rospy.Rate(f)
    lifetime = T*0.6

    # -- Set rviz drawer.
    RvizMarker.init()
    RvizMarker.set_dot(size=0.1, color='r', lifetime=lifetime)
    RvizMarker.set_link(size=0.01, color='g', lifetime=lifetime)

    # -- Loop.
    ite = 0
    while not rospy.is_shutdown():

        # Compute the pos of next point,
        m = 2*math.pi * ite / f / T
        x = math.cos(m)
        y = math.sin(m)

        # Draw marker
        ite += 1
        if ite > 1:
            RvizMarker.draw_dot(id=ite,
                                xyz=[x, y, 0])
            RvizMarker.draw_link(id=ite+10000,
                                 xyz1=[x, y, 0],
                                 xyz2=[x_pre, y_pre, 0])
            TEST_DELETE = True
            if TEST_DELETE:  # Delete the previous dot.
                # So there is only one dot being visualized in rviz.
                RvizMarker.delete_marker(ite-1)

        # Update info.
        x_pre = x
        y_pre = y

        rate.sleep()
        if ite % 10 == 0:
            print(ite)
