#!/usr/bin/env python
'''
A wrapper class for drawing dots or links on Rviz.

Unit test:
    python utils/lib_ros_marker.py
    roslaunch ros_openpose_rgbd run_rviz.launch 
'''

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import rospy
import copy
import math
from geometry_msgs.msg import Point


class MarkerDrawer(object):
    def __init__(self, frame_id="map", topic_name="visualization_marker"):

        self._pub = rospy.Publisher(topic_name, Marker, queue_size=10)
        self._MARKER_TEMPLATE = self._create_template_marker()
        self._MARKER_TEMPLATE.header.frame_id = frame_id

    def draw_dot(self, id, xyz, size=0.1, lifetime=-1):
        marker = copy.deepcopy(self._MARKER_TEMPLATE)
        marker.id = id
        marker.type = marker.SPHERE
        marker.header.stamp = rospy.Time.now()
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        self._pub.publish(marker)

    def draw_link(self, id, xyz1, xyz2, size=0.1, lifetime=-1):
        marker = copy.deepcopy(self._MARKER_TEMPLATE)
        marker.id = id
        marker.type = marker.LINE_LIST
        marker.header.stamp = rospy.Time.now()
        marker.points = [Point(*xyz1), Point(*xyz2)]
        marker.scale.x = size
        if lifetime > 0:
            marker.lifetime = rospy.Duration(lifetime)
        self._pub.publish(marker)

    def _create_template_marker(self):
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
    ''' An example of draw marker on rviz. '''

    rospy.init_node('link_marker')
    drawer = MarkerDrawer()

    # The time period of the end-efforctor sweeping one circle
    T = rospy.get_param("T", 5.0)
    f = 10.0  # How many points to publish per second.
    rate = rospy.Rate(f)
    lifetime = T*0.6

    ite = 0
    while not rospy.is_shutdown():

        # Compute the pos of next point,
        m = 2*math.pi * ite / f / T
        x = math.cos(m)
        y = math.sin(m)

        # Draw marker
        ite += 1
        if ite > 1:
            drawer.draw_dot(id=ite, 
                            xyz=[x, y, 0], 
                            size=0.1,
                            lifetime=lifetime)
            drawer.draw_link(id=ite+10000,
                             xyz1=[x, y, 0],
                             xyz2=[x_pre, y_pre, 0],
                             size=0.01,
                            lifetime=lifetime)

        # Update info.
        x_pre = x
        y_pre = y

        rate.sleep()
        if ite % 10 == 0:
            print(ite)
