#!/usr/bin/env python
'''
An example code of creating and publishing a marker.
'''

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import rospy


def get_marker(marker_type):
    '''
    Usage:
        MARKER_LINE = get_marker("line")
        MARKER_POINT = get_marker("point")
    '''
    if marker_type == "point":
        USE_POINT = True
    elif marker_type == "line":
        USE_POINT = False
    else:
        assert(False)
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    if USE_POINT:
        marker.type = marker.SPHERE
    else:
        marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1 if USE_POINT else 0
    marker.scale.z = 0.1 if USE_POINT else 0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker


if __name__ == '__main__':
    ''' An example of draw marker on rviz. '''

    import copy
    import math
    from geometry_msgs.msg import Point

    rospy.init_node('link_marker')

    mark_publisher = rospy.Publisher(
        '/visualization_marker', Marker, queue_size=10)
    
    marker_type = "line"
    marker = get_marker(marker_type)

    # The time period of the end-efforctor sweeping one circle
    T = rospy.get_param("T", 5.0)
    f = 10.0  # How many points to publish per second.
    rate = rospy.Rate(f)
    marker.lifetime = rospy.Duration(T*0.6)

    ite = 0
    while not rospy.is_shutdown():

        m = 2*math.pi * ite / f / T
        x = math.cos(m)
        y = math.sin(m)

        # publish marker
        ite += 1
        if ite > 1:
            if marker_type == "point":
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0
            else:
                marker.points = [Point(x, y, 0), Point(x_pre, y_pre, 0)]
            marker.id = ite
            marker.header.stamp = rospy.Time.now()
            mark_publisher.publish(marker)
        x_pre = x
        y_pre = y

        # sleep
        # rospy.loginfo((x,y))
        rate.sleep()
        if ite % 10 == 0:
            print(ite)
