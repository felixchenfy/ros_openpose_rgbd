cmake_minimum_required(VERSION 2.8.3)
project(ros_openpose_rgbd)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)