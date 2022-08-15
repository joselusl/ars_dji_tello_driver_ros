cmake_minimum_required(VERSION 3.0.2)
project(dji_tello_driver_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  dji_tello_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs dji_tello_msgs ars_robot_msgs
    INCLUDE_DIRS
    LIBRARIES
)
