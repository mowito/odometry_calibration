cmake_minimum_required(VERSION 2.8.3)
project(odometry_calibration)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
