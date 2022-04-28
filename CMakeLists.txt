cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(control_p3)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf_sim
)

# Defining the package
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
