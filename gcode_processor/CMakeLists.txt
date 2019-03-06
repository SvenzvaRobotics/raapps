cmake_minimum_required(VERSION 2.8.3)
project(gcode_processor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)

catkin_python_setup()

catkin_package(
    DEPENDS roscpp rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
