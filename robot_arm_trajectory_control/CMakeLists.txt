cmake_minimum_required(VERSION 2.8.3)
project(robot_arm_trajectory_control)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2_ros rospy
        DEPENDS EIGEN3
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
