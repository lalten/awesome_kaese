cmake_minimum_required(VERSION 2.8.3)
project(awesome_kaese)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()