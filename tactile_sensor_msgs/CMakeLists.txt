cmake_minimum_required(VERSION 2.8.3)
project(tactile_sensor_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  geometry_msgs
  )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  WrenchStampedArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs  # Or other packages containing msgs
)

catkin_package(
  )
