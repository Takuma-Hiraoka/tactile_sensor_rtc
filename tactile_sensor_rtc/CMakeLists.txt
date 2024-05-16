cmake_minimum_required(VERSION 3.0.2)
project(tactile_sensor_rtc)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  )

rtmbuild2_init()

catkin_package(
  )

add_subdirectory(rtc)
