cmake_minimum_required(VERSION 3.0.2)
project(tactile_sensor_tools)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

# 相対パスを絶対パスに直す

set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_executable(tactile_sensor_wrench_converter src/tactile_sensor_wrench_converter.cpp)

target_link_libraries(tactile_sensor_wrench_converter
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES})
