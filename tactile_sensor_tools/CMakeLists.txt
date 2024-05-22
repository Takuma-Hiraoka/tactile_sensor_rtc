cmake_minimum_required(VERSION 3.0.2)
project(tactile_sensor_tools)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tactile_sensor_msgs
  )
find_package(choreonoid REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

find_package(PkgConfig)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})

add_executable(tactile_sensor_wrench_converter src/tactile_sensor_wrench_converter.cpp)
target_link_libraries(tactile_sensor_wrench_converter
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES})

add_executable(tactile_sensor_visualizer src/tactile_sensor_visualizer.cpp)
target_link_libraries(tactile_sensor_visualizer
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES})

add_executable(config_generator src/config_generator.cpp)
target_link_libraries(config_generator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES})
