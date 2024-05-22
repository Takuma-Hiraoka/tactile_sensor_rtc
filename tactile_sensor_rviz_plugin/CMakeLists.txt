cmake_minimum_required(VERSION 2.8.3)
project(tactile_sensor_rviz_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rviz
  tactile_sensor_msgs
  )

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES tactile_sensor_rviz_plugin
  CATKIN_DEPENDS #
  DEPENDS rviz
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)


set(CMAKE_AUTOMOC ON)

add_library(tactile_sensor_rviz_plugin src/wrench_array_display.cpp)

if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  unset(OGRE_OV_LIBRARIES_ABS_TMP)
  foreach(_lib ${OGRE_OV_LIBRARIES})
    set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")

    find_library(${OGRE_OV_LIB_TAG}
      NAMES ${_lib}
      HINTS ${OGRE_OV_LIBRARY_DIRS}
      PATHS ${OGRE_OV_LIBRARY_DIRS}
      )

    set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
  endforeach(_lib)

  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
    CACHE FILEPATH "Pathname of library ${_lib}")
endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)



if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui QtOpenGL)
  include(${QT_USE_FILE})
  target_link_libraries(tactile_sensor_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets OpenGL Widgets)
  target_link_libraries(tactile_sensor_rviz_plugin Qt5::Widgets ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
endif()
add_definitions(-DQT_NO_KEYWORDS -g)

#############
## Install ##
#############

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(TARGETS tactile_sensor_rviz_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
