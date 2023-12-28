add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp)
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS})

# 相対パスを絶対パスに直す

set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_library(TactileSensorROSBridge SHARED TactileSensorROSBridge.cpp)
target_link_libraries(TactileSensorROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES})
set_target_properties(TactileSensorROSBridge PROPERTIES PREFIX "")

add_executable(TactileSensorROSBridgeComp TactileSensorROSBridgeComp.cpp)
target_link_libraries(TactileSensorROSBridgeComp TactileSensorROSBridge)

install(TARGETS TactileSensorROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS TactileSensorROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
