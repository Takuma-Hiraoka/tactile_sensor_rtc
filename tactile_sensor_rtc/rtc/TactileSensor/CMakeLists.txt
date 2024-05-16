add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

add_library(TactileSensor SHARED TactileSensor.cpp)
target_link_libraries(TactileSensor
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )
set_target_properties(TactileSensor PROPERTIES PREFIX "") # libTactileSensor.so -> TactileSensor.so
#add_dependencies(TactileSensor RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(TactileSensorComp TactileSensorComp.cpp)
target_link_libraries(TactileSensorComp TactileSensor)

install(TARGETS TactileSensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS TactileSensorComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
