add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  )
find_package(choreonoid REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す

set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(TactileSensor SHARED
  TactileSensor.cpp
  tactile_shm.cpp
  )
target_link_libraries(TactileSensor
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )
set_target_properties(TactileSensor PROPERTIES PREFIX "") # libTactileSensor.so -> TactileSensor.so
add_dependencies(TactileSensor RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(TactileSensorComp TactileSensorComp.cpp)
target_link_libraries(TactileSensorComp TactileSensor)
