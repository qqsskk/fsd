cmake_minimum_required(VERSION 2.8.3)
project(planning)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  fsd_common_msgs
  fs_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  fsd_common_msgs
  geometry_msgs
  fs_msgs
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  include/objects
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/planning_handle.cpp
  src/planning.cpp
  src/main.cpp
  src/TrackBoundaryEstimation/TrackBoundaryEstimator.cpp
  src/TrackBoundaryEstimation/objects/Cone.cpp
  src/TrackBoundaryEstimation/objects/Line.cpp
  src/TrackBoundaryEstimation/objects/Point.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )