cmake_minimum_required(VERSION 3.0.2)
project(jose)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  srdfdom
  trac_ik_lib
  visualization_msgs
  interactive_markers
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jose
  CATKIN_DEPENDS geometry_msgs roscpp rviz srdfdom trac_ik_lib visualization_msgs interactive_markers
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(jose src/jose.cpp)
target_link_libraries(jose ${catkin_LIBRARIES})

add_executable(jose_test src/jose_test.cpp)
target_link_libraries(jose_test jose ${catkin_LIBRARIES})

add_library(jose_markers src/jose_markers.cpp)
target_link_libraries(jose_markers jose ${catkin_LIBRARIES})

add_executable(jose_markers_node src/jose_markers_node.cpp)
target_link_libraries(jose_markers_node jose_markers ${catkin_LIBRARIES})
