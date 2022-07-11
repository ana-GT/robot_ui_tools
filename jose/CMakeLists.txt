cmake_minimum_required(VERSION 3.5)
project(jose)

find_package(trac_ik_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(srdfdom REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)


set(DEPS
  trac_ik_lib
  geometry_msgs
  rclcpp
  srdfdom
  visualization_msgs
  interactive_markers
  )

include_directories(include)



add_executable(trac_ik_test src/trac_ik_test.cpp)
ament_target_dependencies(trac_ik_test ${DEPS})

#target_link_libraries(trac_ik_test 
#add_library(jose src/jose.cpp)
#target_link_libraries(jose ${catkin_LIBRARIES})

#add_executable(jose_test src/jose_test.cpp)
#target_link_libraries(jose_test jose ${catkin_LIBRARIES})

#add_library(jose_markers src/jose_markers.cpp)
#target_link_libraries(jose_markers jose ${catkin_LIBRARIES})

#add_executable(jose_markers_node src/jose_markers_node.cpp)
#target_link_libraries(jose_markers_node jose_markers ${catkin_LIBRARIES})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS trac_ik_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )      

ament_export_dependencies(${DEPS})

ament_export_include_directories(include)

ament_package()
