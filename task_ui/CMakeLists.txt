cmake_minimum_required(VERSION 3.16)
project(task_ui)

find_package(ament_cmake REQUIRED)

set(DEPS
  trac_ik_lib
  geometry_msgs
  sensor_msgs
  rclcpp
  srdfdom
  visualization_msgs
  interactive_markers
  tf2
  tf2_geometry_msgs
  reachability_msgs
  )

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)



add_executable(trac_ik_test src/trac_ik_test.cpp)
ament_target_dependencies(trac_ik_test ${DEPS})

add_library(task_ui src/task_ui.cpp)
ament_target_dependencies(task_ui ${DEPS})

#add_executable(task_ui_test src/task_ui_test.cpp)
#target_link_libraries(task_ui_test task_ui ${catkin_LIBRARIES})

add_library(task_ui_markers src/task_ui_markers.cpp)
target_link_libraries(task_ui_markers task_ui)
ament_target_dependencies(task_ui_markers ${DEPS})

add_executable(task_ui_markers_node src/task_ui_markers_node.cpp)
target_link_libraries(task_ui_markers_node task_ui_markers)

add_library(robot_to_task_markers src/robot_to_task_markers.cpp)
#target_link_libraries(robot_to_task_markers )
ament_target_dependencies(robot_to_task_markers ${DEPS})

add_executable(robot_to_task_markers_node src/robot_to_task_markers_node.cpp)
target_link_libraries(robot_to_task_markers_node robot_to_task_markers)


install(DIRECTORY launch meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} 
  task_ui_markers 
  task_ui_markers_node
  task_ui_to_task_markers
  robot_to_task_markers_node 
  trac_ik_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )      

ament_export_dependencies(${DEPS})

ament_export_include_directories(include)

ament_package()
