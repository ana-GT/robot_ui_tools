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
  generate_parameter_library
  )

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)


# Parameters
set(ROBOT_TASK_UI_PARAM_LIB robot_task_ui_params)
generate_parameter_library(${ROBOT_TASK_UI_PARAM_LIB} 
                           src/robot_task_ui_params.yaml)

add_library(${PROJECT_NAME} SHARED src/task_ui.cpp)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})

add_library(task_ui_markers SHARED src/task_ui_markers.cpp)
target_link_libraries(task_ui_markers ${PROJECT_NAME})
ament_target_dependencies(task_ui_markers ${DEPS} )

add_executable(task_ui_markers_node src/task_ui_markers_node.cpp)
target_link_libraries(task_ui_markers_node task_ui_markers)


add_executable(trac_ik_test src/trac_ik_test.cpp)
ament_target_dependencies(trac_ik_test ${DEPS})

# **********************************************************
# Markers used for Reachability
add_library(robot_task_markers SHARED src/robot_task_markers.cpp)
target_link_libraries(robot_task_markers ${ROBOT_TASK_UI_PARAM_LIB})
ament_target_dependencies(robot_task_markers ${DEPS})

add_executable(markers_get_robot_base_node src/markers_get_robot_base_node.cpp src/markers_get_robot_base.cpp)
target_link_libraries(markers_get_robot_base_node robot_task_markers)

add_executable(markers_get_reach_poses_node src/markers_get_reach_poses_node.cpp src/markers_get_reach_poses.cpp)
target_link_libraries(markers_get_reach_poses_node robot_task_markers)


install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  ${ROBOT_TASK_UI_PARAM_LIB} 
  task_ui_markers 
  task_ui_markers_node
  robot_task_markers
  markers_get_robot_base_node 
  markers_get_reach_poses_node
  trac_ik_test
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )      

ament_export_dependencies(${DEPS})

ament_export_include_directories(include)

ament_package()
