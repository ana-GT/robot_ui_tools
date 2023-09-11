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

add_library(task_ui src/task_ui.cpp)
ament_target_dependencies(task_ui ${DEPS})

# Markers used for Reachability
add_library(robot_task_markers src/robot_task_markers.cpp)
target_link_libraries(robot_task_markers ${ROBOT_TASK_UI_PARAM_LIB})
ament_target_dependencies(robot_task_markers ${DEPS})

add_executable(robot_task_markers_node src/robot_task_markers_node.cpp)
target_link_libraries(robot_task_markers_node robot_task_markers)



add_library(task_ui_markers src/task_ui_markers.cpp)
target_link_libraries(task_ui_markers task_ui)
ament_target_dependencies(task_ui_markers ${DEPS} )

add_executable(task_ui_markers_node src/task_ui_markers_node.cpp)
target_link_libraries(task_ui_markers_node task_ui_markers)


add_executable(trac_ik_test src/trac_ik_test.cpp)
ament_target_dependencies(trac_ik_test ${DEPS})



install(DIRECTORY launch meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  ${ROBOT_TASK_UI_PARAM_LIB} 
  task_ui_markers 
  task_ui_markers_node
  robot_task_markers
  robot_task_markers_node 
  trac_ik_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )      

ament_export_dependencies(${DEPS})

ament_export_include_directories(include)

ament_package()
