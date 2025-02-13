cmake_minimum_required(VERSION 3.16)
project(robot_sim_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

set(DEPS 
  ament_cmake
  rclcpp
  trac_ik_lib
  urdf
  srdfdom
  tf2_eigen
  tf2_eigen_kdl
  tf2_kdl
  robot_sim_msgs
  tf2_geometry_msgs
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)


include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(simulate_robot_base_motion src/simulate_robot_base_motion.cpp)
ament_target_dependencies(simulate_robot_base_motion ${DEPS})

add_executable(simple_placement_solver src/simple_placement_solver.cpp)
ament_target_dependencies(simple_placement_solver ${DEPS})


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  simulate_robot_base_motion
  simple_placement_solver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch #config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(
#    ${PROJECT_NAME}
)

#ament_export_targets(
#  reachability_generation_node
#)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()
