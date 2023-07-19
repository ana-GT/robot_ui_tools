cmake_minimum_required(VERSION 3.5)
project(robots_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config robots
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
