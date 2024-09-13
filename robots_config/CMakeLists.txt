cmake_minimum_required(VERSION 3.16)
project(robots_config)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robots_config
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

