cmake_minimum_required(VERSION 3.0.2)
project(jose_gui)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roslib
  roscpp
  rviz
  srdfdom
  trac_ik_lib
  visualization_msgs
)


add_definitions(-DQT_NO_KEYWORDS)

set(QT_FORMS ${PROJECT_SOURCE_DIR}/ui/jose_gui.ui)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_add_resources(QT_RESOURCES ${QT_RESS})
qt5_wrap_cpp(COMMON_MOC_FILES
  include/${PROJECT_NAME}/jose_gui.h
#  include/qgv/QGVCommon.h
  )
#qt5_wrap_cpp(QGV_MOC_FILES
#  include/qgv/QGVScene.h
#  )

set(SRC_FILES
  src/jose_gui.cpp
  ${COMMON_MOC_FILES}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp rviz srdfdom trac_ik_lib visualization_msgs
#  DEPENDS system_lib
)

include_directories( include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} ${QT_FORMS_HPP} ${QT_RESOURCES} ${SRC_FILES} )#${QGV_SRC_FILES} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ) #gvc )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install( FILES plugin_description.xml
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

