cmake_minimum_required(VERSION 3.0.2)
project(corner_detection)

find_package(catkin REQUIRED COMPONENTS
  laser_line_extraction
  roscpp
  rospy
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES corner_detection
 CATKIN_DEPENDS laser_line_extraction roscpp rospy
#  DEPENDS system_lib
)

add_library(corner_detection_library
  src/corner_detection.cpp
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#
add_executable(corner_detection_node src/corner_detection_node.cpp)
add_dependencies(corner_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(corner_detection_node corner_detection_library ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  scripts/corner_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
