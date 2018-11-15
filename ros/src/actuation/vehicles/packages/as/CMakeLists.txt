cmake_minimum_required(VERSION 2.8.3)
project(as)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  dbw_mkz_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  dbw_mkz_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(as_interface
  nodes/as_interface/as_interface_node.cpp
  nodes/as_interface/as_interface.cpp
  )
target_link_libraries(as_interface ${catkin_LIBRARIES})
add_dependencies(as_interface ${catkin_EXPORTED_TARGETS} )

