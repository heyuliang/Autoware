cmake_minimum_required(VERSION 2.8.3)
project(lidar_adas_ground_detect)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  pcl_ros
  autoware_msgs
  tf
  grid_map_ros
  )

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  tf
  grid_map_ros
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(lidar_adas_ground_detect
  nodes/lidar_adas_ground_detect/adas_ground_detect.cpp)

target_link_libraries(lidar_adas_ground_detect PRIVATE
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_dependencies(lidar_adas_ground_detect
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS
        lidar_adas_ground_detect
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
