cmake_minimum_required(VERSION 2.8.3)
project(lidar_object_visualizer)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  autoware_msgs
  jsk_recognition_msgs
  )


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  autoware_msgs
  jsk_recognition_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)


#lidar_object_visualizer
add_executable(lidar_object_visualizer
  nodes/lidar_object_visualizer/lidar_object_visualizer_main.cpp
  nodes/lidar_object_visualizer/lidar_object_visualizer.cpp
  )
target_link_libraries(lidar_object_visualizer
  ${catkin_LIBRARIES}
  )
add_dependencies(lidar_object_visualizer
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS
        lidar_object_visualizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
