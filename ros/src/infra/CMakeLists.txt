cmake_minimum_required(VERSION 2.8.3)
project(infra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autoware_msgs
  geometry_msgs
  std_msgs
  tf
)

catkin_package(CATKIN_DEPENDS
  roscpp
  autoware_msgs
  geometry_msgs
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (EXISTS /usr/include/mosquitto.h)

  add_executable(mqtt_send_bb src/mqtt_send_bb.cpp src/mqtt_bb.cpp)
  add_executable(mqtt_recv_bb src/mqtt_recv_bb.cpp src/mqtt_bb.cpp)

  target_link_libraries(mqtt_send_bb -lmosquitto ${catkin_LIBRARIES})
  target_link_libraries(mqtt_recv_bb -lmosquitto ${catkin_LIBRARIES})

  add_dependencies(mqtt_send_bb ${catkin_EXPORTED_TARGETS})
  add_dependencies(mqtt_recv_bb ${catkin_EXPORTED_TARGETS})

else ()
  message("'libmosquitto-dev' is not installed. 'mqtt_send_bb' and 'mqtt_recv_bb' will not be built.")
endif ()
