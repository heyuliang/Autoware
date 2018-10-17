cmake_minimum_required(VERSION 2.8.3)
project(vehicle_dynamics_lib)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  roscpp
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle_dynamics
  CATKIN_DEPENDS autoware_msgs geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vehicle_dynamics 
  src/vehicle_model/unicycle_model.cpp
  src/brake_model/simple_delay_model.cpp
  src/accel_model/simple_delay_model.cpp)
target_link_libraries(vehicle_dynamics ${catkin_LIBRARIES})

# Install library
install(TARGETS vehicle_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/vehicle_dynamics_lib/vehicle_model include/vehicle_dynamics_lib/accel_model include/vehicle_dynamics_lib/brake_model
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)