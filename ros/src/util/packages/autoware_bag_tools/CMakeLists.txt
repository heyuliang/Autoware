cmake_minimum_required(VERSION 2.8.3)
project(autoware_bag_tools)

find_package(catkin REQUIRED COMPONENTS autoware_build_flags rospy rosbag nmea_msgs roslib pcl_ros )
find_package(Proj)

catkin_package(
  CATKIN_DEPENDS rosbag nmea_msgs roslib 
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
    scripts/change_frame_id.py
    scripts/replace_msg_time_with_hdr.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(nmea2kml nodes/nmea2kml/nmea2kml.cpp nodes/nmea2kml/ReadNMEASentence.cpp)
target_link_libraries(nmea2kml ${catkin_LIBRARIES})

add_executable(map2kml nodes/map2kml/map2kml.cpp nodes/map2kml/MapGrid.cpp)
target_link_libraries(map2kml ${catkin_LIBRARIES} ${PROJ_LIBRARY})

add_dependencies(nmea2kml map2kml ${catkin_EXPORTED_TARGETS})