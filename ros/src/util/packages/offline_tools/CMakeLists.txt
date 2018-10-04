cmake_minimum_required(VERSION 2.8.3)
project(offline_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz)
find_package(Qt5 REQUIRED Core Widgets)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS})

link_directories(
	${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

add_library(
	rviz_offline_display
		rviz_plugins/PCDFileDisplay.cpp
		rviz_plugins/CSVTrajectory.cpp
)

target_link_libraries(
	rviz_offline_display
		Qt5::Core
		Qt5::Widgets
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
)

