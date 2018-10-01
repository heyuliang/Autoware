cmake_minimum_required(VERSION 2.8.3)
project(offline_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz)
find_package(Qt5 REQUIRED Core Widgets)
find_package(PCL REQUIRED)

catkin_package()

add_library(
	rviz_offline_display
		rviz_plugins/PCDFileDisplay.cpp
)

target_link_libraries(
	rviz_offline_display
		${QT_LIBRARIES}
		${catkin_LIBRARIES}
)

