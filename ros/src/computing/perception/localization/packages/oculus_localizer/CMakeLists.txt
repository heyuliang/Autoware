cmake_minimum_required(VERSION 3.1.0)
project(oculus_localizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set (CMAKE_INCLUDE_CURRENT_DIR		ON)
set (CMAKE_AUTOMOC					ON)
set (CMAKE_AUTOUIC					ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
#	autoware_build_flags
	roscpp
	std_msgs
	tf
	cv_bridge
	rosbag_storage
	
	# Test offline GNSS localization
	nmea_msgs
	gnss
)

find_package (Boost REQUIRED COMPONENTS serialization python graph filesystem)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package (OpenMP REQUIRED)
find_package (PythonLibs REQUIRED)
find_package (G2O REQUIRED)

find_package (Qt5Widgets CONFIG REQUIRED)

add_subdirectory (DBoW2)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oculus_localizer
#  CATKIN_DEPENDS other_catkin_pkg
	LIBRARIES oculus
	CATKIN_DEPENDS libg2o
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}
	DBoW2
	
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}	
	${catkin_INCLUDE_DIRS}
	
)


set(REQUIRED_LIBS
	DBoW2
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	Boost::serialization
	${G2O_LIBRARIES}
)


add_library(
	oculus STATIC
		src/VMap.cpp
		src/MapBuilder.cpp
		src/MapBuilder2.cpp
		src/KeyFrame.cpp
		src/MapPoint.cpp
		src/Viewer.cpp
		src/ImageDatabase.cpp
		src/Frame.cpp
		src/Localizer.cpp
		src/triangulation.cpp
		src/optimizer.cpp
		src/utilities.cpp
		src/datasets/OxfordDataset.cpp	
		src/datasets/GenericDataset.cpp
		src/datasets/CustomDataset.cpp
		src/datasets/RandomAccessBag.cpp
		src/datasets/MeidaiBagDataset.cpp
		src/datasets/MeidaiLocalizerGNSS.cpp
)

add_executable(map_test
	src/bin/map_test.cpp
)

target_link_libraries(
	map_test
	oculus
	${REQUIRED_LIBS}
	${catkin_LIBRARIES}
)

add_executable(map_oxford
	src/bin/map_oxford.cpp
)

target_link_libraries(
	map_oxford
	oculus
	${REQUIRED_LIBS}
	${catkin_LIBRARIES}
)


# Dataset Browser
add_executable(dataset_browser
	src/bin/dataset_browser/main.cpp
	src/bin/dataset_browser/DatasetBrowser.cpp	
	src/bin/dataset_browser/ratio_layouted_frame.cpp
	src/bin/dataset_browser/dataset_browser.ui
# this header must be included to be processed by MOC & UIC
	src/bin/dataset_browser/ratio_layouted_frame.h
)

target_include_directories(dataset_browser
	PUBLIC src/bin/dataset_browser 
)

target_link_libraries(
	dataset_browser
		Qt5::Widgets
		oculus
		${REQUIRED_LIBS}
		${catkin_LIBRARIES}
)


add_executable(test_bag
	src/bin/test_bag.cpp
)

target_link_libraries(
	test_bag
		oculus
		${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_oculus_localizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
