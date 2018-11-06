cmake_minimum_required(VERSION 2.8.12)
project(cloud_projector)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_sensor_msgs
        geometry_msgs
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        tf2_sensor_msgs
        geometry_msgs
        )

# Library
add_library(cloud_projector_lib SHARED
        src/cloud_projector.cpp
        include/cloud_projector/cloud_projector.h
        )

if (OPENMP_FOUND)
    set_target_properties(cloud_projector_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

target_include_directories(cloud_projector_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(cloud_projector_lib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_dependencies(cloud_projector_lib
        ${catkin_EXPORTED_TARGETS}
        )

#Fusion node
add_executable(cloud_projector
        src/cloud_projector_node.cpp
        )
target_include_directories(cloud_projector PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(cloud_projector
        cloud_projector_lib)

install(TARGETS cloud_projector
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
