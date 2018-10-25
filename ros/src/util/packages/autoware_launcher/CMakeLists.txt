cmake_minimum_required(VERSION 2.8.3)
project(autoware_launcher)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(Qt5 5.3 REQUIRED Core Widgets)

set(CMAKE_AUTOMOC ON)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autoware_launcher
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
#  include
#  src/common
#  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_main
    src/main.cpp
    src/main_window.cpp
    src/main_menu.cpp
    src/browse_launcher.cpp
    src/browse_button.cpp
    src/launch_button.cpp
    src/util/config.hpp
)
set_target_properties(${PROJECT_NAME}_main PROPERTIES OUTPUT_NAME main PREFIX "")
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${QT_LIBRARIES})

# Add header files as custom target to be listed by IDE
FILE(GLOB_RECURSE HEADERS_FOR_IDE "./*.hpp")
add_custom_target(headers SOURCES ${HEADERS_FOR_IDE})

# ToDo: install



