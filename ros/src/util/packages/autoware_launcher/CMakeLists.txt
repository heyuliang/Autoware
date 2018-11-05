cmake_minimum_required(VERSION 2.8.3)
project(autoware_launcher)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

set(CMAKE_AUTOMOC ON)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS rviz
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_main
    src/main.cpp
    src/main_window.cpp
    src/main_config.cpp
    src/main_tool.cpp
    src/browse_launcher.cpp
    src/launch_button.cpp
    src/map_config_widget.cpp
    src/map_config_window.cpp
    src/frames/base_window.cpp
    src/frames/base_widget.cpp
    src/frames/single_file_selector.cpp
    src/frames/multi_file_selector.cpp
    src/widgets/browse_button.cpp
)
set_target_properties(${PROJECT_NAME}_main PROPERTIES OUTPUT_NAME main PREFIX "")
target_link_libraries(${PROJECT_NAME}_main ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_library(${PROJECT_NAME}_rviz
    src/rviz_panel.cpp
    src/launch_button.cpp
    src/widgets/browse_button.cpp
)
target_link_libraries(${PROJECT_NAME}_rviz ${catkin_LIBRARIES} ${QT_LIBRARIES})

# Add header files as custom target to be listed by IDE
FILE(GLOB_RECURSE HEADERS_FOR_IDE "./*.hpp")
add_custom_target(headers SOURCES ${HEADERS_FOR_IDE})

# ToDo: install



