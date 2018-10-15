cmake_minimum_required(VERSION 2.8.3)
project(autoware_core)

add_compile_options(-std=c++11)

find_package(Boost COMPONENTS thread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES autoware_core
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(autoware_core_node src/autoware_core_main.cpp src/autoware_core.cpp src/command_executer.cpp src/node_launcher.cpp)
target_link_libraries(autoware_core_node ${Boost_LIBRARIES})
#add_dependencies(autoware_core_node ${catkin_EXPORTED_TARGETS})

set(CPP_EXEC_NAMES autoware_core_node)
foreach(cpp_exec_names ${CPP_EXEC_NAMES})
  install(TARGETS ${cpp_exec_names}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(cpp_exec_names)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY include/cpp-httplib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)