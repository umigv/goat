cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(cmd_vel_selector)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp
                                        sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs)

include_directories(include/ ${catkin_INCLUDE_DIRS})

SET(SOURCES src/cmd_vel_selector_node.cpp src/node_driver.cpp)

add_executable(cmd_vel_selector_node ${SOURCES})
target_link_libraries(cmd_vel_selector_node ${catkin_LIBRARIES})
add_dependencies(cmd_vel_selector_node ${catkin_EXPORTED_TARGETS})

install(TARGETS cmd_vel_selector_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
