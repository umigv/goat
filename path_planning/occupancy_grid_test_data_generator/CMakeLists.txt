cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(occupancy_grid)

find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp std_msgs message_generation)

add_message_files(
	FILES
)

generate_messages(
	DEPENDENCIES)

catkin_package(CATKIN_DEPENDS nav_msgs roscpp message_runtime)

add_executable(occupancy_grid occupancy_grid_constructor_node.cpp)

include_directories(include/ ${catkin_INCLUDE_DIRS})

target_link_libraries(occupancy_grid ${catkin_LIBRARIES})
add_dependencies(occupancy_grid ${catkin_EXPORTED_TARGETS})

install(TARGETS occupancy_grid
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})