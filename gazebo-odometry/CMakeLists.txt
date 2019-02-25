cmake_minimum_required(VERSION 3.1)
project(gazebo-odometry)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  gazebo_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME}_node src/odometry_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)