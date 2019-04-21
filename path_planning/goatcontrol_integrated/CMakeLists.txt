cmake_minimum_required(VERSION 3.1)
project(listener)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  sensor_msgs
  roscpp
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME}_node src/goat/listener/listener.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(goatcontrol_lib src/goat/listener/goatcontrol.h src/goat/listener/goatcontrol.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} goatcontrol_lib)
