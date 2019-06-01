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

add_subdirectory(
  catch2/Catch2
)

option(BUILD_TESTS "Build tests for A*" ON)

if(BUILD_TESTS)
	enable_testing()

	add_executable(test_a_star tests/runner.cpp tests/small_test.cpp)
	target_link_libraries(test_a_star Catch2::Catch2)

	include(CTest)
	include(${CMAKE_CURRENT_SOURCE_DIR}/catch2/Catch2/contrib/Catch.cmake)
	catch_discover_tests(test_a_star)

endif()

catkin_package()

add_executable(${PROJECT_NAME}_node listener.cpp goatcontrol.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
