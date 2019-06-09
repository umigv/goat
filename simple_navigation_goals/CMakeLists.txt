cmake_minimum_required(VERSION 3.14)

project(simple_navigation_goals VERSION 0.1.0)

find_package(catkin REQUIRED COMPONENTS roscpp geodesy geographic_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp geodesy geographic_msgs geometry_msgs)

add_executable(simple_navigation_goals_node src/main.cpp)
target_include_directories(simple_navigation_goals_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(simple_navigation_goals_node ${catkin_LIBRARIES})
add_dependencies(simple_navigation_goals_node ${catkin_EXPORTED_TARGETS})
target_compile_features(simple_navigation_goals_node PRIVATE cxx_std_14)

install(TARGETS simple_navigation_goals_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

