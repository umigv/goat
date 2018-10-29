cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(transform_imu)

find_package(catkin REQUIRED
	         COMPONENTS roscpp sensor_msgs tf2_geometry_msgs
			 			tf2_ros tf2 umigv_utilities)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs tf2_geometry_msgs
						      tf2_ros tf2 umigv_utilities)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(transform_imu_node src/imu_transformer.cpp
								  src/transform_imu_node.cpp)
target_link_libraries(transform_imu_node ${catkin_LIBRARIES})
add_dependencies(transform_imu_node ${catkin_EXPORTED_TARGETS})

install(TARGETS transform_imu_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
