cmake_minimum_required(VERSION 3.14)

project(phidgets_encoder VERSION 0.3.0)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)

add_executable(phidgets_encoder_node src/encoder.cpp)

include_directories(include/ ${catkin_INCLUDE_DIRS})

add_executable(phidgets_encoder_node src/phidgets_encoder_node.cpp
               src/encoder_state_publisher.cpp)
target_link_libraries(phidgets_encoder_node ${catkin_LIBRARIES})
add_dependencies(phidgets_encoder_node ${catkin_EXPORTED_TARGETS})

install(TARGETS phidgets_encoder_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
