cmake_minimum_required(VERSION 3.14)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

project(phidgets_encoder VERSION 0.3.0)

find_package(Phidget22 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)

add_executable(phidgets_encoder_node src/encoder.cpp src/node.cpp)
target_include_directories(phidgets_encoder_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(phidgets_encoder_node ${catkin_LIBRARIES}
                                            Phidget22::Phidget22)
add_dependencies(phidgets_encoder_node ${catkin_EXPORTED_TARGETS})
target_compile_features(phidgets_encoder_node PRIVATE cxx_std_14)

install(TARGETS phidgets_encoder_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
