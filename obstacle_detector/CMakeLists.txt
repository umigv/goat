# UMIGV Obstacle Detector CMake
# Following template here http://wiki.ros.org/catkin/CMakeLists.txt

# 1. Required CMake Version
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 2. Package Name
project(obstacle_detector)

# 3. Find other CMake/Catkin packages needed for build
# Every package that becomes a component of catkin will be appended
# to the catkin_ variables. catkin_INCLUDE_DIRS will contain the include
# paths for not only catkin but also for roscpp, std_msgs, cv_bridge, etc.

find_package(catkin REQUIRED COMPONENTS costmap_2d cv_bridge goat_msgs
                                        image_geometry image_transport pluginlib
                                        roscpp sensor_msgs std_msgs tf2
                                        tf2_geometry_msgs tf2_ros umigv_utilities)

find_package(OpenCV REQUIRED)

# 7. Specify package build info export

catkin_package(INCLUDE_DIRS include LIBRARIES white_line_layer
               CATKIN_DEPENDS costmap_2d cv_bridge goat_msgs image_geometry
                              image_transport pluginlib roscpp sensor_msgs
                              std_msgs tf2 tf2_geometry_msgs tf2_ros umigv_utilities
               DEPENDS OpenCV)

# 7.3 Include paths and library paths
# 7.3.1
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# 7.4 Executable targets
# $PROJECT_NAME set by project() in step 2 above
add_executable(${PROJECT_NAME}_node src/obstacle_detector.cpp 
                                    src/lane_detector.cpp
                                    src/cv.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

add_library(white_line_layer plugins/white_line_costmap_layer.cpp)
target_link_libraries(white_line_layer ${catkin_LIBRARIES})
add_dependencies(white_line_layer ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node white_line_layer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
