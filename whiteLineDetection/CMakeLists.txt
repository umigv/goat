cmake_minimum_required(VERSION 3.5)

project(whiteLineDetection)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CUDA 9 REQUIRED)
find_package(ZED 2.7 REQUIRED)
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge tf2 tf2_ros)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS}
                                        ${OpenCV_INCLUDE_DIRS})

add_executable(whiteLineDetection src/main.cpp src/WhiteLineDetection.cpp)
target_link_libraries(whiteLineDetection ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${ZED_LIBRARIES}
                                          ${OpenCV_LIBRARIES})




 
