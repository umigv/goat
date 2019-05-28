cmake_minimum_required(VERSION 3.9)

message(STATUS "${CMAKE_PREFIX_PATH}" )

project(whiteLineDetection LANGUAGES CXX CUDA)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED
	NO_MODULE
	PATHS /usr/local
	NO_DEFAULT_PATH
)

set(OpenCV_INCLUDE_DIRS
 /usr/local/include
 /usr/local/include/opencv2
)
 
set(OpenCV_LIB_DIR
  /usr/local/lib
) 

#set(OpenCV_LIBS
#	opencv_core
#	opencv_cudaarithm
#)

find_package(CUDA 9 REQUIRED)
find_package(ZED 2.7 REQUIRED)

find_package(SimtTf REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp  tf2 tf2_ros) #remove cv_bridge

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include  ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS}
                                        ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

add_executable(whiteLineDetection src/main.cpp src/WhiteLineDetection.cpp )

target_link_libraries(whiteLineDetection ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${ZED_LIBRARIES}
                                          ${OpenCV_LIBS} SimtTf::SimtTf)




 
