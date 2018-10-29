cmake_minimum_required(VERSION 2.8.3)

project(umigv_utilities)

find_package(catkin REQUIRED COMPONENTS phidgets_api roscpp)
find_package(Boost REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp INCLUDE_DIRS include
               DEPENDS Boost)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
