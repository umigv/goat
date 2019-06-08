cmake_minimum_required(VERSION 2.8.3)

project(goat_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY robots/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots)
