cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(phidgets_imu)

include_directories(include/)

find_package(Boost 1.56 REQUIRED)

find_library(PHIDGET22_LIB phidget22)

set(SOURCES src/phspat.cpp src/err.cpp)

add_library(phspat_shared SHARED ${SOURCES})
set_target_properties(phspat_shared PROPERTIES PUBLIC_HEADER "phspat.h"
                                               OUTPUT_NAME phspat)
target_link_libraries(phspat_shared ${PHIDGET22_LIB})

add_library(phspat_static STATIC ${SOURCES})
set_target_properties(phspat_static PROPERTIES PUBLIC_HEADER "phspat.h"
                                               OUTPUT_NAME phspat)
target_link_libraries(phspat_static ${PHIDGET22_LIB})

option(PHIDGETS_IMU_BUILD_DEMOS "Build demos for phidgets_imu." ON)
if(PHIDGETS_IMU_BUILD_DEMOS)
    include_directories(external/Catch2/single_include)

    add_executable(phspat_demo demo/main.cpp)
    target_link_libraries(phspat_demo phspat_static)
endif()

install(TARGETS phspat_static phspat_shared PUBLIC_HEADER DESTINATION include
                                            LIBRARY DESTINATION lib
                                            ARCHIVE DESTINATION lib)
