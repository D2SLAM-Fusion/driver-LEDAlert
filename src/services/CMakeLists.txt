cmake_minimum_required(VERSION 3.15)
project(services)

set(CMAKE_CXX_STANDARD 17)

if(DEFINED ENV{ROS_DISTRO} OR DEFINED RMW_IMPLEMENTATION)
    set(SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/vinsChecker/vinsChecker.cpp
    )
endif(DEFINED ENV{ROS_DISTRO} OR DEFINED RMW_IMPLEMENTATION)


add_library(${PROJECT_NAME} STATIC
    ${CMAKE_CURRENT_LIST_DIR}/services.cpp
    ${SOURCES}
)