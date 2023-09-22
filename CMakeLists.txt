cmake_minimum_required(VERSION 3.15)
project(ledAlert)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(BUILD_EXAMPLE "Build EXAMPLE" ON)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

add_library(${PROJECT_NAME} STATIC
    src/driver/gpio.cpp
    src/driver/ledAlert.cpp
)

# ROS1
if(DEFINED ENV{ROS_DISTRO})
    add_definitions(-DROS_VERSION_ROS1=True)
    message(STATUS "ROS_DISTRO: $ENV{ROS_DISTRO}")
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
    )
    catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
    )
    include_directories(
        ${catkin_INCLUDE_DIRS}
    )
    add_executable(${PROJECT_NAME}_node src/ledAlert_node.cpp)
    target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        pthread
    )
endif(DEFINED ENV{ROS_DISTRO})

# ROS1
if(DEFINED RMW_IMPLEMENTATION)
    message(STATUS "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}")
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    add_executable(${PROJECT_NAME}_node src/ledAlert_node.cpp)
    target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        rclcpp
        std_msgs
        pthread
    )
else()
    message(STATUS "Not running in ROS 2 environment")
endif()

if(BUILD_EXAMPLE)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/example)
endif(BUILD_EXAMPLE)