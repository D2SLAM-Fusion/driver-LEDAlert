cmake_minimum_required(VERSION 3.15)
include(FetchContent)

project(healthManager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(BUILD_EXAMPLE "Build EXAMPLE" ON)
option(DEBUG "Debug mode" OFF)

if(DEBUG)
    message(STATUS "Debug mode")
    add_definitions(-DDEBUG=True)
    set(CMAKE_BUILD_TYPE Debug)
    include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/3rdparty/inc/debug
    )
    set(CMAKE_CXX_STANDARD_LIBRARIES "${CMAKE_CXX_STANDARD_LIBRARIES} -lbfd -ldw ")
endif(DEBUG)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}/3rdparty/inc
)

# driver
add_library(LEDAlert STATIC
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
        sensor_msgs
        nav_msgs
    )
    catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
    )
    include_directories(
        ${catkin_INCLUDE_DIRS}
    )

    # services
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/services)

    add_executable(${PROJECT_NAME}_node
        src/healthManager_node.cpp
        src/ros/service_ros1.cpp
    )
    target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        pthread

        services
    )
endif(DEFINED ENV{ROS_DISTRO})

# ROS2
if(DEFINED RMW_IMPLEMENTATION)
    add_definitions(-DROS_VERSION_ROS2=True)
    message(STATUS "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}")
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    # services
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/services)
    add_executable(${PROJECT_NAME}_node
        src/healthManager_node.cpp
        src/ros/service_ros2.cpp
    )
    target_link_libraries(${PROJECT_NAME}_node
        rclcpp
        std_msgs
        pthread

        services
    )
endif()

if(BUILD_EXAMPLE)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/example)
endif(BUILD_EXAMPLE)