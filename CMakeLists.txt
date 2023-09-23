cmake_minimum_required(VERSION 3.15)
project(healthManager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(BUILD_EXAMPLE "Build EXAMPLE" ON)

# Fetch nlohmann_json
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

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
    add_executable(${PROJECT_NAME}_node
        src/healthManager_node.cpp
        src/ros/service_ros1.cpp
    )
    target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        pthread
        nlohmann_json::nlohmann_json
    )
endif(DEFINED ENV{ROS_DISTRO})

# ROS2
if(DEFINED RMW_IMPLEMENTATION)
    message(STATUS "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}")
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    add_executable(${PROJECT_NAME}_node
        src/healthManager_node.cpp
        src/ros/service_ros2.cpp
    )
    target_link_libraries(${PROJECT_NAME}_node
        rclcpp
        std_msgs
        pthread
        nlohmann_json::nlohmann_json
    )
endif()

if(BUILD_EXAMPLE)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/example)
endif(BUILD_EXAMPLE)