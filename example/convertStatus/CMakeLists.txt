cmake_minimum_required(VERSION 3.15)
project(convertStatus)

set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    LEDAlert
    pthread
)

