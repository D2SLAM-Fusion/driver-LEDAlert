cmake_minimum_required(VERSION 3.15)
project(example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/example")

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/convertStatus)