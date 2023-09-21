cmake_minimum_required(VERSION 3.15)
project(ledAlert)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/inc
)

add_library(${PROJECT_NAME} STATIC
  src/gpio.cpp
  src/ledAlert.cpp
)

add_executable(${PROJECT_NAME}_node src/ledAlert_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  pthread
)