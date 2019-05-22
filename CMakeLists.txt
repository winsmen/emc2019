cmake_minimum_required(VERSION 2.8)
project(escRoom)

find_package(OpenCV REQUIRED)

add_executable(main src/main.cpp ${INC_ALL})
include_directories(opencv)
target_link_libraries(main emc-framework ${OpenCV_LIBS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
