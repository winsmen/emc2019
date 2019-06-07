cmake_minimum_required(VERSION 2.8)
project(my_project)

add_executable(BinaryImg src/BinaryImg.cpp ${INC_ALL})
target_link_libraries(BinaryImg emc-framework)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)