cmake_minimum_required(VERSION 2.8)
project(my_project)

add_executable(move src/move.cpp)
target_link_libraries(move emc-framework)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
