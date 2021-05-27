cmake_minimum_required(VERSION 3.19)
project(KdTree)

set(CMAKE_CXX_STANDARD 14)

add_executable(KdTree main.cpp NoteOfKdTree/KdTree.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(KdTree ${PCL_LIBRARIES})