cmake_minimum_required(VERSION 3.19)
project(Features)

set(CMAKE_CXX_STANDARD 14)

add_executable(Features main.cpp NoteOfDocuments/NormalsOfPointCloud.cpp)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Features ${PCL_LIBRARIES})