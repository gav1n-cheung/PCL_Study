cmake_minimum_required(VERSION 3.19)
project(IO)

set(CMAKE_CXX_STANDARD 14)

add_executable(IO main.cpp NoteOfIO/PCDFileFormat.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(IO ${PCL_LIBRARIES})