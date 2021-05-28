cmake_minimum_required(VERSION 3.10)
project(Recognition)

set(CMAKE_CXX_STANDARD 14)

add_executable(Recognition main.cpp NoteOfRecongnition/3DObjectRecognitonBasedOnCorrespondenceGrouping.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Recognition ${PCL_LIBRARIES})