cmake_minimum_required(VERSION 3.16)
project(Surface)

set(CMAKE_CXX_STANDARD 14)

add_executable(Surface main.cpp NoteOfSurface/NoteOfSmoothingEstimation.cpp NoteOfSurface/NoteOfConstruct.cpp NoteOfSurface/NoteOfGreedyTriangulation.cpp NoteOfSurface/NoteOfBsplineFitting.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Surface ${PCL_LIBRARIES})