cmake_minimum_required(VERSION 3.10)
project(Filtering)

set(CMAKE_CXX_STANDARD 14)

add_executable(Filtering main.cpp NoteOfFiltering/PassThroughFilter.cpp NoteOfFiltering/VoxelGridFilter.cpp NoteOfFiltering/StatisticsOutlierRemovalFilter.cpp NoteOfFiltering/ParametricModel.cpp NoteOfFiltering/ExtractingIndices.cpp NoteOfFiltering/RadiusOutlier.cpp)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Filtering ${PCL_LIBRARIES})