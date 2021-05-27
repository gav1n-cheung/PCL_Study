cmake_minimum_required(VERSION 3.19)
project(Octree)

set(CMAKE_CXX_STANDARD 14)

add_executable(Octree main.cpp NoteOfOctree/PointCloudCompression.cpp NoteOfOctree/OcTree.cpp NoteOfOctree/SpatialChangeDetection.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
TARGET_LINK_LIBRARIES(Octree ${PCL_LIBRARIES})