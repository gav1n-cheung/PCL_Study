cmake_minimum_required(VERSION 3.10)
project(RangeImage)

set(CMAKE_CXX_STANDARD 14)

add_executable(RangeImage main.cpp NoteOfRangeImage/CreateRangeImageFromPointCloud.cpp NoteOfRangeImage/ExtractBordersFromRangeImages.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(RangeImage ${PCL_LIBRARIES})