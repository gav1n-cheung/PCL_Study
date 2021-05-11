cmake_minimum_required(VERSION 3.10)
project(PCL_Study)

set(CMAKE_CXX_STANDARD 14)

add_executable(PCL_Study main.cpp NoteOfDocuments/NoteOfUsage.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(PCL_Study ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})