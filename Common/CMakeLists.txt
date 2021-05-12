cmake_minimum_required(VERSION 3.10)
project(Common)

set(CMAKE_CXX_STANDARD 14)

add_executable(Common main.cpp NoteOfDocuments/NoteOfUsage.cpp NoteOfDocuments/MatrixTrans.cpp NoteOfDocuments/NoteOfCustomPointT.cpp NoteOfDocuments/NoteOfCustomPointT.h)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Common ${PCL_LIBRARIES})