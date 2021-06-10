cmake_minimum_required(VERSION 3.16)
project(Registration)

set(CMAKE_CXX_STANDARD 14)

add_executable(Registration main.cpp NoteOfRegistration/NoteOfPCLRegistrationAPI.cpp NoteOfRegistration/NoteOfUseIterativeClosesetPoint.cpp NoteOfRegistration/NoteOfIncremental.cpp NoteOfRegistration/NoteOfInteractiveClosestPoint.cpp NoteOfRegistration/NoteOfAlignmentPrerejective.cpp)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Registration ${PCL_LIBRARIES})