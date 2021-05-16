cmake_minimum_required(VERSION 3.10)
project(Features)

set(CMAKE_CXX_STANDARD 14)

add_executable(Features main.cpp NoteOfDocuments/NormalsOfPointCloud.cpp NoteOfDocuments/NormalEstimationUsingIntegralImages.cpp NoteOfDocuments/PointFeatureHistogramsDescriptors.cpp NoteOfDocuments/PFH.cpp NoteOfDocuments/FPFHOfPointCloud.cpp NoteOfDocuments/VFH.cpp NoteOfDocuments/NARFOfRangeImage.cpp NoteOfDocuments/MomentOfInertiaAnd**.cpp)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(Features ${PCL_LIBRARIES})