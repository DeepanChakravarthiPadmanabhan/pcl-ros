cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation1 generate_pcd_example1.cpp)
target_link_libraries (planar_segmentation1 ${PCL_LIBRARIES})
