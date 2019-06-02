cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(max_min)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (max_min max_min.cpp)
target_link_libraries (max_min ${PCL_LIBRARIES})
