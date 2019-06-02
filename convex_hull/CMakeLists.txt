cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(concvex_hull_2d)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (convex_hull_2d convex_hull.cpp)
target_link_libraries (convex_hull_2d ${PCL_LIBRARIES})
