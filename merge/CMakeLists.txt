cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(merge)

find_package(PCL 1.3 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (merge main.cpp)
target_link_libraries (merge ${PCL_LIBRARIES} ${OpenCV_LIBS})
