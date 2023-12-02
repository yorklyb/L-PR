cmake_minimum_required(VERSION 3.1)
project(lfm VERSION 3.1.0 LANGUAGES C CXX)

if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Set a default build type if none was specified
set(default_build_type "Release")



#yorktaglidar
find_package(PCL 1.3 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(yaml-cpp REQUIRED)

include_directories(/usr/local/include/yaml-cpp)
include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable (tag_detection ./main.cpp)
target_link_libraries (tag_detection ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

