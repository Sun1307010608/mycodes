cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(getAreaList)

find_package(PCL 1.7 REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (getAreaList getAreaList.cpp)
target_link_libraries (getAreaList ${PCL_LIBRARIES})

add_executable (pcdMergeXYZ pcdMergeXYZ.cpp)
target_link_libraries (pcdMergeXYZ ${PCL_LIBRARIES})

add_executable (pcdMergeXYZI pcdMergeXYZI.cpp)
target_link_libraries (pcdMergeXYZI ${PCL_LIBRARIES})

add_executable(pcdVoxelGridFilter pcdVoxelGridFilter.cpp)
target_link_libraries(pcdVoxelGridFilter ${PCL_LIBRARIES})

