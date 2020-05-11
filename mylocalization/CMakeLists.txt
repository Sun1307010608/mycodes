cmake_minimum_required(VERSION 2.8.3)
project(mylocalization)

add_compile_options(-std=c++11)
find_package(PCL REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        pcl_conversions
        ${PCL_OPENMP_PACKAGES}
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        std_msgs
        ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(mylocalization src/mylocalization.cpp)
target_link_libraries(mylocalization ${catkin_LIBRARIES})
add_dependencies(mylocalization ${catkin_EXPORTED_TARGETS})

add_executable(mylocalization_icp src/mylocalization_icp.cpp)
target_link_libraries(mylocalization_icp ${catkin_LIBRARIES})
add_dependencies(mylocalization_icp ${catkin_EXPORTED_TARGETS})

add_executable(mylocalization_gridmap src/mylocalization_gridmap.cpp)
target_link_libraries(mylocalization_gridmap ${catkin_LIBRARIES})
add_dependencies(mylocalization_gridmap ${catkin_EXPORTED_TARGETS})

add_executable(mylocalization_carto src/test/mylocalization_carto.cpp)
target_link_libraries(mylocalization_carto ${catkin_LIBRARIES})
add_dependencies(mylocalization_carto ${catkin_EXPORTED_TARGETS})
