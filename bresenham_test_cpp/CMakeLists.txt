cmake_minimum_required(VERSION 2.8)
project(bresenham_test)

find_package(OpenCV REQUIRED)


include_directories(include)

add_executable(bresenham_test src/bresenham_test.cpp)
target_link_libraries(bresenham_test ${OpenCV_LIBRARIES})
