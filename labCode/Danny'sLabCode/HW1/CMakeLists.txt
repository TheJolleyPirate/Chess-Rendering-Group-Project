cmake_minimum_required(VERSION 3.10)
project(homework1)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(task2 task2.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp mesh.cpp)
target_link_libraries(task2 ${OpenCV_LIBRARIES})

add_executable(task3 task3.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp mesh.cpp)
target_link_libraries(task3 ${OpenCV_LIBRARIES})