cmake_minimum_required(VERSION 3.10)
project(chessRendering)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/headerFiles
    ${PROJECT_SOURCE_DIR}/src
    )

#add extra .cpp and .hpp files here
add_library(supplemental SHARED
    src/loadModel.cpp
    src/meshChecker.cpp
    src/rasterizer.cpp
    src/scene.cpp
    src/shader.cpp
    src/bvh.cpp
    src/camera.cpp
    src/pathtracer.cpp
)

add_executable(chessRenderer main.cpp)
target_link_libraries(chessRenderer ${OpenCV_LIBRARIES} supplemental OpenMP::OpenMP_CXX)

add_executable(checkMesh src/meshChecker.cpp)
target_link_libraries(checkMesh ${OpenCV_LIBRARIES} supplemental)

#add more executables as required for testing:
#example:
#add_executable(test1 test1.cpp ${OpenCV_LIBRARIES} supplemental)
#target_link_libraries(test1 ${OpenCV_LIBRARIES} supplemental) 