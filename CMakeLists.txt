cmake_minimum_required (VERSION 3.9)

project (Slam_viewer)


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -O3")  # -Werror  -Wall

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin")

## -----+-----+-----+ Eigen -----+-----+-----+
find_package (Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories("${PROJECT_SOURCE_DIR}/include")

file (GLOB files "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable (slam_viewer ${files})

target_link_libraries(slam_viewer
    LINK_PUBLIC  ${EIGEN_LIBRARIES}
    )

