cmake_minimum_required (VERSION 3.9)

project (Slam_viewer)


set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -O3 -static-libstdc++ -static-libgcc")  # -Werror  -Wall

include_directories("${PROJECT_SOURCE_DIR}/include")

file (GLOB files "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable (slam_viewer ${files})

target_link_libraries(slam_viewer)

