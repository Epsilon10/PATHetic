
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(pathetic)



add_library(pathetic SHARED
	src/control/pidf_controller.cc
	src/math/vector2d.cc
	src/math/pose2d.cc
	src/path/quintic_spline.cc
)

target_include_directories(pathetic
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)
