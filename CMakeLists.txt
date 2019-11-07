
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(pathetic)

add_library(pathetic SHARED
	src/control/pidf_controller.cc
	src/control/pid_coeffecients.cc
	src/kinematics/tank_kinematics.cc
	src/kinematics/kinematics.cc
	src/math/vector2d.cc
	src/math/pose2d.cc
	src/spline/quintic_hermite_spline.cc
)

add_library(pathetic::library ALIAS pathetic_library)

target_include_directories(pathetic
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)


add_executable(spline_test 
	src/test/spline_test.cc
)

target_link_libraries( spline_test
    PRIVATE 
        pathetic::library
)