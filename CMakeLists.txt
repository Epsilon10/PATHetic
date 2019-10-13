
cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(pathetic)



add_library(pathetic SHARED
	src/control/pid_coeffecients.cc
	src/control/pidf_controller.cc
	src/kinematics/kinematics.cc
	src/kinematics/tank_kinematics.cc
	src/math/vector2d.cc
	src/math/pose2d.cc
	src/path/quintic_spline.cc
	src/path/parametric_curve.cc
	src/path/quintic_polynomial.cc
	src/profile/motion_state.cc
	src/profile/motion_profile.cc
	src/profile/motion_segment.cc
	src/profile/profile_generator.cc
	src/trajectory/constraints/drive_constraints.cc
)

target_include_directories(pathetic
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include
)
