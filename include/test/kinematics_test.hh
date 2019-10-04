#pragma once
#include "math/pose2d.hh"
#include "math/vector2d.hh"
#include "kinematics/tank_kinematics.hh"
#include "math/general.hh"
#include <cmath>
#include <iostream>

class kinematics_test {
  kinematics_test() = default;
  
  static void test_tank_kinematics() {
    pathetic::math::pose2d actual_velocity{2.0,0.0, -M_PI / 4};
    auto const wheel_velocites = pathetic::kinematics::tank::robot_to_wheel_velocities(actual_velocity, 10.0);
    auto const predicted_velocity = pathetic::kinematics::tank::wheel_to_robot_velocity(wheel_velocites, 10.0);
    assert(pathetic::math::floating_point_eq(predicted_velocity, actual_velocity));
    std::cout << "Tank Kinematics Test Passed. " << std::endl;
  }

  // TODO: add mecanum
};