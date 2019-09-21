#pragma once

#include "math/pose2d.hh"
#include "math/general.hh"
#include <vector>
namespace pathetic::kinematics {
  auto field_to_robot_velocity(math::pose2d const& field_pos, math::pose2d const& field_vel);

  auto field_to_robot_acceleration(
    math::pose2d const& field_pos, math::pose2d const& field_vel, math::pose2d const& field_acc);

  auto calculate_motor_feedforward(
    double vel, double acc, double kV, double kA, double kStatic) -> double;
  
  auto calculate_motor_feedforward(
    std::vector<double> const& vels, std::vector<double> accels,
    double kV, double kA, double kStatic
  ) -> std::vector<double>;

  auto relative_odometry_update(math::pose2d const& field_pos, 
    math::pose2d const& robot_pos_delta) -> math::pose2d;

}
