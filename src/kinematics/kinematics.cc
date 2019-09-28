#include "kinematics/kinematics.hh"

namespace pathetic::kinematics {
  auto field_to_robot_velocity(math::pose2d const& field_pos, math::pose2d const& field_vel) {
    return math::pose2d(field_vel.vec().rotated(-field_pos.heading), field_vel.heading);
  }

  auto field_to_robot_acceleration(
    math::pose2d const& field_pos, math::pose2d const& field_vel, math::pose2d const& field_acc
  ) {
    return math::pose2d(field_acc.vec().rotated(-field_pos.heading), field_vel.heading) + 
      math::pose2d(
        -field_vel.x * std::sin(field_pos.heading) + field_vel.y * std::cos(field_pos.heading),
        -field_vel.x * std::cos(field_pos.heading) - field_vel.y * std::sin(field_pos.heading),
        0.0
      ) * field_vel.heading;
  }

  auto calculate_motor_feedforward(
    double vel, double acc, double kV, double kA, double kStatic) -> double {
    auto base_power = vel * kV + acc * kA;
    return math::floating_point_eq(base_power, 0.0) ? 0.0 : base_power + 
          math::sign(base_power) * kStatic;
  }

  auto calculate_motor_feedforward(
    std::vector<double> const& vels, std::vector<double> accels,
    double kV, double kA, double kStatic
  ) -> std::vector<double> {
    std::vector<double> ffs(vels.size());
    for (auto const& v : vels) {
      for (auto const& a : accels) {
        ffs.push_back(calculate_motor_feedforward(
          v, a, kV, kA, kStatic
        ));
      }
    }

    return ffs;
  }

  auto relative_odometry_update(math::pose2d const& field_pos, 
    math::pose2d const& robot_pos_delta) -> math::pose2d {
      auto dtheta = robot_pos_delta.heading;
      
      if (math::floating_point_eq(robot_pos_delta.heading, 0.0)) {
        auto final_heading = field_pos.heading + robot_pos_delta.heading;
        auto cos_term = std::cos(final_heading) - std::cos(field_pos.heading);
        auto sin_term = std::sin(final_heading) - std::sin(field_pos.heading);

        auto field_pos_delta = math::pose2d(
          (robot_pos_delta.x * sin_term + robot_pos_delta.y * cos_term) / robot_pos_delta.heading,
          (-robot_pos_delta.x * cos_term + robot_pos_delta.y * sin_term) / robot_pos_delta.heading,
          robot_pos_delta.heading 
        );

        return field_pos_delta + field_pos;
      } else {
        auto field_pos_delta = math::pose2d(
          robot_pos_delta.vec().rotated(field_pos.heading + robot_pos_delta.heading / 2.0),
          robot_pos_delta.heading
        );

        return field_pos + field_pos_delta;
      }
  }

}
