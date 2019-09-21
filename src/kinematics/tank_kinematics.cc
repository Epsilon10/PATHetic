#include "kinematics/tank_kinematics.hh"

namespace pathetic::kinematics::tank {
  auto robot_to_wheel_velocities(math::pose2d const& robot_vel, double track_width)
    -> std::vector<double> {
    return { 
        robot_vel.x - track_width / 2.0 * robot_vel.heading,
        robot_vel.y + track_width / 2.0 * robot_vel.heading 
    };
  }

  auto wheel_to_robot_velocity(std::vector<double> const& wheel_vels, double track_width) 
    -> math::pose2d {
      auto left = wheel_vels[0];
      auto right = wheel_vels[1];

      return math::pose2d(
        (left + right) / 2.0,
        0.0,
        (-left + right) / track_width
      );
  }
}