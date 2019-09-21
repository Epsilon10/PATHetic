#include "kinematics/kinematics.hh"

#include <vector>
namespace pathetic::kinematics::tank {
  auto robot_to_wheel_velocities(math::pose2d const& robot_vel, double track_width)
    -> std::vector<double>;

  auto wheel_to_robot_velocity(std::vector<double> const& wheel_vels, double track_width) 
    -> math::pose2d;
};