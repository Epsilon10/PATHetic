#include "trajectory/constraints/drive_constraints.hh"
#include "math/general.hh"
#include <limits>
#include <cmath>
#include <algorithm>

namespace pathetic::trajectory::constraints {
  simple_motion_constraints::simple_motion_constraints(double max_vel, double max_accel) 
    : max_vel(max_vel), max_accel(max_accel) { }
  
  drive_constraints::drive_constraints(
    double max_vel, double max_accel, double max_jerk, 
    double max_ang_vel, double max_ang_accel, double max_ang_jerk
  ) : max_vel(max_vel), max_accel(max_accel), max_jerk(max_jerk), 
      max_ang_vel(max_ang_vel), max_ang_accel(max_ang_accel), max_ang_jerk(max_ang_jerk) { }
  
  auto drive_constraints::get(math::pose2d const& pos, math::pose2d const& deriv, math::pose2d const& second_deriv) 
      const -> simple_motion_constraints {
    auto adj_max_vel = std::numeric_limits<double>::max();
    if (!(math::floating_point_eq(deriv.heading, 0.0)))
      adj_max_vel = max_ang_vel / std::abs(deriv.heading);
    
    return simple_motion_constraints{std::min(max_vel, adj_max_vel), max_accel};
  }
}