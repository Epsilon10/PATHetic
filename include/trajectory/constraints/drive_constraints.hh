#pragma once

#include "math/pose2d.hh"

namespace pathetic::trajectory::constraints {
  struct simple_motion_constraints {
    simple_motion_constraints(double max_vel, double max_accel);
    double max_vel, max_accel;
  };

  class drive_constraints {
    public:
    drive_constraints(
      double max_vel, double max_accel, double max_jerk, 
      double max_ang_vel, double max_ang_accel, double max_ang_jerk
    );

    auto get(math::pose2d const& pos, math::pose2d const& deriv, math::pose2d const& second_deriv) 
      const -> simple_motion_constraints;

    double max_vel, max_accel, max_jerk;
    double max_ang_vel, max_ang_accel, max_ang_jerk;

  };
}