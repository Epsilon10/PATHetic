#pragma once

#include "control/pid_coeffecients.hh"
#include "control/pidf_controller.hh"
#include "math/pose2d.hh"

namespace pathetic::follower {
  class tank_pidva_follower {
    public:
    tank_pidva_follower(
      control::pid_coeffecients axial_coeffecients, control::pid_coeffecients lateral_coeffecients,
      math::pose2d const& admissible_error, double timeout = 0.0
    );

    private:
    control::pidf_controller axial_controller, lateral_controller;
    math::pose2d last_error;

    
  };
}