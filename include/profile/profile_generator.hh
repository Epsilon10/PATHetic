#pragma once

#include "profile/motion_state.hh"

namespace pathetic::profile {
  motion_profile generate_motion_profile(
    motion_state const& start, motion_state const& goal, 
    double resolution = 0.25
  );
}