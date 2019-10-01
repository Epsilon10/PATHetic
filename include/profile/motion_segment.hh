#pragma once

#include "profile/motion_state.hh"

namespace pathetic::profile {
  class motion_segment {
    public:
    motion_segment(motion_state const& start, double dt);
    auto operator[](double t) const -> motion_state;
    auto end() const -> motion_state;
    
    auto reversed() const -> motion_segment;
    
    motion_state start;
    double dt;
  };
}