#include <algorithm>
#include "profile/motion_profile.hh"

namespace pathetic::profile {
  motion_profile::motion_profile(std::vector<motion_segment>& segments) 
    : segments(segments) { }
  
  auto motion_profile::operator[](double t) const -> motion_state {
    auto remaining_time = std::max(0.0, std::min(t, duration()));
    if (segments.size() == 0)
      return motion_state{0.0,0.0, 0.0, 0.0};
    for (auto const& segment : segments) {
      if (remaining_time <= segment.dt) 
        return segment[remaining_time];
      remaining_time -= segment.dt;
    }
    return (*segments.end()).end();
  }
}