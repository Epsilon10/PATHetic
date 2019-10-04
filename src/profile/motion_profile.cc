#include <algorithm>
#include <numeric>
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

  auto motion_profile::duration() const -> double {
    return std::accumulate(segments.begin(), segments.end(), 0.0,
      [](auto const& a, auto const& b) { return a.dt + b.dt; });
  }

  auto motion_profile::start() const -> motion_state { return (*this)[0.0]; }
  auto motion_profile::end() const -> motion_state { return (*this)[duration()]; }
}