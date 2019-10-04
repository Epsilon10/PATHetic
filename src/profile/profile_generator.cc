#include "profile/profile_generator.hh"
#include <cmath> 
#include <vector>
#include <algorithm>

#include 
namespace pathetic::profile {
  motion_profile generate_motion_profile(
    motion_state const& start, motion_state const& goal, 
    double resolution = 0.25
  ) {
    auto const length = goal.x - start.x;
    auto const samples = static_cast<std::uint32_t>(std::ceil(length / resolution));
    auto const dx = length / static_cast<double>(samples);

    std::vector<double> s(samples);
    std::for_each(s.begin(), s.end(), )
  }
}