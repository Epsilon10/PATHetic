#include "profile/profile_generator.hh"
#include <cmath> 
#include <vector>
#include <algorithm>
#include <cstdint>

namespace pathetic::profile {
  auto generate_motion_profile(
    motion_state const& start, motion_state const& goal, 
    double resolution = 0.25
  ) -> motion_profile {
    
    auto const length = goal.x - start.x;

    auto const samples = static_cast<std::size_t>(std::ceil(length / resolution));
    auto const dx = length / samples;

    std::vector<double> s(samples);
    for(std::size_t i = 0; i < samples; i++)
      s.push_back(i * dx);

    
  }
}