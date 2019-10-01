#pragma once

#include <vector>
#include "profile/motion_state.hh"
#include "profile/motion_segment.hh"

namespace pathetic::profile {
  class motion_profile {
    public:
    motion_profile(std::vector<motion_segment>& segments);
    auto operator[](double t) const -> motion_state;
    auto duration() const -> double;
    auto reversed() -> motion_profile;
    auto start() -> motion_state;
    auto end() -> motion_state;
    private:
    std::vector<motion_segment>& segments;
  };
}