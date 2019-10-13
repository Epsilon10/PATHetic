#pragma once

#include <vector>
#include "profile/motion_state.hh"
#include "profile/motion_segment.hh"

namespace pathetic::profile {
  /**
   * A collection of constant acceleraiton motion segments
   */
  class motion_profile {
    public:
    motion_profile(std::vector<motion_segment>& segments);
    auto operator[](double t) const -> motion_state;
    auto duration() const -> double;
    auto reversed() -> motion_profile; // TODO
    auto start() const -> motion_state;
    auto end() const -> motion_state;
    private:
    std::vector<motion_segment>& segments;
  };
}