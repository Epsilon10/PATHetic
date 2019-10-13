#pragma once

namespace pathetic::profile {
  /**
   * Kinematic state of a motion profile at a given time
   */
  class motion_state {
    public:
    motion_state(double x, double v, double a, double j);

    auto operator[](double t) const -> motion_state;
    auto flipped() const -> motion_state;

    double x,v,a,j;
  };
}