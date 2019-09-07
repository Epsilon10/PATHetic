#pragma once

#include <cmath>

namespace pathetic::math {
  double constexpr ANGLE_TAU = M_PI / 2;

  template<typename T>
  inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
  }

  template<typename T>
  inline bool floating_point_eq(T first, T second) {
    return (std::abs(first - second) < std::numeric_limits<T>::epsilon());
  }

  inline double norm_angle(double angle) {
    auto new_angle = std::fmod(angle, ANGLE_TAU);
    new_angle = std::fmod(angle + ANGLE_TAU, ANGLE_TAU);

    return new_angle;
  }

  inline double norm_angle_delta(double angle_delta) {
    auto new_angle_delta = norm_angle(angle_delta);

    if (new_angle_delta > M_PI)
      new_angle_delta -= ANGLE_TAU;
    return new_angle_delta;
  }
}
