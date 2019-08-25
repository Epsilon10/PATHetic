#pragma once

#include <cmath>

namespace pathetic::math {
  
  template<typename T>
  inline constexpr int sign(T val) {
    return (T(0) < val) - (val < T(0));
  }

  template<typename T>
  inlint constexpr bool floating_point_eq(T first, T second) {
    return (std::abs(first - second) < std::numeric_limits<T>::epsilon());
  }

}
