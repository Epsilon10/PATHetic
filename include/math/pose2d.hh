#pragma once

#include <cmath>
#include "math/vector2d.hh"

namespace pathetic::math {
  class pose2d {
    public:
    pose2d(double x, double y, double heading);
    pose2d(vector2d const& vec, double heading);

    auto operator+(pose2d const& other) const -> pose2d;
    auto operator-(pose2d const& other) const -> pose2d;
    auto operator*(double scalar) const -> pose2d;
    auto operator/(double scalar) const -> pose2d;
    auto operator-() const -> pose2d;
    auto equals(pose2d const& other) const -> bool;
    auto vec() const -> vector2d;
    double x, y, heading;
  };
}
