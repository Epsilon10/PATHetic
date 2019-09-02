#pragma once

#include <cmath>
#include "vector2d.hh"

namespace pathetic::math {
  class pose2d {
    public:
    explicit pose2d::pose2d(double x, double y, double heading = 0.0);
    explicit pose2d::pose2d(vector2d const& vec, double heading=0.0);

    auto operator+(pose2d const& other) const;
    auto operator-(pose2d const& other) const;
    auto operator*(double scalar) const;
    auto operator/(double scalar) const;
    auto operator-() const;
    auto equals(pose2d const& other) const;
    auto vec() const -> vector2d;
    double x, y, heading;
  };
}
