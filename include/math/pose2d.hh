#pragma once

#include <cmath>

namespace pathetic::math {
  class pose2d {
    public:
    explicit pose2d::pose2d(double x, double y, double heading = 0.0);

    auto operator+(pose2d const& other);
    auto operator-(pose2d const& other);
    auto operator*(double scalar);
    auto operator/(double scalar);
    auto operator-();

    double x, y, heading;
  };
}
