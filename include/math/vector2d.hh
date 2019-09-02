#pragma once

#include <cmath>
#include "math/pose2d.hh"

namespace pathetic::math {
class vector2d {
  public:
  explicit vector2d(double x = 0.0, double y = 0.0);
  
  auto norm() const -> double { return std::sqrt(x * x + y * y); }
  auto angle() const -> double { return std::atan2(y, x); }
  auto operator+(vector2d const& other) const;
  auto operator-(vector2d const& other) const;
  auto operator*(double scalar) const;
  auto operator/(double scalar) const;
  auto operator-() const;
  auto dot(vector2d const& other) const -> double;
  auto dist_to(vector2d const& other) const -> double;
  auto project_onto(vector2d const& target) const -> vector2d;
  auto rotated(double angle) const -> vector2d;
  auto vec() const -> vector2d;
  double x, y;

}; 
}
