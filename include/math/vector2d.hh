#pragma once

#include <cmath>

namespace pathetic::math {
class vector2d {
  public:
  vector2d(double x, double y);
  
  auto norm() const -> double { return std::sqrt(x * x + y * y); }
  auto angle() const -> double { return std::atan2(y, x); }
  auto operator+(vector2d const& other) const -> vector2d;
  auto operator-(vector2d const& other) const -> vector2d;
  auto operator*(double scalar) const-> vector2d;
  auto operator/(double scalar) const-> vector2d;
  auto operator-() const-> vector2d;
  auto dot(vector2d const& other) const -> double;
  auto dist_to(vector2d const& other) const -> double;
  auto project_onto(vector2d const& target) const -> vector2d;
  auto rotated(double angle) const -> vector2d;
  auto vec() const -> vector2d;
  double x, y;

}; 
}
 