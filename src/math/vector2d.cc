#include "math/vector2d.hh"

namespace pathetic::math {
  vector2d::vector2d(double x=0.0, double y=0.0)
    : x(x), y(y) { }

  auto vector2d::operator+(vector2d const& other) const {
    return vector2d(other.x + x, other.y + y);
  }

  auto vector2d::operator-(vector2d const& other) const {
    return vector2d(x - other.x, y - other.y);
  }

  auto vector2d::operator*(double scalar) const {
    return vector2d(x * scalar, y * scalar);
  }

  auto vector2d::operator/(double scalar) const {
    return vector2d(x / scalar, y / scalar);
  }

  auto vector2d::dot(vector2d const& other) const -> double {
    return x * other.x + y * other.y;
  }

  auto vector2d::dist_to(vector2d const& other) const -> double {
    return (*this - other).norm();
  }
  
  auto vector2d::project_onto(vector2d const& target) const -> vector2d {
    auto d1 = this->dot(target);
    auto d2 = target.dot(target);
    return target * (d1 / d2);
  }

  auto vector2d::rotated(double angle) const -> vector2d {
    auto new_x = x * std::cos(angle) - y * std::sin(angle);
    auto new_y = x * std::sin(angle) + y * std::cos(angle);
    return vector2d(new_x, new_y);
  }
}
