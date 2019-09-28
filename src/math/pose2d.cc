#include "math/general.hh"
#include "math/pose2d.hh"
#include "math/vector2d.hh"
#include <cmath>

namespace pathetic::math {
  pose2d::pose2d(double x, double y, double heading)
    : x(x), y(y), heading(heading) { }
  
  pose2d::pose2d(vector2d const& vec, double heading) 
    : x(vec.x), y(vec.y), heading(heading) { }

  auto pose2d::operator+(pose2d const& other) const -> pose2d {
    return pose2d(x + other.x, y + other.y, heading + other.heading);
  }
  
  auto pose2d::operator-(pose2d const& other) const -> pose2d {
    return pose2d(x - other.x, y - other.y, heading - other.heading);
  } 

  auto pose2d::operator*(double scalar) const -> pose2d {
    return pose2d(x * scalar, y * scalar, heading * scalar);
  }

  auto pose2d::operator/(double scalar) const -> pose2d {
    return pose2d(x / scalar, y / scalar, heading / scalar);
  }

  auto pose2d::operator-() const -> pose2d {
    return pose2d(-x, -y, -heading);
  }

  auto pose2d::equals(pose2d const& other) const -> bool {
    return floating_point_eq(x, other.x) && floating_point_eq(y, other.y) &&
        floating_point_eq(y, other.y);
  }

  auto pose2d::vec() const -> vector2d {
    return vector2d(x,y);
  }
}
