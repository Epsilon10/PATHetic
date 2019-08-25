#include ".../include/math/pose2d.hh"
#include ".../include/math/general.hh"

#include <cmath>

namespace pathetic::math {
  pose2d::pose2d(double x, double y, double heading=0.0)
    : x(x), y(y), heading(heading) { }

  auto operator+(pose2d const& other) {
    return pose2d(x + other.x, y + other.y, heading + other.heading);
  }
  
  auto operator-(pose2d const& other) {
    return pose2d(x - other.x, y - other.y, heading - other.heading);
  } 

  auto operator*(double scalar) {
    return pose2d(x * scalar, y * scalar, heading * scalar);
  }

  auto operator/(double scalar) {
    return pose2d(x / scalar, y / scalar, heading / scalar);
  }

  auto operator-() {
    return pose2d(-x, -y, -heading);
  }

  auto equals(pose2d const& other) {
    return floating_point_eq(x, other.x) && floating_point_eq(y, other.y)
        floating_point_eq(y, other.y);
  }

}
