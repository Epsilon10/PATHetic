#include "path/quintic_spline.hh"
#include <tuple>
#include "math/general.hh"

namespace pathetic::path {
  waypoint::waypoint(
    double x, double y, double dx, double dy,
    double d2x, double d2y
  ) : x(x), y(y), dx(dx), dy(dy), d2x(d2x), d2y(d2y) { }

  quintic_spline::quintic_spline(
      waypoint const& start, waypoint const& end, double max_segment_length = 0.25,
      double max_delta_k = 0.01
  ) : x(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x)
    , y(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y) { }

  auto quintic_spline::approx_length(
    math::vector2d& v1, math::vector2d const& v2, math::vector2d const& v3    
  ) const -> double {
    if (math::floating_point_eq(v1.x, v2.x)) v1.x = 0.001;
    
    auto const k_1 = 0.5 * 
      (v1.x * v1.x + v1.y * v1.y - v2.x * v2.x - v2.y * v2.y) / (v1.x - v2.x);
    auto const k_2 = (v1.y - v2.y)/(v1.x - v2.x);

    auto const b_numerator = 0.5 * (
      v2.x * v2.x - 2 * v2.x * k_1 + v2.y * v2.y - v3.x * v3.x + 2 * 
      v3.x * k_1 - v3.y * v3.y
    );

    auto const b_denom = v3.x * k_2 - v3.y + v2.y - v2.x * k_2;
    auto const b = b_numerator / b_denom;

    auto const a = k_1 - k_2 * b;
    auto const radius = std::sqrt((v1.x - a) * (v1.x - a) + (v1.y - b) * (v1.y - b));

    auto const angle = 2 * std::asin((v1.dist_to(v3) / 2.0) / radius);
    return angle * radius; 
  }

  auto quintic_spline::curvature(double t) const -> double {
    auto deriv = pnml_deriv(t);
    auto second_deriv = pnml_second_deriv(t);
    auto deriv_norm = deriv.norm();

    return std::abs(deriv.x * second_deriv.y - deriv.y * second_deriv.x)
      / std::pow(deriv_norm, 3);
    }
  
  auto quintic_spline::parametrize(
    double t_lo, double t_hi, math::vector2d const& v_lo,
    math::vector2d const& v_hi
  ) -> void {
    auto t_mid = 0.5 * (t_lo + t_hi);
    auto v_mid = (t_mid);
  }

  auto quintic_spline::pnml_get(double t) const -> math::vector2d {
    return math::vector2d{x[t], y[t]};
  }


}