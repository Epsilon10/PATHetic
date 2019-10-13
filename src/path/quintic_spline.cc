#include "path/quintic_spline.hh"
#include <tuple>
#include "math/general.hh"
#include <cmath>

namespace pathetic::path {
  waypoint::waypoint(
    double x, double y, double dx, double dy,
    double d2x, double d2y
  ) : x(x), y(y), dx(dx), dy(dy), d2x(d2x), d2y(d2y) { }

  quintic_spline::quintic_spline(
      waypoint const& start, waypoint const& end, double max_segment_length,
      double max_delta_k
  ) : x_pnml(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x)
    , y_pnml(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y), max_segment_length(max_segment_length), 
      max_delta_k(max_delta_k) {
        s_samples.reserve(1000);
        t_samples.reserve(1000);
    }

  auto quintic_spline::approx_length(
    math::vector2d const& v1, math::vector2d const& v2, math::vector2d const& v3    
  ) const -> double {
    auto v1_x = v1.x;
    if (math::floating_point_eq(v1_x, v2.x)) v1_x = 0.001;
    
    auto const k_1 = 0.5 * 
      (v1_x * v1_x + v1.y * v1.y - v2.x * v2.x - v2.y * v2.y) / (v1_x - v2.x);
    auto const k_2 = (v1.y - v2.y)/(v1_x - v2.x);

    auto const b_numerator = 0.5 * (
      v2.x * v2.x - 2 * v2.x * k_1 + v2.y * v2.y - v3.x * v3.x + 2 * 
      v3.x * k_1 - v3.y * v3.y
    );

    auto const b_denom = v3.x * k_2 - v3.y + v2.y - v2.x * k_2;
    auto const b = b_numerator / b_denom;

    auto const a = k_1 - k_2 * b;
    auto const radius = std::sqrt((v1_x - a) * (v1_x - a) + (v1.y - b) * (v1.y - b));

    auto const angle = 2.0 * std::asin((v1.dist_to(v3) / 2.0) / radius);
    return angle * radius; 
  }

  auto quintic_spline::curvature(double t) const -> double {
    auto const deriv = pnml_deriv(t);
    auto const second_deriv = pnml_second_deriv(t);
    auto const deriv_norm = deriv.norm();

    return std::abs(deriv.x * second_deriv.y - deriv.y * second_deriv.x)
      / std::pow(deriv_norm, 3);
    }
  
  auto quintic_spline::parametrize(
    double t_lo, double t_hi, math::vector2d const& v_lo,
    math::vector2d const& v_hi
  ) -> void {
    auto const t_mid = 0.5 * (t_lo + t_hi);
    auto const v_mid = pnml_get(t_mid);

    auto const delta_k = std::abs(curvature(t_lo) - curvature(t_hi));
    auto const segment_length = approx_length(v_lo, v_mid, v_hi); 

    if (delta_k > max_delta_k || segment_length > max_segment_length) {
      parametrize(t_lo, t_mid, v_lo, v_mid);
      parametrize(t_mid, t_hi, v_mid, v_hi);
    } else {
      length += segment_length;
      s_samples.push_back(length);
      t_samples.push_back(t_hi);
    }
  }

  auto quintic_spline::interp(double s, double s_lo, double s_hi, double t_lo, double t_hi) const -> double {
    return t_lo + (s - s_lo) * (t_hi - t_lo) / (s_hi - s_lo);
  }

  auto quintic_spline::reparam(double s) const -> double {
    if (s <= 0.0) return 0.0;

    if (s >= length) return 1.0;

    std::size_t lo = 0;
    auto hi = s_samples.size();

    while (lo <= hi) {
      auto mid = (lo + hi) / 2;

      if (s < s_samples[mid]) {
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }

    return interp(s, s_samples[lo], s_samples[hi], t_samples[lo], t_samples[hi]);
  }

  
  auto quintic_spline::pnml_deriv(double t) const -> math::vector2d {
    return math::vector2d{x_pnml.deriv(t), y_pnml.deriv(t)};
  }

  auto quintic_spline::pnml_second_deriv(double t) const -> math::vector2d {
    return math::vector2d{x_pnml.second_deriv(t), y_pnml.second_deriv(t)};
  }

  auto quintic_spline::pnml_get(double t) const -> math::vector2d {
    return math::vector2d{x_pnml[t], y_pnml[t]};
  }

  auto quintic_spline::param_deriv(double t) const -> double {
    auto const deriv = pnml_deriv(t);
    return 1.0 / std::sqrt(deriv.x * deriv.x 
      + deriv.y * deriv.y);  
  }

  auto quintic_spline::param_second_deriv(double t) const -> double {
    auto const deriv = pnml_deriv(t);
    auto const second_deriv = pnml_second_deriv(t);
    auto const numerator = -1.0 * (deriv.x * second_deriv.x + deriv.y * second_deriv.x);
    auto const denominator = deriv.x * deriv.x + deriv.y * deriv.y;
    return numerator / (denominator * denominator);
  }

}