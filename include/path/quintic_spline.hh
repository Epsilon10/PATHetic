#pragma once
#include "path/parametric_curve.hh"
#include "path/quintic_polynomial.hh"
#include <array>

namespace pathetic::path {
  struct waypoint {
    waypoint(
      double x, double y, double dx, double dy,
      double d2x, double d2y
    );

    auto pos() const -> math::vector2d;
    auto deriv() const -> math::vector2d;
    auto second_deriv() const -> math::vector2d;

    double x, y, dx, dy, d2x, d2y;

  };
  class quintic_spline : public parametric_curve {
    quintic_spline(
      waypoint const& start, waypoint const& end, double max_segment_length = 0.25,
      double max_delta_k = 0.01
      );

    auto pnml_get(double t) const -> math::vector2d;
    private:
    std::array<double, 1000> s_samples;
    std::array<double, 1000> t_samples;

    quintic_polynomial x{start.x, end.x, }

    auto approx_length(
      math::vector2d& v1, math::vector2d const&, math::vector2d const& v3 
    ) const -> double;

    auto curvature(double t) const -> double;
    auto parametrize(
      double t_lo, double t_hi, math::vector2d const& v_lo,
      math::vector2d const& v_hi
    ) -> void;


  };
}
