#pragma once
#include "path/parametric_curve.hh"
#include "path/quintic_polynomial.hh"
#include <vector>

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
    public:
    quintic_spline(
      waypoint const& start, waypoint const& end, double max_segment_length = 0.25,
      double max_delta_k = 0.01
    );

    auto pnml_get(double t) const -> math::vector2d;
    private:
    std::vector<double> s_samples;
    std::vector<double> t_samples;

    quintic_polynomial x,y;

    auto approx_length(
      math::vector2d const& v1, math::vector2d const&, math::vector2d const& v3 
    ) const -> double;

    auto curvature(double t) const -> double;
    auto parametrize(
      double t_lo, double t_hi, math::vector2d const& v_lo,
      math::vector2d const& v_hi
    ) -> void;

    auto reparam(double s) -> double;

    auto interp(double s, double s_lo, double s_hi, double t_lo, double t_hi) -> double;

    private:
    double max_delta_k, max_segment_length;
    double length = 0.0;
  };
}
