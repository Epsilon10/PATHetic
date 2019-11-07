#pragma once

#include <array>

#include <Eigen/Core>

#include "spline/spline.hh"
#include "spline/spline_parametrizer.hh"

namespace pathetic {
  class quintic_hermite_spline : spline<5> {
    public:
    quintic_hermite_spline(
      waypoint const& start, waypoint const& end, double max_delta_k = .01, 
      double max_segment_length = 0.25
    );

    auto get_coeffecients() const -> Eigen::Matrix<double,6,6>;

    private:
    Eigen::Matrix<double, 6, 6> coefficients;

    static Eigen::Matrix<double, 6, 6> make_hermite_basis() {
      static const auto basis = (Eigen::Matrix<double, 6, 6>() <<
        -06.0, -03.0, -00.5, +06.0, -03.0, +00.5,
        +15.0, +08.0, +01.5, -15.0, +07.0, +01.0,
        -10.0, -06.0, -01.5, +10.0, -04.0, +00.5,
        +00.0, +00.0, +00.5, +00.0, +00.0, +00.0,
        +00.0, +01.0, +00.0, +00.0, +00.0, +00.0,
        +01.0, +00.0, +00.0, +00.0, +00.0, +00.0).finished();
      return basis;
    }

  };
}