#include "spline/quintc_hermite_spline.hh"

namespace pathetic {
  quintic_hermite_spline::quintic_hermite_spline(
    waypoint const& start, waypoint const& end, double max_delta_k = .01, 
    double max_segment_length = 0.25
  ) {
    Eigen::Matrix<double, 6, 1> x;
    x << start.x, start.dx, start.d2x, end.x, end.dx, end.d2x;

    Eigen::Matrix<double, 6, 1> y;
    y << start.y, start.dy, start.d2y, end.y, end.dy, end.d2y;

    auto const hermite = make_hermite_basis();

    auto const x_coeffs = (hermite * x).transpose();
    auto const y_coeffs = (hermite * y).transpose();

    for (std::uint32_t i = 0; i < 6; i++) {
      coefficients(0, i) = x_coeffs(0,i);
      coefficients(1, i) = y_coeffs(0,i);

      // Populate Row 2 and Row 3 with the derivatives of the equations above.
      // Then populate row 4 and 5 with the second derivatives.
      coefficients(2, i) = coefficients(0, i) * (5 - i);
      coefficients(3,i) = coefficients(1, i) * (5 - i);
      coefficients(4,i) = coefficients(2, i) * (5 - i);
      coefficients(5,i) = coefficients(3, i) * (5 - i);
    }
  }

  auto quintic_hermite_spline::get_coeffecients() const -> Eigen::Matrix<double,6,6> {
    return coefficients;
  }
}