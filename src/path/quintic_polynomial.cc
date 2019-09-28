#include "path/quintic_polynomial.hh"
#include <Eigen/LU>

namespace pathetic::path {
  Eigen::Matrix<double, 6,6> get_coeff_matrix()  {
    std::array<std::array<double, 6>, 6> coeffs = {{
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 2.0, 0.0, 0.0},
      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
      {5.0, 4.0, 3.0, 2.0, 1.0, 0.0},
      {20.0, 12.0, 6.0, 2.0, 0.0, 0.0}
    }};

    Eigen::Matrix<double, 6,6> coeff_matrix;
    for (uint8_t i = 0; i < 6; i++) {
      for (uint8_t j = 0; j < 6; j++) {
        coeff_matrix(i,j) = coeffs[i][j];
      }
    }

    return coeff_matrix;
  }
  
  quintic_polynomial::quintic_polynomial(
      double start, double start_deriv, double start_second_deriv,
      double end, double end_deriv, double end_second_deriv
  ) {
    Eigen::Matrix<double, 6, 1> target;
    target(0,0) = start;
    target(1,0) = start_deriv;
    target(2,0) = start_second_deriv;
    target(3,0) = end;
    target(4,0) = end_deriv;
    target(5,0) = end_second_deriv;

    auto const result = get_coeff_matrix().partialPivLu().solve(target);
      
    b = result(1,0);
    c = result(2,0);
    d = result(3,0);
    e = result(4,0);
    f = result(5,0);
  }

  auto quintic_polynomial::operator[](double t) const-> double {
    return (a * t + b) * (t * t * t * t) + c * (t * t * t) + d * (t * t) + e * t + f;
  }

  auto quintic_polynomial::deriv(double t) const{
    return (5 * a * t + 4 * b) * (t * t * t) + (3 * c * t + 2 * d) * t + e;
  }

  auto quintic_polynomial::second_deriv(double t) const{
    return (20 * a * t + 12 * b) * (t * t) + 6 * c * t + 2 * d;
  }


}