#include <array>
#include <Eigen/Core>

namespace pathetic::path {
  Eigen::Matrix<double, 6,6> get_coeff_matrix() {
    std::array<std::array<double, 6>, 6> coeffs = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 2.0, 0.0, 0.0},
      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
      {5.0, 4.0, 3.0, 2.0, 1.0, 0.0},
      {20.0, 12.0, 6.0, 2.0, 0.0, 0.0}
    };

    Eigen::Matrix<double, 6,6> coeff_matrix;
    for (uint8_t i = 0; i < 6; i++) {
      for (uint8_t j = 0; j < 6; j++) {
        coeff_matrix(i,j) = coeffs[i][j];
      }
    }

    return coeff_matrix;
  }

  Eigen::Matrix<double, 6,6> const coeff_matrix = get_coeff_matrix();

  class quintic_polynomial {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit quintic_polynomial(
      double start, double start_deriv, double start_second_deriv,
      double end, double end_deriv, double end_second_deriv
    ); 
    
    auto operator[](double t) const-> double;
    
    auto deriv(double t) const;

    auto second_deriv(double t) const;

    private:
    double a, b, c, d, e, f;

  }; 
}