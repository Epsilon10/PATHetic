#include <array>
#include <Eigen/Core>

namespace pathetic::path {
  Eigen::Matrix<double, 6,6> get_coeff_matrix();

  // Eigen::Matrix<double, 6,6> const coeff_matrix = get_coeff_matrix();

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