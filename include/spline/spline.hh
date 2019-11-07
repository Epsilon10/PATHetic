#pragma once

#include <Eigen/Core>
#include "math/pose2d.hh"
#include "math/vector2d.hh"
#include <cmath>
#include <array>

namespace pathetic {
  struct waypoint {
    waypoint(double x, double y, double dx, double dy, double d2x, double d2y)
      : x(x), y(y), dx(dx), dy(dy), d2x(d2x), d2y(d2y) { }

    double x, y, dx, dy, d2x, d2y;
  };

  template <std::uint32_t Degree>
  class spline {
    public:
    Spline() = default;

    // x[2] is second deriv, x[1] is first deriv ect
    struct control_vector {
      std::array<double, (Degree + 1) / 2> x;
      std::array<double, (Degree + 1) / 2> y;
    };

    auto get_point(double t) -> math::vector2d {
      auto const combined = get_combined_vector(t);
      return math::vector2d{combined(0), combined(1)};
    }

    auto deriv(double t) -> math::vector2d {
      double dx,dy;

      if (t==0.0) {
        dx = coefficients()(2, Degree - 1);
        dy = coefficients()(3, Degree - 1);
      } else {
        auto const combined = get_combined_vector(t);
        dx = combined(2) / t / t;
        dy = combined(3) / t / t;
      }

      return math::vector2d{dx, dy};
    }

    auto second_deriv(double t) -> math::vector2d {
      double d2x, d2y;

      if (t==0.0) {
        dx = coefficients()(4, Degree - 2);
        dy = coefficients()(5, Degree - 2);
      } else {
        auto const combined = get_combined_vector(t);
        dx = combined(4) / t / t;
        dy = combined(5) / t / t;
      }

      return math::vector2d{d2x, d2y};
    }

    auto curvature(double t) -> double {
      auto const _deriv = deriv(t);
      auto const _deriv_norm = _deriv.norm();
      auto const _second_deriv = second_deriv(t);
      return std::abs(_second_deriv.x * _deriv.y - _deriv.x * _second_deriv.y) / std::pow(_deriv_norm, 3);
    }

    protected:
    virtual Eigen::Matrix<double, 6, Degree + 1> coefficients() const = 0;

    private:
    auto get_combined_vector(double t) -> Eigen::Matrix<double, 6, 1>{
      Eigen::Matrix<double, Degree + 1, 1> polynomial_bases;
      for (std::uint32_t i = 0; i <= Degree; i++) 
        polynomial_bases(i) = std::pow(t, Degree - i);
      return coefficients() * polynomial_bases;
    }
  };

}