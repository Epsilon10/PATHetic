#pragma once

#include "math/vector2d.hh"

namespace pathetic::path {
  class parametric_curve {
    public:
    virtual auto length() -> double = 0;
    virtual auto reparam(double s) -> double = 0;
    virtual auto reparam(std::vector<double> const& s) -> void;

    virtual auto pnml_get(double t) -> math::vector2d = 0;
    virtual auto pnml_deriv(double t) -> math::vector2d = 0;
    virtual auto pnml_second_deriv(double t) -> math::vector2d = 0;
    
    virtual auto param_deriv(double t) -> double = 0;
    virtual auto param_second_deriv(double t) -> double = 0;

    parametric_curve() = default;

    auto operator[](double s, double t = reparam(s)) -> math::vector2d {
      return pnml_get(t);
    }

    auto deriv(double s, double t = reparam(s)) -> math::vector2d {
      return pnml_deriv(t) * param_deriv(t);
    }

    auto second_deriv(double s, double t = reparam(s)) -> math::vector2d {
      return pnml_second_deriv(t) * param_deriv(t) * param_deriv(t)
        * pnml_deriv(t) * param_second_deriv(t);
    }

    auto start() {
      return this[0.0,0.0];
    }

    auto start_deriv() {
      return deriv(0.0, 0.0);
    }

    auto start_second_deriv() {
      return second_deriv(0.0, 0.0);
    }

    auto end() {
      return this[length(), 1.0];
    }

    auto end_deriv() {
      return deriv(length(), 1.0);
    }

    auto end_second_deriv() {
      return second_deriv(length(), 1.0);
    }
  };
}