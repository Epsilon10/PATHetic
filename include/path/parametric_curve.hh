#pragma once

#include "math/vector2d.hh"
#include <vector>

namespace pathetic::path {
  class parametric_curve {
    public:
    virtual auto length() const -> double = 0;
    virtual auto reparam(double s) const -> double = 0;
    virtual auto reparam(std::vector<double> const& s) const -> void;

    virtual auto pnml_get(double t) const -> math::vector2d = 0;
    virtual auto pnml_deriv(double t) const -> math::vector2d = 0;
    virtual auto pnml_second_deriv(double t) const -> math::vector2d = 0;
    
    virtual auto param_deriv(double t) const -> double = 0;
    virtual auto param_second_deriv(double t) const -> double = 0;

    parametric_curve() = default;

    auto get(double s) const -> math::vector2d {
      auto t = reparam(s);
      return pnml_get(t);
    }

    auto get(double s, double t) const -> math::vector2d {
      return pnml_get(t);
    }

    auto deriv(double s) -> math::vector2d {
      auto t = reparam(s);
      return pnml_deriv(t) * param_deriv(t);
    }

    auto deriv(double s, double t) -> math::vector2d {
      return pnml_deriv(t) * param_deriv(t);
    }

    auto second_deriv(double s) -> math::vector2d {
      auto t = reparam(s);
      return pnml_second_deriv(t) * param_deriv(t) * param_deriv(t)
        * pnml_deriv(t) * param_second_deriv(t);
    }

    auto second_deriv(double s, double t) -> math::vector2d {
      return pnml_second_deriv(t) * param_deriv(t) * param_deriv(t)
        * pnml_deriv(t) * param_second_deriv(t);
    }

    auto start() {
      return get(0.0,0.0);
    }

    auto start_deriv() {
      return deriv(0.0, 0.0);
    }

    auto start_second_deriv() {
      return second_deriv(0.0, 0.0);
    }

    auto end() {
      return get(length(), 1);
    }

    auto end_deriv() {
      return deriv(length(), 1.0);
    }

    auto end_second_deriv() {
      return second_deriv(length(), 1.0);
    }

    auto tangent_angle(double s, double t = reparam(s)) {
      return deriv(s, t).angle();
    }

    auto tangent_angle_deriv(double s) {
      auto t = reparam(s);
      auto _deriv = deriv(s,t);
      auto _second_deriv = second_deriv(s,t);

      return _deriv.x * _second_deriv.y - _deriv.y * _second_deriv.x;
    }

    auto tangent_angle_deriv(double s, double t) {
      auto _deriv = deriv(s,t);
      auto _second_deriv = second_deriv(s,t);

      return _deriv.x * _second_deriv.y - _deriv.y * _second_deriv.x;
    }

  };
}