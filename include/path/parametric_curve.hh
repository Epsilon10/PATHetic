#pragma once

#include "math/vector2d.hh"
#include <vector>

namespace pathetic::path {
  class parametric_curve {
    public:
    virtual auto length() const -> double = 0;
    virtual auto reparam(double s) const -> double = 0;
    // virtual auto reparam(std::vector<double> const& s) const -> void;

    virtual auto pnml_get(double t) const -> math::vector2d = 0;
    virtual auto pnml_deriv(double t) const -> math::vector2d = 0;
    virtual auto pnml_second_deriv(double t) const -> math::vector2d = 0;
    
    virtual auto param_deriv(double t) const -> double = 0;
    virtual auto param_second_deriv(double t) const -> double = 0;

    parametric_curve() = default;

    auto get(double s) const -> math::vector2d;

    auto get(double s, double t) const -> math::vector2d;

    auto deriv(double s) -> math::vector2d;

    auto deriv(double s, double t) -> math::vector2d;

    auto second_deriv(double s) -> math::vector2d;

    auto second_deriv(double s, double t) -> math::vector2d;

    auto start();

    auto start_deriv();

    auto start_second_deriv();

    auto end();

    auto end_deriv();

    auto end_second_deriv();

    auto tangent_angle(double s, double t);

    auto tangent_angle_deriv(double s);

    auto tangent_angle_deriv(double s, double t);

  };
}