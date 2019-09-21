#include "path/parametric_curve.hh"

namespace pathetic::path {
    auto parametric_curve::get(double s) const -> math::vector2d {
      auto t = reparam(s);
      return pnml_get(t);
    }

    auto parametric_curve::get(double s, double t) const -> math::vector2d {
      return pnml_get(t);
    }

    auto parametric_curve::deriv(double s) -> math::vector2d {
      auto t = reparam(s);
      return pnml_deriv(t) * param_deriv(t);
    }

    auto parametric_curve::deriv(double s, double t) -> math::vector2d {
      return pnml_deriv(t) * param_deriv(t);
    }

    auto parametric_curve::second_deriv(double s) -> math::vector2d {
      auto t = reparam(s);
      return pnml_second_deriv(t) * param_deriv(t) * param_deriv(t)
        * pnml_deriv(t) * param_second_deriv(t);
    }

    auto parametric_curve::second_deriv(double s, double t) -> math::vector2d {
      return pnml_second_deriv(t) * param_deriv(t) * param_deriv(t)
        * pnml_deriv(t) * param_second_deriv(t);
    }

    auto parametric_curve::start() {
      return get(0.0,0.0);
    }

    auto parametric_curve::start_deriv() {
      return deriv(0.0, 0.0);
    }

    auto parametric_curve::start_second_deriv() {
      return second_deriv(0.0, 0.0);
    }

    auto parametric_curve::end() {
      return get(length(), 1);
    }

    auto parametric_curve::end_deriv() {
      return deriv(length(), 1.0);
    }

    auto parametric_curve::end_second_deriv() {
      return second_deriv(length(), 1.0);
    }

    auto parametric_curve::tangent_angle(double s, double t) {
      return deriv(s, t).angle();
    }

    auto parametric_curve::tangent_angle_deriv(double s) {
      auto t = reparam(s);
      auto _deriv = deriv(s,t);
      auto _second_deriv = second_deriv(s,t);

      return _deriv.x * _second_deriv.y - _deriv.y * _second_deriv.x;
    }

    auto parametric_curve::tangent_angle_deriv(double s, double t) {
      auto _deriv = deriv(s,t);
      auto _second_deriv = second_deriv(s,t);

      return _deriv.x * _second_deriv.y - _deriv.y * _second_deriv.x;
    }
}