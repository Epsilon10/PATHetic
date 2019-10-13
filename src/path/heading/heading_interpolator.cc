#include "path/heading/heading_interpolator.hh"

namespace pathetic::path::heading {
  heading_interpolator::heading_interpolator(std::unique_ptr<parametric_curve> curve) 
    : curve(std::move(curve)) { }
  
  auto heading_interpolator::get(double s, double t) const -> double {
    return internal_get(s,t);
  }

  auto heading_interpolator::get(double s) const -> double {
    auto const t = curve->reparam(s);
    return internal_get(s,t);
  }

  auto heading_interpolator::deriv(double s, double t) const -> double {
    return internal_deriv(s,t);
  }

  auto heading_interpolator::deriv(double s) const -> double {
    auto const t = curve->reparam(s);
    return internal_deriv(s,t);
  }

  auto heading_interpolator::second_deriv(double s, double t) const -> double {
    return internal_deriv(s,t);
  }

  auto heading_interpolator::second_deriv(double s) const -> double {
    auto const t = curve->reparam(s);
    return internal_deriv(s,t);
  }

  auto heading_interpolator::start() const -> double {
    return get(0.0,0.0);
  }

  auto heading_interpolator::end() const -> double {
    return get(curve->length(), 1.0);
  }

  auto heading_interpolator::start_deriv() const -> double {
    return deriv(0.0,0.0);
  }

  auto heading_interpolator::end_deriv() const -> double {
    return deriv(curve->length(), 1.0);
  }

  auto heading_interpolator::start_second_deriv() const -> double {
    return second_deriv(0.0,0.0);
  }

  auto heading_interpolator::end_second_deriv() const -> double {
    return second_deriv(curve->length(), 1.0);
  }
}