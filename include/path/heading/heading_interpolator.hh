#pragma once

#include "path/parametric_curve.hh"
#include <memory> 

namespace pathetic::path::heading {
  class heading_interpolator {
    public:
    heading_interpolator(std::unique_ptr<parametric_curve> curve);
    std::unique_ptr<parametric_curve> curve;
    auto get(double s, double t) const -> double;
    auto get(double s) const -> double;

    auto deriv(double s, double t) const -> double;
    auto deriv(double s) const -> double;

    auto second_deriv(double s, double t) const -> double;
    auto second_deriv(double s) const -> double;

    auto start() const -> double;
    auto end() const -> double;

    auto start_deriv() const -> double;
    auto start_second_deriv() const -> double;

    auto end_deriv() const -> double;
    auto end_second_deriv() const -> double;

    private:
    virtual auto internal_get(double s, double t) const -> double;
    virtual auto internal_deriv(double s, double t) const -> double;
    virtual auto internal_second_deriv(double s, double t) const -> double;
  };
}