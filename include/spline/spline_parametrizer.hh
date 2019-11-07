#pragma once 


#include "spline/spline.hh"
#include "math/vector2d.hh"
#include "math/general.hh"

#include <cmath>
#include <vector>

namespace pathetic {
  class spline_parametrizer {
    template<std::uint32_t Degree>
    static auto parametrize(
      spline<Degree> const& spline, double& length, double t_lo, double t_hi, 
      std::vector<double>& t_samples, std::vector<double>& s_samples
     ) -> void {
       auto const v_lo = spline.get_point(t_lo);
       auto const v_hi = spline.get_point(t_hi);
       parametrize(spline, length, t_lo, t_hi, v_lo, v_hi, t_samples, s_samples);
    } 
    
    private:
    double static constexpr max_delta_k = 0.01;
    double static constexpr max_segment_length = 0.25;

    template<std::uint32_t Degree>
    static auto parametrize(
      spline<Degree> const& spline, double& length, double t_lo, double t_hi, 
      math::vector2d v_lo, math::vector2d v_hi, std::vector<double>& t_samples, 
      std::vector<double>& s_samples
    ) -> void {
      auto const t_mid = 0.5 * (t_lo + t_hi);
      auto v_mid = spline.get_point(t_mid);

      auto const delta_k = std::abs(spline.curvature(t_lo) - spline.curvature(t_hi));
      auto const segment_length = approx_length(v_lo, v_mid, v_hi);
      
      if (delta_k > max_delta_k || segment_length > max_segment_length) {
        parametrize(spline, length, t_lo, t_mid, v_lo, v_mid, t_samples, s_samples);
        parametrize(spline, length, t_mid, t_hi, v_mid, v_hi, t_samples, s_samples);
      } else {
        length += segment_length;
        s_samples.add(length);
        t_samples.add(t_hi);
      }
    }

    static auto approx_length(math::vector2d const& v1, math::vector2d const& v2, math::vector2d const& v3) -> double {
      auto const w1 = (v2 - v1) * 2.0;
      auto const w2 = (v2 - v3) * 2.0;
      auto const det = w1.x * w2.y - w2.x * w1.y;
      auto const chord = v1.dist_to(v3);

      if (math::floating_point_eq(det, 0.0)) {
        return chord;
      } else {
        auto const x1 = v1.x * v1.x + v1.y * v1.y;
        auto const x2 = v2.x * v2.x + v2.y * v2.y;
        auto const x3 = v3.x * v3.x + v3.y * v3.y;

        auto const y1 = x2 - x1;
        auto const y2 = x2 - x3;

        auto const origin = math::vector2d{y1 * w2.y - y2 * w1.y, y2 * w1.x - y1 * w2.x} / det;
        auto const radius = origin.dist_to(v1);

        return 2.0 * radius * std::asin(chord / (2.0 * radius));
      }
    }

    static auto interp(double s, double s_lo, double s_hi, double t_lo, double t_hi) -> double {
      return t_lo + (s - s_lo) * (t_hi - t_lo) / (s_hi - s_lo);
    }

    static auto reparam(double s, double length, std::vector<double> const& t_samples, std::vector<double> const& s_samples)
      -> double {
        if (s <= 0.0) return 0.0;
        if (s >= length) return 1.0;

        auto lo = 0;
        auto hi = s_samples.size();

        while (lo <= hi) {
          auto const mid = (hi + lo) / 2;
          if (s < s_samples[mid])
            hi = mid - 1;
          else if (s > s_samples[mid])
            lo = mid + 1;
        }
        
        return interp(s, s_samples[lo], s_samples[hi], t_samples[lo], t_samples[hi]);
    }
  };
}