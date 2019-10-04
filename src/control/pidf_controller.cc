#include "control/pidf_controller.hh"
#include "math/general.hh"
#include <chrono>
#include <algorithm>

namespace pathetic::control {

  pidf_controller::pidf_controller(
    pid_coeffecients pid, double kV, double kA, double kStatic, 
    std::function<double(double)>& kF) 
      : kP(pid.kP), kI(pid.kI), kD(pid.kD), kV(kV), kA(kA), kF(kF) { }
 

  auto pidf_controller::set_input_bounds(double min, double max) -> void {
    if (min < max) {
      input_bounded = true;
      min_input = min;
      max_input = max;
    }  
  }
  
  
  auto pidf_controller::set_output_bounds(double min, double max) -> void {
    if (min < max) {
      output_bounded = true;
      min_output = min;
      max_output = max;
    }
  }

  auto pidf_controller::get_error(double position) -> double {
    auto error = target_position - position;
    if (input_bounded) {
      auto input_range = max_output - min_input;
      while (std::abs(error) > input_range / 2.0) 
        error -= math::sign(error) * input_range;
    }

    return error;
  }

  auto pidf_controller::update(double position, double velocity, double acceleration) -> double {
    auto const now = std::chrono::high_resolution_clock::now();
    auto const error = get_error(position);
    
    if (!has_one_update) {
      last_error = error;
      last_update_timestamp = std::move(now);
      has_one_update = true;
      return 0.0;
    } else {
        auto dt = std::chrono::duration_cast<std::chrono::seconds>(now - last_update_timestamp).count();
        error_sum += 0.5 * (error + last_error) * dt;
        auto error_deriv = (error - last_error) / dt;

        last_error = error;
        last_update_timestamp = std::move(now);

        auto base_output = kP * error + kI * error_sum + kD * (error_deriv - velocity) +
          kV * velocity + kA * acceleration + kF(position);
        
        auto output =  math::floating_point_eq(base_output, 0.0) ? 0.0 : base_output + math::sign(base_output) 
          * kStatic;

        return output_bounded ? std::max(min_output, std::min(output, max_output)) : output; 
    }
  }

  auto pidf_controller::reset() {
    error_sum = 0.0;
    last_error = 0.0;
    has_one_update = false;
  }

} 
