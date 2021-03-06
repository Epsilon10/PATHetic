#pragma once

#include <cmath>
#include <functional>
#include <chrono>

#include "control/pid_coeffecients.hh"
namespace pathetic::control {

class pidf_controller {
  public:
    explicit pidf_controller(
        pid_coeffecients pid, double kV, double kA, double kStatic, 
        std::function<double(double)>& kF);
    
    auto set_input_bounds(double min, double max) -> void;
    auto set_output_bounds(double min, double max) -> void;
    auto get_error(double position) -> double;
    auto update(double position, double velocity = 0.0, double acceleration = 0.0) -> double;
    auto reset();
  private:
    double kP, kI, kD;
    double kV, kA, kStatic;
    std::function<double(double)>& kF;
    
    std::chrono::time_point<std::chrono::high_resolution_clock> last_update_timestamp;
    double error_sum = 0.0;
    double min_input = 0.0;
    double max_input = 0.0;
    double min_output = 0.0;
    double max_output = 0.0;

    double target_position = 0.0;
    double last_error = 0.0;

    bool input_bounded = false;
    bool output_bounded = false;
    bool has_one_update = false;
}; 

}
