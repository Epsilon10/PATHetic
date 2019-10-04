#pragma once

namespace pathetic::control {
  class pid_coeffecients {
    public:
    pid_coeffecients(double kP, double kI, double kD);
    double kP, kI, kD;
  };
}