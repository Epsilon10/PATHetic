#include "control/pid_coeffecients.hh"

namespace pathetic::control {
  pid_coeffecients::pid_coeffecients(double kP, double kI, double kD) 
    : kP(kP), kI(kI), kD(kD) { }
}