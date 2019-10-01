#include "profile/motion_state.hh"

namespace pathetic::profile {
  motion_state::motion_state(double x, double v, double a, double j) 
    : x(x), v(v), a(a), j(j) { }
  
  auto motion_state::operator[](double t) const -> motion_state {
    return motion_state{
      x + v * t + a / 2 * t * t + j / 6 * t * t * t,
      v + a * t + j / 2 * t * t,
      a + j * t,
      j
    };
  } 

  auto motion_state::flipped() const -> motion_state {
    return motion_state{-x, -v, -a, -j};
  }
}