#include "profile/motion_segment.hh"

namespace pathetic::profile {
  motion_segment::motion_segment(motion_state const& start, double dt) 
    : start(start), dt(dt) { }
  
  auto motion_segment::operator[](double t) const -> motion_state {
    return start[t];
  }

  auto motion_segment::end() const -> motion_state {
    return start[dt];
  }

  auto motion_segment::reversed() const -> motion_segment {
    auto const _end = end();
    auto const state = motion_state{_end.x, _end.v, -_end.a, -_end.j};
    return motion_segment{state, dt};
  }
}