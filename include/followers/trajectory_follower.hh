#pragma once

namespace pathetic::followers {
  class trajectory_follower {
    public:
    math::pose2d last_error;
    
    private:
    math::pose2d admissible_error;
    double timeout = 0.0;

  };
}