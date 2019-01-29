#pragma once

#include "racecar_simulator/pose_2d.hpp"

namespace racecar_simulator {

class AckermannKinematics {

  public:

    static Pose2D update(
        const Pose2D start, 
        double velocity, 
        double steering_angle, 
        double wheelbase, 
        double dt);

};

}
