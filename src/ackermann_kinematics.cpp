#include <cmath>

#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/ackermann_kinematics.hpp"

using namespace racecar_simulator;

Pose2D AckermannKinematics::update(
    const Pose2D start, 
    double velocity, 
    double steering_angle, 
    double wheelbase, 
    double dt) {

  Pose2D end;

  // dthetadt = v * tan(steering_angle)/wheelbase
  double dthetadt = velocity * std::tan(steering_angle) / wheelbase;
  end.theta = start.theta + dthetadt * dt;

  // The solution to the integral of
  // dxdt = v * cos(theta)
  // dydt = v * cos(theta)
  if (dthetadt == 0) {
    end.x = start.x + dt * velocity * std::cos(end.theta);
    end.y = start.y + dt * velocity * std::sin(end.theta);
  } else {
    end.x = start.x + (velocity/dthetadt) * (std::sin(end.theta) - std::sin(start.theta));
    end.y = start.y + (velocity/dthetadt) * (std::cos(start.theta) - std::cos(end.theta));
  }

  return end;
}
