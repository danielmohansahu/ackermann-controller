#pragma once
/* @file Limits.hpp
 * @copyright [2020]
 */

#include <memory>
#include <limits>

#include "Params.hpp"

namespace ackermann {

/* @brief Class used to apply known limits to a given desired control signal.
 */
class Limits {
 public:
  /* @brief Constructor */
  explicit Limits(const std::shared_ptr<const Params>& params);

  /* @brief Apply known limits to the given controller command.
   *
   * Apply known kinematic constraints (velocity, acceleration,
   * angular velocity, angular acceleration) to the given
   * potential commands, and return the limited version.
   *
   * @param current_speed: Current speed.
   * @param current_steering: Current commanded steering angle.
   * @param desired_thottle: Desired throttle.
   * @param desired_steering: Desired steering angle.
   * @param dt: Fixed time step between commands.
  */
  void limit(const double current_speed,
                     const double current_steering,
                     const double current_steering_vel,
                     double& desired_throttle,
                     double& desired_steering,
                     double& desired_steering_vel,
                     double dt) const;

  /* @brief Use Parameters structure to convert throttle to speed as a
  * function of maximum allowable speed.
  * @param throttle Throttle setting in range of [0,1]
  */
  double throttleToSpeed(double throttle) const;
  /* @brief Use Parameters structure to convert speed to throttle as a
  * function of maximum allowable speed.
  * @param speed Speed in range of [0,max_speed]
  */
  double speedToThrottle(double speed) const;

  /* @brief Calculate the direction to minimize turning angle (eg, don't turn
  * 270deg right if you can turn 90deg left
  * @param current_heading Current heading in radians
  * @param desired_heading Desired heading in radians
  */
  double shortestArcToTurn(double current_heading, double desired_heading) const;

  /* @brief
  * @param current_heading Current heading in radians
  * @param desired_heading Desired heading in radians
  */
  double boundHeading(const double heading) const;

 private:
  // access to our parameters object (contains limits)
  const std::shared_ptr<const Params> params_;
};

} // namespace ackermann
