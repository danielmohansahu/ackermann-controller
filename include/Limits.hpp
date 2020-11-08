#pragma once
/**
* @file Limits.hpp
* @brief Class declaration for Limits class
*
* Object containing limitations for the Ackermann rover to prevent sending
* steering or speed commands past allowable bounds.
*
* @author Spencer Elyard
* @author Daniel M. Sahu
* @author Santosh Kesani
* @copyright [2020]
*/

#include <memory>
#include <limits>

#include "Params.hpp"

namespace ackermann {

/**
* @brief Class used to apply known limits to a given desired control signal.
 */
class Limits {
 public:
   /**
   * @brief Constructor
   * @param params Shared pointer detailing rover characteristic parameters
   */
  explicit Limits(const std::shared_ptr<const Params>& params);
  Limits() = delete;

  /**
  * @brief Apply known limits to the given controller command.
   *
   * Apply known kinematic constraints (velocity, acceleration,
   * angular velocity, angular acceleration) to the given
   * potential commands, and return the limited version. (Return paramaters
   * noted)
   *
   * @param current_speed: Current speed in (m/s).
   * @param current_steering: Current commanded steering angle (rad).
   * @param desired_thottle: (Return Parameter) Desired throttle
   * (generally [0,1]).
   * @param desired_steering: (Return Parameter) Desired steering angle (rad).
   * @param desired_steering_vel: (Return parameter) Desired steering velocity
   * (rad/s).
   * @param dt: Fixed time step between commands (s).
  */
  void limit(const double current_speed,
                     const double current_steering,
                     const double current_steering_vel,
                     double& desired_throttle,
                     double& desired_steering,
                     double& desired_steering_vel,
                     double dt) const;

/**
* @brief Use Parameters structure to convert throttle to speed as a
  * function of maximum allowable speed.
  * @param throttle Throttle setting in range of generally [0,1]
  * @return Speed based on throttle input (linear relationship) (m/s)
  */
  double throttleToSpeed(double throttle) const;
  /**
  * @brief Use Parameters structure to convert speed to throttle as a
  * function of maximum allowable speed.
  * @param speed Speed in range of [0,max_speed]
  * @return Throttle position estimate from speed (linear relationship) [0,1]
  */
  double speedToThrottle(double speed) const;

  /**
  * @brief Calculate the direction to minimize turning angle (eg, don't turn
  * 270deg right if you can turn 90deg left
  * @param current_heading Current heading in radians
  * @param desired_heading Desired heading in radians
  * @return Angle and direction (+/-) to turn (rad)
  */
  double shortestArcToTurn(double current_heading, double desired_heading) const;

  /**
  * @brief Bound heading to [-pi,pi) range; prevents odd behavior.
  * @param current_heading Current heading in radians
  * @param desired_heading Desired heading in radians
  * @return Bound heading in radians
  */
  double boundHeading(const double heading) const;

 private:
  /**
  * @brief A copy of our configuration parameters.
  */
  const std::shared_ptr<const Params> params_;
};

} // namespace ackermann
