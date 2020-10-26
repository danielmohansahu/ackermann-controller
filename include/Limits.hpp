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
   * @param current_thottle: Current commanded throttle.
   * @param current_steering: Current commanded steering angle.
   * @param desired_thottle: Desired throttle.
   * @param desired_steering: Desired steering angle.
   * @param dt: Fixed time step between commands.
  */
  void limit(const double current_throttle,
             const double current_steering,
             double& desired_throttle,
             double& desired_steering,
             double dt) const;

 private:
  // access to our parameters object (contains limits)
  const std::shared_ptr<const Params> params_;
};

} // namespace ackermann

