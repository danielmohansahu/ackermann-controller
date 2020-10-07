#pragma once

/* @file Controller.hpp
 * @brief Top level ACME Ackermann Controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */

#include "State.hpp"
#include "AckermannVehicle.hpp"
#include "PID.hpp"
#include "Limits.hpp"

namespace ackermann
{

class Controller
{
 public:

  // @brief constructor
  Controller();

  /* @brief Calculate the appropriate wheel velocities for the given setpoints.
   */
  // @TODO [Daniel Sahu] figure out a better interface API for this method.
  void update(double current_speed, double current_heading, double desired_speed, double desired_heading, double& left, double& right);

 private:
  
  // @brief Ackermann model (used in translating speed/heading into wheel speeds)
  std::unique_ptr<AckermannVehicle> model_;
  
  // @brief Object used to apply kinematic constraints to a calculated command (to prevent saturation)
  std::unique_ptr<Limits> limits_;

  // @brief Internal encapsulated PID controller
  std::unique_ptr<PID> pid_;
};

} // namespace ackermann
