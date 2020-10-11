#pragma once

/* @file AckermannVehicle.hpp
 * @brief Class declaration for an Ackermann vehicle
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include "Vehicle.hpp"
#include "State.hpp"

namespace ackermann
{

/* @brief This class is used to further define a vehicle with Ackermann steering
 *
 * This class implements a vehicle with Ackermann steering.
 */
class AckermannVehicle : public Vehicle
{
public:
  // @brief constructor
  explicit AckermannVehicle(double maxSteeringAngle = 45.0);

  /* @brief Set the maximum steering angle (degrees)
   * @parameter maxAngle Maximum steering angle (degrees)
   */
  void set_max_steering_angle(double maxAngle);

  /* @brief Set the maximum steering angle (degrees)
   * @parameter None
   * @return Maximum steering angle (degrees) (double)
   */
  double get_max_steering_angle() const;

  /* @brief Calculate Ackermann vehicle dynamics for vehicle
   * @parameter headingChange Relative heading change for vehicle
   * @parameter velocity Velocity of vehicle
   * @return None
   */
  State getState(double speed, double heading) override;

 private:
  // @brief Maximum steering angle
  double max_steering_angle_;
};

} // namespace ackermann
