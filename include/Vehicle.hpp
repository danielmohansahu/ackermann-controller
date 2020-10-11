#pragma once
/* @file Vehicle.hpp
 * @brief Class declaration for a generic vehicle.
 * 
 * This class encapsulates a generic vehicle that provides a method for translating
 * heading and velocity into wheel speeds.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */

#include "State.hpp"

class Vehicle
{
 public:
  /* @brief constructor
   */
  Vehicle(double wheel_base, double wheel_separation);

  /* @brief Core API that converts heading/speed into wheel velocities.
   */
  virtual ackermann::State getState(double speed, double heading) = 0;

  /* @brief Set the vehicle wheelbase (axle distance front/back)
   * @parameter wheelbase Distance
   */
  void set_wheel_base(double wheel_base);

  /* @brief Set the vehicle track (wheel distance left/right)
   * @parameter track Distance
   */
  void set_wheel_separation(double wheel_separation);

  /* @brief Get vehicle wheelbase
   * @return Wheelbase
   */
  double get_wheel_base() const;

  /* @brief Get vehicle wheel separation
   * @return wheel separation
   */
  double get_wheel_separation() const;

 private:
  // @brief Vehicle wheelbase
  double wheel_base_;

  // @brief Vehicle track
  double wheel_separation_;
};
