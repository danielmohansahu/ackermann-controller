/* @file controller.cpp
 * @brief This is the class definition for a vehicle
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Sentosh Kesani
 *
 * @copyright [2020]
 */

#include <Vehicle.hpp>

using ackermann::State;

Vehicle::Vehicle(double wheel_base, double wheel_separation)
 : wheel_base_(wheel_base),
   wheel_separation_(wheel_separation)
{
}

State Vehicle::getState(double speed, double heading)
{
  return State();
}

void Vehicle::set_wheel_base(double wheel_base)
{
  return;
}

void Vehicle::set_wheel_separation(double wheel_separation)
{
  return;
}

double Vehicle::get_wheel_base()
{
  return 0.0;
}

double Vehicle::get_wheel_separation()
{
  return 0.0;
}
