/* @file AckermannVehicle.cpp
 * @brief This is the class definition for a vehicle which
 * uses an Ackermann Steering Controller.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 * 
 * http://www.iaeng.org/publication/WCE2016/WCE2016_pp1062-1066.pdf
 * https://www.xarg.org/book/kinematics/ackerman-steering/
 */

#include <AckermannVehicle.hpp>

// @TODO Currently STUB implementation; needs to be filled

namespace ackermann
{

AckermannVehicle::AckermannVehicle(double maxSteeringAngle)
 : max_steering_angle_(maxSteeringAngled)
{
}

void AckermannVehicle::set_max_steering_angle(double maxAngle)
{
  return;
}

double AckermannVehicle::get_max_steering_angle()
{
  return 0.0;
}

State AckermannVehicle::getState(double speed, double heading)
{
  return State();
}

} // namespace ackermann
