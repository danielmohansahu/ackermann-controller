/* @file AckermannVehicle.cpp
 * @brief This is the class definition for a vehicle which
 * uses an Ackermann Steering Controller.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 */

#include <AckermannVehicle.hpp>

VehicleControl::AckermannVehicle::AckermannVehicle() {}

// stub implementation
double VehicleControl::AckermannVehicle::compute(double headingChange, double velocity)
{
  return 0.0;
}

// stub implementation
double VehicleControl::AckermannVehicle::CalcWheelSpeedLeft() {
  return 0.0;
}

// stub
double VehicleControl::AckermannVehicle::CalcWheelSpeedRight() {
  return 0.0;
}

void VehicleControl::AckermannVehicle::set_max_steering_angle(double newMaxAngle) {this->max_steering_angle_=newMaxAngle;}
void VehicleControl::AckermannVehicle::set_max_steering_rate(double newMaxRate) {this->max_steering_rate_=newMaxRate;}
void VehicleControl::AckermannVehicle::set_current_steering_angle(double currSteeringAngle) {this->current_steering_angle_ = currSteeringAngle;}

double VehicleControl::AckermannVehicle::return_max_steering_angle() const {return this->max_steering_angle_;}
double VehicleControl::AckermannVehicle::return_max_steering_rate() const {return this->max_steering_rate_;}
double VehicleControl::AckermannVehicle::return_current_steering_angle() const {return this->current_steering_angle_;}

// http://www.iaeng.org/publication/WCE2016/WCE2016_pp1062-1066.pdf
// https://www.xarg.org/book/kinematics/ackerman-steering/
