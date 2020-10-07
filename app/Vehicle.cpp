/* @file controller.cpp
 * @brief This is the class definition for a vehicle
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 */

#include <Vehicle.hpp>

// constructor
VehicleControl::Vehicle::Vehicle(double wheelbase, double track,
  double vehicle_speed, double vehicle_heading, double vehicle_x,
  double vehicle_y)
: wheelbase_{wheelbase}, track_{track}, vehicle_speed_{vehicle_speed},
vehicle_heading_{vehicle_heading}, vehicle_x_{vehicle_x}, vehicle_y_{vehicle_y}
 {}

void VehicleControl::Vehicle::set_wheelbase(double wheelbase) {
    wheelbase_ = wheelbase;
}

void VehicleControl::Vehicle::set_track(double track) {
    track_ = track;
}

void VehicleControl::Vehicle::set_vehicle_speed(double vehicle_speed) {
    vehicle_speed_ = vehicle_speed;
}

void VehicleControl::Vehicle::set_vehicle_heading(double vehicle_heading) {
  vehicle_heading_ = vehicle_heading;
}

void VehicleControl::Vehicle::set_vehicle_x(double new_x) {
  vehicle_x_ = new_x;
}

void VehicleControl::Vehicle::set_vehicle_y(double new_y) {
  vehicle_y_ = new_y;
}

double VehicleControl::Vehicle::get_wheelbase() const {return this->wheelbase_;}
double VehicleControl::Vehicle::get_track() const {return this->track_;}
double VehicleControl::Vehicle::get_vehicle_speed() const {return this->vehicle_speed_;}
double VehicleControl::Vehicle::get_vehicle_heading() const {return this->vehicle_heading_;}
double VehicleControl::Vehicle::get_vehicle_x() const {return this->vehicle_x_;}
double VehicleControl::Vehicle::get_vehicle_y() const {return this->vehicle_y_;}
