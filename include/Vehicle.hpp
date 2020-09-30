#pragma once

/* @file Vehicle.hpp
 * @brief Class declaration for a generic vehicle
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <iostream>

namespace VehicleControl {
class Vehicle
{
  public:
    // @brief constructor
    Vehicle(double wheelbase = 1., double track = 1., double vehicle_speed = 0., double vehicle_heading = 0.);
    void set_wheelbase(double);
    void set_track(double);
    void set_vehicle_speed(double);
    void set_vehicle_heading(double);

    double get_wheelbase() const;
    double get_track() const;
    double get_vehicle_speed() const;
    double get_vehicle_heading() const;

  private:
    double wheelbase_;
    double track_;
    double vehicle_speed_;
    double vehicle_heading_;
};
}
