#pragma once

/* @file AckermannVehicle.hpp
 * @brief Class declaration for an Ackermann vehicle
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <iostream>
#include <Vehicle.hpp>
#include <PIDController.hpp>

namespace VehicleControl {
class AckermannVehicle : public Vehicle {
  public:
    // @brief constructor
    AckermannVehicle();

    void set_max_steering_angle(double);
    void set_max_steering_rate(double);
    void set_current_steering_angle(double);

    double return_max_steering_angle() const;
    double return_max_steering_rate() const;
    double return_current_steering_angle() const;

    // @brief core computation method
    double compute(double, double);

    double CalcWheelSpeedLeft();
    double CalcWheelSpeedRight();

  private:
    double max_steering_angle_;
    double max_steering_rate_;
    double current_wheel_angle;
    double current_steering_angle_;
    double wheel_speed_left_;
    double wheel_speed_right_;
};
}
