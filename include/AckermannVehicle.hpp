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

extern const double global_dt;

namespace VehicleControl {


  /* @brief This class is used to further define a vehicle with Ackermann steering
   *
   * This class implements a vehicle with Ackermann steering.
   */
class AckermannVehicle : public Vehicle {
  public:
    // @brief constructor
    AckermannVehicle(double maxSteeringAngle = 45., double maxSteeringRate = 10.);

    /* @brief Set the maximum steering angle (degrees)
     * @parameter maxAngle Maximum steering angle (degrees)
     */
    void set_max_steering_angle(double maxAngle);

    /* @brief Set the maximum steering angle rate of change (degrees/second)
     * @parameter maxAngleRate Maximum steering angle rate (degrees/second)
     */
    void set_max_steering_rate(double maxAngleRate);

    /* @brief Set the current steering angle (degrees)
     * @parameter currentSteeringAngle Current steering angle (degrees)
     */
    void set_current_steering_angle(double currentSteeringAngle);

    /* @brief Set the maximum steering angle (degrees)
     * @parameter None
     * @return Maximum steering angle (degrees) (double)
     */
    double get_max_steering_angle() const;

    /* @brief Set the maximum steering angle rate of change (degrees/second)
     * @parameter None
     * @return Maximum steering angle rate (degrees/second) (double)
     */
    double get_max_steering_rate() const;

    /* @brief Get the current steering angle (degrees)
     * @parameter None
     * @return Current steering angle (degrees) (double)
     */
    double get_current_steering_angle() const;

    /* @brief Calculate the speed of the left wheel on ground (ft/s)
     * @parameter None
     * @return Current speed of left wheel on ground (ft/s)
     */
    double CalcWheelSpeedLeft();

    /* @brief Calculate the speed of the right wheel on ground
     * @parameter None
     * @return Current speed of right wheel on ground (ft/s)
     */
    double CalcWheelSpeedRight();


    /* @brief STUB - Calculate Ackermann vehicle dynamics for vehicle
     * @parameter headingChange Relative heading change for vehicle
     * @parameter velocity Velocity of vehicle
     * @return None
     */
    void compute(double headingChange, double velocity);

    /* @brief Correct steering angle to maximum value and rates
     * @parameter None
     * @return None
     */
     void UpdateSteeringInput();

  private:
    // @brief Maximum steering angle
    double max_steering_angle_;
    // @brief Maximum steering rate
    double max_steering_rate_;
    // @brief Current steering angle
    double current_steering_angle_;
    // @brief Desired steering angle
    double desired_steering_angle_;
};
}
