#pragma once

/* @file Vehicle.hpp
 * @brief Class declaration for a generic vehicle
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <iostream>
  // @brief global_dt Global timestep variable - keep consistent
  extern const double global_dt;

namespace VehicleControl {

class Vehicle
{
  public:
    // @brief constructor
    Vehicle(double wheelbase = 1., double track = 1., double vehicle_speed = 0.,
       double vehicle_heading = 0., double vehicle_x = 0.,
        double vehicle_y = 0.);

        /* @brief Set the vehicle wheelbase (axle distance front/back)
         * @parameter wheelbase Distance (ft)
         */
    void set_wheelbase(double wheelbase);
    /* @brief Set the vehicle track (wheel distance left/right)
     * @parameter track Distance (ft)
     */
    void set_track(double track);
    /* @brief Set the vehicle speed
     * @parameter vehicle_speed (ft/s)
     */
    void set_vehicle_speed(double vehicle_speed);
    /* @brief Set the vehicle heading (global frame)
     * @parameter vehicle_heading (degrees)
     */
    void set_vehicle_heading(double vehicle_heading);
    /* @brief Set the vehicle X position (global frame)
     * @parameter new_x new X position (ft)
     */
    void set_vehicle_x(double new_x);
    /* @brief Set the vehicle Y position (global frame)
     * @parameter new_Y new Y position (ft)
     */
    void set_vehicle_y(double new_y);

    /* @brief Get vehicle wheelbase
     * @parameter None
     * @return Wheelbase (ft)
     */
    double get_wheelbase() const;
        /* @brief Get vehicle track
         * @parameter None
         * @return Track (ft)
         */
    double get_track() const;

        /* @brief Get vehicle speed
         * @parameter None
         * @return Speed (ft/s)
         */
    double get_vehicle_speed() const;

        /* @brief Get vehicle heading (global frame)
         * @parameter None
         * @return Heading (degrees)
         */
    double get_vehicle_heading() const;

        /* @brief Get vehicle X position (global frame)
         * @parameter None
         * @return X Position (ft)
         */
    double get_vehicle_x() const;
    /* @brief Get vehicle Y position (global frame)
     * @parameter None
     * @return Y Position (ft)
     */
    double get_vehicle_y() const;

  private:
    // @brief Vehicle wheelbase (ft)
    double wheelbase_;
    // @brief Vehicle track (ft)
    double track_;
    // @brief Vehicle speed (ft/s)
    double vehicle_speed_;
    // @brief Vehicle heading (degrees)
    double vehicle_heading_;
    // @brief Vehicle X position (ft)
    double vehicle_x_;
    // @brief Vehicle Y position (ft)
    double vehicle_y_;
};
}
