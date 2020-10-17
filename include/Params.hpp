#pragma once

/* @file Params.hpp
 * @brief Parameters used in the ackermann controller system.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */
#include <limits>

namespace ackermann {

struct Params {
  // default value parameters
  double control_frequency {100.0};
  double velocity_max {std::numeric_limits<double>::max()};
  double velocity_min {std::numeric_limits<double>::min()};
  double acceleration_max {std::numeric_limits<double>::max()};
  double acceleration_min {std::numeric_limits<double>::min()};
  double angular_velocity_max {std::numeric_limits<double>::max()};
  double angular_velocity_min {std::numeric_limits<double>::min()};
  double angular_acceleration_max {std::numeric_limits<double>::max()};
  double angular_acceleration_min {std::numeric_limits<double>::min()};
  double ki_speed {0.0};
  double kd_speed {0.0};
  double ki_heading {0.0};
  double kd_heading {0.0};

  // required parameters
  double wheel_base;
  double max_steering_angle;
  double kp_speed;
  double kp_heading;

  /* @brief Constructor */
  Params(double wheel_base_, double max_steering_angle_, double kp_speed_, double kp_heading_)
    : wheel_base(wheel_base_),
      max_steering_angle(max_steering_angle_),
      kp_speed(kp_speed_),
      kp_heading(kp_heading_)
  {}
};

} // namespace ackermann