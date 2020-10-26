#pragma once

/* @file Params.hpp
 * @brief Parameters used in the ackermann controller system.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */
#include <atomic>
#include <limits>

namespace ackermann {

struct Params {
  // default value parameters
  std::atomic<double> control_frequency {100.0};
  std::atomic<double> velocity_max {std::numeric_limits<double>::max()};
  std::atomic<double> velocity_min {std::numeric_limits<double>::min()};
  std::atomic<double> acceleration_max {std::numeric_limits<double>::max()};
  std::atomic<double> acceleration_min {std::numeric_limits<double>::min()};
  std::atomic<double> angular_velocity_max {std::numeric_limits<double>::max()};
  std::atomic<double> angular_velocity_min {std::numeric_limits<double>::min()};
  std::atomic<double> angular_acceleration_max {std::numeric_limits<double>::max()};
  std::atomic<double> angular_acceleration_min {std::numeric_limits<double>::min()};
  std::atomic<double> ki_speed {0.0};
  std::atomic<double> kd_speed {0.0};
  std::atomic<double> ki_heading {0.0};
  std::atomic<double> kd_heading {0.0};

  // required parameters
  std::atomic<double> wheel_base;
  std::atomic<double> max_steering_angle;
  std::atomic<double> kp_speed;
  std::atomic<double> kp_heading;

  /* @brief Constructor */
  Params(double wheel_base_, double max_steering_angle_, double kp_speed_, double kp_heading_)
    : wheel_base(wheel_base_),
      max_steering_angle(max_steering_angle_),
      kp_speed(kp_speed_),
      kp_heading(kp_heading_)
  {}
};

} // namespace ackermann