#pragma once

/* @file Params.hpp
 * @brief Parameters used in the ackermann controller system.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */
#include <memory>
#include <atomic>
#include <limits>

namespace ackermann {

struct PIDParams {
  std::atomic<double> kp;
  std::atomic<double> ki;
  std::atomic<double> kd;

  // constructor
  PIDParams(double kp_ = 0.0, double ki_ = 0.0, double kd_ = 0.0)
    : kp(kp_), ki(ki_), kd(kd_)
  {};
};

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

  // PID parameters
  const std::shared_ptr<PIDParams> pid_speed;
  const std::shared_ptr<PIDParams> pid_heading;

  // required parameters
  std::atomic<double> wheel_base;
  std::atomic<double> max_steering_angle;

  /* @brief Constructor */
  Params(double wheel_base_, double max_steering_angle_, double kp_speed_, double kp_heading_)
    : pid_speed(std::make_shared<PIDParams>(kp_speed_)),
      pid_heading(std::make_shared<PIDParams>(kp_heading_)),
      wheel_base(wheel_base_),
      max_steering_angle(max_steering_angle_)
  {}
};

} // namespace ackermann