#pragma once

 /**
 * @file Params.hpp
 * @brief Parameters used in the Ackermann controller system.
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

  /**
  * @brief Structure containing PID parameters.
   */
struct PIDParams {
  /**
  * @brief Proportinal parameter for PID controller.
  */
  std::atomic<double> kp;
  /**
  * @brief Integral parameter for PID controller.
  */
  std::atomic<double> ki;
  /**
  * @brief Derivative parameter for PID controller.
  */
  std::atomic<double> kd;

  // constructor
  /**
  * @brief Constructor for PID Parameter structure.
  * @param kp_ Proportinal parameter
  * @param ki_ Integral parameter
  * @param kd_ Derivative parameter
  */
  PIDParams(double kp_ = 0.0, double ki_ = 0.0, double kd_ = 0.0)
    : kp(kp_), ki(ki_), kd(kd_)
  {};
};

/**
* @brief Structure containing rover characteristics and limitations.
 */
struct Params {
  /**
  * @brief Desired frequency of controller loop.
  */
  std::atomic<double> control_frequency {100.0};
  /**
  * @brief Maximum allowable velocity of rover (m/s). Used with throttle command
  * for speed calculation.
  */
  std::atomic<double> velocity_max {10.0};
  /**
  * @brief Minimum allowable velocity of rover (m/s). Backwards driving not yet
  * implemented, set to 0 m/s until implemented..
  */
  std::atomic<double> velocity_min {0.0};
  /**
  * @brief Maximum allowable acceleration of rover (m/s^2)
  */
  std::atomic<double> acceleration_max {5.0};
  /**
  * @brief Minimum allowable acceleration of rover (e.g., braking) (m/s^2)
  */
  std::atomic<double> acceleration_min {-5.0};
  /**
  * @brief Maximum allowable (rightward) angular velocity of steering
  * change (rad/s)
  */
  std::atomic<double> angular_velocity_max {std::numeric_limits<double>::max()};
  /**
  * @brief Minimum allowable (leftward) angular velocity of steering
  * change (rad/s)
  */
  std::atomic<double> angular_velocity_min {std::numeric_limits<double>::lowest()};
  /**
  * @brief Maximum allowable (rightward) angular acceleration of steering
  * change (rad/s^2)
  */
  std::atomic<double> angular_acceleration_max {std::numeric_limits<double>::max()};
  /**
  * @brief Minimum allowable (leftward) angular acceleration of steering
  * change (rad/s^2)
  */
  std::atomic<double> angular_acceleration_min {std::numeric_limits<double>::lowest()};
  /**
  * @brief Maximum throttle setting - should set to 1.0
  */
  std::atomic<double> throttle_max {1.0};
  /**
  * @brief Minimum throttle setting - should set to 0.0
  */
  std::atomic<double> throttle_min {0.0};

  // PID parameters
  /**
  * @brief Speed controller PID parameters
  */
  const std::shared_ptr<PIDParams> pid_speed;
  /**
  * @brief Heading controller PID parameters
  */
  const std::shared_ptr<PIDParams> pid_heading;

  // required parameters
  /**
  * @brief Length between front and rear axles (m)
  */
  std::atomic<double> wheel_base;
  /**
  * @brief Maximum angle of steering mechanism (rad)
  */
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
