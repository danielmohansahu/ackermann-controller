#pragma once

/* @file PID.hpp
 * @brief Class declaration for a PID controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <memory>
#include <Params.hpp>

namespace ackermann {

class PID {
 public:
  /* @brief constructor
   * @parameter kP Proportional Gain
   * @parameter kI Integral Gain
   * @parameter kD Derivative Gain
   */
  PID(const std::shared_ptr<const PIDParams>& params);

  /* @brief Get the kP Proportional Gain
   * @parameter None
   * @return Proportional Gain
   */
  double get_k_p() const;

  /* @brief Get the kI Integral Gain
   * @parameter None
   * @return Proportional Gain
   */
  double get_k_i() const;

  /* @brief Get the kD Derivative Gain
   * @parameter None
   * @return Derivative Gain
   */
  double get_k_d() const;

  /* @brief Perform PID Calculation
   * @parameter current Current value (Feedback)
   * @parameter desired Desired value (Setpoint)
   * @return Output
   */
  double getCommand(double current, double desired, double dt);

 private:
  // @brief PID Gains (kp, ki, kd)
  const std::shared_ptr<const PIDParams> params_;

  // @brief Previous Error
  double prev_error_;

  // @brief Integral Error
  double integral_error_;
};

} // namespace ackermann
