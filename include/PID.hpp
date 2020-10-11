#pragma once

/* @file PID.hpp
 * @brief Class declaration for a PID controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

namespace ackermann {

class PID {
 public:
  /* @brief constructor
   * @parameter kP Proportional Gain
   * @parameter kI Integral Gain
   * @parameter kD Derivative Gain
   */
  PID(double k_p, double k_i, double k_d);

  /* @brief Set the kP Proportional Gain
   * @parameter Gain
   */
  void set_k_p(double kp);

  /* @brief Set the kI Integral Gain
   * @parameter Gain
   */
  void set_k_i(double ki);

  /* @brief Set the kD Derivative Gain
   * @parameter Gain
   */
  void set_k_d(double kd);

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
  double compute(double current, double desired, double dt);

 private:
  // @brief Proportional Gain
  double k_p_;

  // @brief Integral Gain
  double k_i_;

  // @brief Derivative Gain
  double k_d_;

  // @brief Previous Error
  double prev_error_;

  // @brief Integral Error
  double integral_error_;
};

} // namespace ackermann
