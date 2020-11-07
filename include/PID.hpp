#pragma once

/* @file PID.hpp
 * @brief Class declaration for a PID controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */

#include <memory>
#include <limits>
#include <Params.hpp>

namespace ackermann {

class PID {
 public:
  /* @brief constructor
   * @parameter kP Proportional Gain
   * @parameter kI Integral Gain
   * @parameter kD Derivative Gain
   */
  PID(const std::shared_ptr<const PIDParams>& params, 
      double out_minLimit = std::numeric_limits<double>::lowest(),
      double out_maxLimit = std::numeric_limits<double>::max());

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
   * @parameter current_error Current Error(Feedback)
   * @parameter dt time
   * @return Output
   */
  double getCommand(double current_error, double dt);

  /* @brief Reset the PID
   * @parameter None
   * @return None
   */
  void reset_PID();

 private:
  // @brief PID Gains (kp, ki, kd)
  const std::shared_ptr<const PIDParams> params_;

  // @brief Previous Error
  double prev_error_;

  // @brief Integral Error
  double integral_error_;

  // @brief Output Maximum Limit (For PID windup)
  double out_maxLimit_;

  //@brief Output Minimum Limit (For PID windup)
  double out_minLimit_;
};

} // namespace ackermann
