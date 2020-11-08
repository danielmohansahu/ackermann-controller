#pragma once

/**
 * @file PID.hpp
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

/**
* @brief This class implements a PID controller with max/min clamping of output
* values to prevent integral windup.
*/
class PID {
 public:
  /**
  * @brief Constructor
   * @param params Shared PIDParams pointer detailing PID parameters
   * (kP, kI, kD)
   * @param out_minLimit Optional, clamp minimum value of PID controller
   * output
   * @param out_maxLimit Optional, clamp maximum value of PID controller
   * output
   */
  explicit PID(const std::shared_ptr<const PIDParams>& params,
      double out_minLimit = std::numeric_limits<double>::lowest(),
      double out_maxLimit = std::numeric_limits<double>::max());

  /**
  * @brief Get the kP Proportional Gain
   * @param None
   * @return Proportional Gain
   */
  double get_k_p() const;

  /**
  * @brief Get the kI Integral Gain
   * @param None
   * @return Proportional Gain
   */
  double get_k_i() const;

  /**
  *  @brief Get the kD Derivative Gain
   * @param None
   * @return Derivative Gain
   */
  double get_k_d() const;

  /** @brief Perform PID Calculation
   * @param current_error Current Error (Feedback)
   * @param dt Change in time since previous value collected.
   * @return Output
   */
  double getCommand(double current_error, double dt);

  /**
  * @brief Reset the PID
   * @param None
   * @return None
   */
  void reset_PID();

 private:
  /**
  * @brief PID Gains (kp, ki, kd)
  */
  const std::shared_ptr<const PIDParams> params_;

  /**
  * @brief Previous Error
  */
  double prev_error_;

  /**
  * @brief Integral Error
  */
  double integral_error_;

  /**
  * @brief Output Minimum Limit (For PID windup)
  */
  double out_minLimit_;

  /**
  * @brief Output Maximum Limit (For PID windup)
  */
  double out_maxLimit_;
};

}  // namespace ackermann
