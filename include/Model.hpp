#pragma once

/* @file Model.hpp
 * @brief Class declaration for an Ackermann vehicle model.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @copyright [2020]
 */

#include <memory>
#include <atomic>

#include "Params.hpp"
#include "Limits.hpp"


namespace ackermann {

/* @brief This class is used to further define a vehicle with Ackermann steering
 */
class Model {
 public:
  /* @brief Constructor */
  Model(const std::shared_ptr<const Params>& params);

  /* @brief Reset system state variables to defaults. */
  void reset();

  /* @brief Set the current system state.
   *
   * @param speed: Actual system speed.
   * @param heading: Actual system heading.
   */
  void setState(const double speed, const double heading);

  /* @brief Get the current state estimate.
   *
   * @param speed: Estimated system speed.
   * @param heading: Estimated system heading.
   */
  void getState(double& speed, double& heading) const;

  /* @brief Get the target setpoint.
   *
   * @param speed: Desired system speed.
   * @param heading: Desired system heading.
   */
  void setGoal(const double speed, const double heading);

  /* @brief Get the current setpoint.
   *
   * @param speed: Desired system speed.
   * @param heading: Desired system heading.
   */
  void getGoal(double& speed, double& heading) const;

  /* @brief Get the current commanded throttle and steering angle.
   *
   * @param throttle: The last commanded throttle (limited to [0,1]).
   * @param steering: The last commanded steering angle (degrees).
   */
  void getCommand(double& throttle, double& steering) const;

  /* @brief Get the current commanded throttle and steering angle.
   *
   * @param throttle: The last commanded throttle (limited to [0,1]).
   * @param steering: The last commanded steering angle (degrees).
   * @param steer_vel: The current steering angle velocity.
   */
  void getCommand(double& throttle, double& steering, double& steer_vel) const;

  /* @brief Simulate execution of the given throttle and steering commands.
   *
   * @param throttle: The commanded speed.
   * @param steering: The commanded steering angle.
   * @param dt: The amount of time to simulate over.
   */
  void command(double desired_speed, double steering, const double dt);

  /* @brief Return the current error between desired and setpoint.
   *
   * @param speed_error: Current speed error.
   * @param heading_error: Current heading error.
   */
  void getError(double& speed_error, double& heading_error) const;

 private:
   /* @brief shared parameter object (contains system kinematics)
    */
  const std::shared_ptr<const Params> params_;
  /* @brief Limits object contains limitations imposed on model behavior
   */
  std::unique_ptr<Limits> limits_;

  // system state variables
  // use for current conditions for limits
  std::atomic<double> current_throttle_ {0.0};
  std::atomic<double> current_steering_ {0.0};
  std::atomic<double> current_steering_vel_ {0.0};
  //use for setting goal
  std::atomic<double> desired_speed_ {0.0};
  std::atomic<double> desired_heading_ {0.0};
  //current states
  std::atomic<double> current_speed_ {0.0};
  std::atomic<double> current_heading_ {0.0};


};

} // namespace ackermann
