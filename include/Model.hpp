#pragma once

/** @file Model.hpp
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

/**
* @brief This class is used to further define a vehicle with Ackermann steering
 */
class Model {
 public:
  /**
  * @brief Constructor
  */
  Model(const std::shared_ptr<const Params>& params);

  /**
  * @brief Reset system state variables to defaults.
  */
  void reset();

  /**
  * @brief Set the current system state.
   *
   * @param speed: Actual system speed (m/s)
   * @param heading: Actual system heading (rad).
   */
  void setState(const double speed, const double heading);

  /**
  * @brief Get the current state estimate.
   *
   * @param speed: Estimated system speed (m/s).
   * @param heading: Estimated system heading (rad).
   */
  void getState(double& speed, double& heading) const;

  /**
  * @brief Get the target setpoint.
   *
   * @param speed: Desired system speed (m/s).
   * @param heading: Desired system heading (rad).
   */
  void setGoal(const double speed, const double heading);

  /**
  * @brief Get the current setpoint.
   *
   * @param speed: Desired system speed (m/s).
   * @param heading: Desired system heading (rad).
   */
  void getGoal(double& speed, double& heading) const;

  /**
  * @brief Get the current commanded throttle and steering angle.
   *
   * @param throttle: The last commanded throttle (limited to [0,1]).
   * @param steering: The last commanded steering angle (rad).
   */
  void getCommand(double& throttle, double& steering) const;

  /**
  * @brief Get the current commanded throttle and steering angle.
   *
   * @param throttle: The last commanded throttle (limited to [0,1]).
   * @param steering: The last commanded steering angle (rad).
   * @param steer_vel: The current steering angle velocity (rad/s).
   */
  void getCommand(double& throttle, double& steering, double& steer_vel) const;

  /**
  * @brief Simulate execution of the given throttle and steering commands.
   *
   * @param throttle: The commanded throttle ([0,1]).
   * @param steering: The commanded steering angle (rad).
   * @param dt: The amount of time to simulate over (s).
   */
  void command(double desired_speed, double steering, const double dt);

  /**
  * @brief Return the current error between desired and setpoint.
   *
   * @param speed_error: Current speed error (m/s).
   * @param heading_error: Current heading error (rad).
   */
  void getError(double& speed_error, double& heading_error) const;

 private:
   /**
   * @brief shared parameter object (contains system kinematics)
    */
  const std::shared_ptr<const Params> params_;
  /**
  * @brief Limits object contains limitations imposed on model behavior
   */
  std::unique_ptr<Limits> limits_;

  // system state variables
  // use for current conditions for limits
  /**
  * Current throttle setting for rover.
  */
  std::atomic<double> current_throttle_ {0.0};
  /**
  * Current steering position for rover.
  */
  std::atomic<double> current_steering_ {0.0};
  /**
  * Current steering velocity for rover.
  */
  std::atomic<double> current_steering_vel_ {0.0};

  //use for setting goal
  /**
  * Desired speed for rover.
  */
  std::atomic<double> desired_speed_ {0.0};
  /**
  * Desired heading for rover.
  */
  std::atomic<double> desired_heading_ {0.0};

  //current states
  /**
  * Current speed for rover.
  */
  std::atomic<double> current_speed_ {0.0};
  /**
  * Current heading for rover.
  */
  std::atomic<double> current_heading_ {0.0};


};

} // namespace ackermann
