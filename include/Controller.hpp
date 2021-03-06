#pragma once

/**
 * @file Controller.hpp
 * @brief Class declaration for top level ACME Ackermann Controller
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 * @copyright [2020]
 */

#include <assert.h>
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>

#include "Params.hpp"
#include "Model.hpp"
#include "PID.hpp"
#include "Limits.hpp"

/**
* @brief Namespace for Ackermann controller implementation
 */
namespace ackermann {

   /**
   * @brief Implementation of a steering and speed controller for a rover
   * with an Ackermann steering mechanism.
   *
   */
class Controller {
 public:
  /**
  * @brief Constructor; constructs and initializes parameters of all composition classes.
  * @param params Shared pointer detailing rover characteristic parameters
  */
  explicit Controller(const std::shared_ptr<const Params>& params);

  ~Controller();

  /**
  * @brief Begin execution of a control loop.
  */
  void start();

  /** @brief Stop execution of a control loop and rejoin.
   * This also calls reset() to clear any state variables.
   *
   * @ param block: Optionally wait for any running thread to rejoin. Default false.
   */
  void stop(bool block = false);

  /**
  * @brief Clear system state variables.
  */
  void reset();

  /**
   * @brief Return true if the core execution thread is running.
   * This is a useful utility for testing.
   *
   * @returns: Whether or not the core control loop is executing.
   */
  bool isRunning() const;

  /**
   * @brief Set the current state (speed, heading) of the system.
   *
   * This represents an update from external sensors as to our true
   * system state. If this is not called the controller will continue
   * open loop.
   *
   * @param heading: The actual vehicle heading (rad)
   * @param speed: The actual vehicle speed (m/s).
   */
  void setState(const double speed, const double heading);

  /**
   * @brief Get the current state (speed, heading) of the system; return
   * as parameters specified.
   *
   * @param heading: The estimated vehicle heading (rad).
   * @param speed: The estimated vehicle speed (m/s).
   */
  void getState(double& speed, double& heading) const;

  /**
   * @brief Set the current system setpoint (speed, heading).
   *
   * This sets a new target for our control loop. Expected usage
   * is to call this method before calling 'start', although
   * concurrent execution is supported.
   *
   * @param heading: The desired vehicle heading (rad).
   * @param speed: The desired vehicle speed (m/s).
   */
  void setGoal(const double speed, const double heading);

  /**
  *  @brief Get the current system setpoint (speed, heading); return as
  * parameters specified.
   *
   * @param heading: The current vehicle heading setpoint (rad).
   * @param speed: The current vehicle speed setpoint (m/s).
   */
  void getGoal(double& speed, double& heading) const;

  /**
  * @brief Get the current system command (speed, heading); return as
  * parameters specified.
   *
   * This represents the main point of access for a polling
   * architecture, i.e. an external control loop would
   * periodically update the controller with the true system state
   * and request command updates.
   *
   * @param throttle: The latest throttle command (limited between [0,1]).
   * @param steering: The latest steering angle command (rad).
   */
  void getCommand(double& throttle, double& steering) const;

  /**
  * @brief Get the current system wheel speeds; return as parameters specified.
   *
   * @param left_front: The left front wheel velocity (m/s).
   * @param right_front: The right front wheel velocity (m/s).
   * @param left_rear: The left rear wheel velocity (m/s).
   * @param right_rear: The right rear wheel velocity (m/s).
   */
  void getWheelLinVel(double& left_front,
                      double& right_front,
                      double& left_rear,
                      double& right_rear) const;

 private:
  /**
  * @brief Control loop (executed asynchronously)
  */
  void controlLoop();

  /**
  * @brief A copy of our configuration parameters.
   */
  const std::shared_ptr<const Params> params_;

  /**
  * @brief Object used to apply kinematic constraints to
   * a calculated command (to prevent saturation)
   */
  std::unique_ptr<Limits> limits_;

  /**
  * @brief Ackermann model (used in translating
   * speed/heading into wheel speeds)
   */
  std::unique_ptr<Model> model_;

  /**
  * @brief Internal encapsulated PID controller for system throttle.
  */
  std::unique_ptr<PID> pid_throttle_;

  /**
  * @brief Internal encapsulated PID controller for system heading.
  */
  std::unique_ptr<PID> pid_heading_;

  /**
  * @brief Thread handle for the currently executing control loop.
  */
  std::thread control_loop_handle_;
  std::atomic<bool> cancel_ {false};
};

}  // namespace ackermann
