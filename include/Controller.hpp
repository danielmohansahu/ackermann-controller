#pragma once

/* @file Controller.hpp
 * @brief Top level ACME Ackermann Controller
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

namespace ackermann {

class Controller {
 public:
  /* @brief Constructor; constructs and initializes parameters of all composition classes. */
  explicit Controller(const Params& params);

  /* @brief Begin execution of a control loop. */
  void start();

  /* @brief Stop execution of a control loop and rejoin.
   * This also calls reset() to clear any state variables.
   * 
   * @ param block: Optionally wait for any running thread to rejoin. Default false.
   */
  void stop(bool block = false);

  /* @brief Clear system state variables. */
  void reset();

  /* @brief Return true if the core execution thread is running. 
   *
   * This is a useful utility for testing.
   * 
   * @returns: Whether or not the core control loop is executing.
   */
  bool isRunning() const;

  /* @brief Set the current state (speed, heading) of the system. 
   * 
   * This represents an update from external sensors as to our true
   * system state. If this is not called the controller will continue
   * open loop.
   * 
   * @param heading: The actual vehicle heading.
   * @param speed: The actual vehicle speed.
   */
  void setState(const double heading, const double speed);

  /* @brief Get the current state (speed, heading) of the system. 
   * 
   * @param heading: The estimated vehicle heading.
   * @param speed: The estimated vehicle speed.
   */
  void getState(double& heading, double& speed) const;

  /* @brief Set the current system setpoint (speed, heading). 
   * 
   * This sets a new target for our control loop. Expected usage
   * is to call this method before calling 'start', although
   * concurrent execution is supported.
   * 
   * @param heading: The desired vehicle heading.
   * @param speed: The desired vehicle speed.
   */
  void setGoal(const double heading, const double speed);

  /* @brief Get the current system setpoint (speed, heading). 
   * 
   * @param heading: The current vehicle heading setpoint.
   * @param speed: The current vehicle speed setpoint.
   */
  void getGoal(double& heading, double& speed) const;

  /* @brief Get the current system command (speed, heading). 
   * 
   * This represents the main point of access for a polling
   * architecture, i.e. an external control loop would 
   * periodically update the controller with the true system state
   * and request command updates.
   * 
   * @param throttle: The latest throttle command (limited between [0,1]).
   * @param steering: The latest steering angle command (degrees).
   */
  void getCommand(double& throttle, double& steering) const;

 private:
  /* @brief Control loop (executed asynchronously) */
  void controlLoop();

  /* @brief A copy of our configuration parameters. */
  Params params_;

  /* @brief Ackermann model (used in translating 
   * speed/heading into wheel speeds)
   */
  std::unique_ptr<Model> model_;

  /* @brief Object used to apply kinematic constraints to 
   * a calculated command (to prevent saturation)
   */
  std::unique_ptr<Limits> limits_;

  /* @brief Internal encapsulated PID controller for system speed. */
  std::unique_ptr<PID> pid_speed_;

  /* @brief Internal encapsulated PID controller for system heading. */
  std::unique_ptr<PID> pid_heading_;

  /* @brief Thread handle for the currently executing control loop. */
  std::thread control_loop_handle_;
  std::atomic<bool> running_ {false};
  std::atomic<bool> cancel_ {false};
};

} // namespace ackermann
