/* @file Controller.cpp
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Santosh Kesani
 *
 * @copyright [2020]
 */

// @TODO Currently STUB implementation; needs to be filled
#include <iostream>
#include <Controller.hpp>

namespace ackermann {

using namespace std::chrono_literals;
using std::chrono::steady_clock;
using std::chrono::duration;

Controller::Controller(const std::shared_ptr<const Params>& params)
  : params_(params) {
    this->limits_ = std::make_unique<Limits>(params);
    this->pid_throttle_ = std::make_unique<PID>(params_->pid_speed,
       (params_->throttle_min - params_->throttle_max),
       (params_->throttle_max - params_->throttle_min));
    this->pid_heading_ = std::make_unique<PID>(params_->pid_heading,
       -2*params_->max_steering_angle,
       2*params_->max_steering_angle);
    this->model_ = std::make_unique<Model>(params);
}

Controller::~Controller()
{
  stop(true);
}

void Controller::start() {
  // rejoin any existing thread
  stop(true);

  // spin off a thread processing our control loop
  cancel_ = false;
  control_loop_handle_ = std::thread([this](){this->controlLoop();});
}

void Controller::stop(bool block) {
  // set cancel; optionally wait for the thread to return
  cancel_ = true;
  if (block && control_loop_handle_.joinable())
    control_loop_handle_.join();
}

void Controller::reset() {
  pid_throttle_->reset_PID();
  pid_heading_->reset_PID();
  model_->reset();
}

bool Controller::isRunning() const {
  return control_loop_handle_.joinable();
}

void Controller::setState(const double speed, const double heading) {
  this->model_->setState(speed, heading);
}

void Controller::getState(double& speed, double& heading) const {
  this->model_->getState(speed, heading);
}

void Controller::setGoal(const double speed, const double heading) {
  this->model_->setGoal(speed, heading);
}

void Controller::getGoal(double& speed, double& heading) const {
  this->model_->getGoal(speed, heading);
}

void Controller::getCommand(double& throttle, double& steering) const {
  this->model_->getCommand(throttle, steering);
}

void Controller::controlLoop() {
  // loop monitoring variables
  unsigned int timing_violations = 0;
  const double timing_threshold = 0.1;

  // initialize timing variables
  std::chrono::microseconds duration(static_cast<int>(1000000/params_->control_frequency));
  auto next_loop_time = steady_clock::now();

  // execute loop at the desired frequency
  while (!cancel_) {
    // update next target time
    double dT = 1/params_->control_frequency;

    // get goal values
    double desired_speed, desired_heading;
    this->model_->getGoal(desired_speed, desired_heading);

    // get model current state
    double current_speed,current_heading;
    this->model_->getState(current_speed, current_heading);

    // get current throttle, current steering, current steering velocity
    double current_throttle, current_steering, current_steering_vel;
    this->model_->getCommand(current_throttle, current_steering, current_steering_vel);
    //std::cout << "CS: " << current_steering << std::endl;
    //std::cout << "CSV: " << current_steering_vel << std::endl;

    // convert speed error to throttle error
    double throttle_error = limits_->speedToThrottle(desired_speed)
         - limits_->speedToThrottle(current_speed);

    // get speed error and heading error
    double speed_error, heading_error;
    this->model_->getError(speed_error, heading_error);

    // PID controller
    double command_throttle = current_throttle + this->pid_throttle_->getCommand(throttle_error, dT);
    double command_steering = this->pid_heading_->getCommand(heading_error, dT);
    double command_steering_vel;

    // apply limits and generate commands
    this->limits_->limit(current_speed, current_steering, current_steering_vel,
                   command_throttle, command_steering, command_steering_vel,
                   dT);

    // apply commands
    this->model_->command(command_throttle, command_steering, dT);

    // sleep until next loop
    std::this_thread::sleep_until(next_loop_time + duration);

    // check and warn if this loop time is longer or shorter than expected
    auto actual_duration = std::chrono::duration_cast<std::chrono::microseconds>(steady_clock::now() - next_loop_time);
    double timing_ratio = static_cast<double>(actual_duration.count() - duration.count())/duration.count();
    if (std::abs(timing_ratio) > timing_threshold)
      // increment problem count
      std::cerr << "Loop frequency violation #" << ++timing_violations
                << ": off by " << static_cast<int>(timing_ratio * 100) << "%" << std::endl;

    // update next loop time
    next_loop_time += duration;
  }
}

} // namespace ackermann
