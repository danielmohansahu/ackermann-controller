/* @file Controller.cpp
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 * @author Sentosh Kesani
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
    pid_speed_ = std::make_unique<PID>(params_->pid_speed);
    pid_heading_ = std::make_unique<PID>(params_->pid_heading);;
    model_ = std::make_unique<Model>(params_);;
}

Controller::~Controller()
{
  stop(true);
}

void Controller::start() {
  // rejoin any existing thread
  stop(true);

  // spin off a thread processing our control loop
  control_loop_handle_ = std::thread([this](){this->controlLoop();});
}

void Controller::stop(bool block) {
  // set cancel; optionally wait for the thread to return
  cancel_ = true;
  if (block && control_loop_handle_.joinable())
    control_loop_handle_.join();
  cancel_ = false;
}

void Controller::reset() {
  pid_speed_->reset_PID();
  pid_heading_->reset_PID();
  model_->reset();
}

bool Controller::isRunning() const {
  return control_loop_handle_.joinable();
}

void Controller::setState(const double heading, const double speed) {
}

void Controller::getState(double& heading, double& speed) const {
}

void Controller::setGoal(const double heading, const double speed) {
}

void Controller::getGoal(double& heading, double& speed) const {
}

void Controller::getCommand(double& throttle, double& steering) const {
}

void Controller::controlLoop() {
  // initialize timing variables
  std::chrono::milliseconds duration(static_cast<int>(1000/params_->control_frequency));
  auto next_loop_time = steady_clock::now();

  // execute loop at the desired frequency
  while (!cancel_) {
    // update next target time
    next_loop_time += duration;

    //
    // @TODO groundbreaking robotics goes here
    //

    // sleep until next loop
    std::this_thread::sleep_until(next_loop_time);
  }
}

} // namespace ackermann
