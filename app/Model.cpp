/* @file Model.cpp
 * @brief This is the class definition for a vehicle which
 * uses an Ackermann Steering Controller.
 *
 * @author Spencer Elyard
 * @author Daniel M. Sahu
 *
 * @copyright [2020]
 *
 * http://www.iaeng.org/publication/WCE2016/WCE2016_pp1062-1066.pdf
 * https://www.xarg.org/book/kinematics/ackerman-steering/
 */

#include <iostream>
#include <Model.hpp>
#include <math.h>
#include <atomic>

namespace ackermann {

Model::Model(const std::shared_ptr<const Params>& params)
  : params_(params) {
  this->limits_ = std::make_unique<Limits>(params);
  this->reset();
}

void Model::reset() {
  this->desired_speed_ = 0.0;
  this->desired_heading_ = 0.0;
  this->current_speed_ = 0.0;
  this->current_heading_ = 0.0;
  this->current_throttle_ = 0.0;
  this->current_steering_ = 0.0;
}

void Model::setState(const double speed, const double heading) {
  this->current_speed_ = speed;
  this->current_heading_ = limits_->boundHeading(heading);
}

void Model::getState(double& speed, double& heading) const {
  speed = this->current_speed_;
  heading = this->current_heading_;
}

void Model::setGoal(const double speed, const double heading) {
  this->desired_speed_ = speed;
  this->desired_heading_ = limits_->boundHeading(heading);
}

void Model::getGoal(double& speed, double& heading) const {
  speed = this->desired_speed_;
  heading = this->desired_heading_;
}

void Model::getCommand(double& throttle, double& steering) const {
  throttle = this->current_throttle_;
  steering = this->current_steering_;
}

void Model::getCommand(double& throttle, double& steering, double& steer_vel) const {
  throttle = this->current_throttle_;
  steering = this->current_steering_;
  steer_vel = this->current_steering_vel_;
}

void Model::command(const double cmd_throttle, const double steering, const double dt) {
  // update current throttle to new value
  this->current_throttle_ = cmd_throttle;
  // take throttle and convert to speed
  this->current_speed_ = limits_->throttleToSpeed(cmd_throttle);
  // update current steering value to output from limit
  this->current_steering_vel_ = (steering - this->current_steering_) / dt;
  this->current_steering_ = steering;
  // update current heading to new heading value
  this->current_heading_ = limits_->boundHeading(this->current_heading_ + ((this->current_speed_/params_->wheel_base) * tan(steering) * dt));
}

void Model::getError(double& speed_error, double& heading_error) const {
  speed_error = desired_speed_ - current_speed_;
  // minimize heading error
  heading_error = limits_->shortestArcToTurn(current_heading_, desired_heading_);

}

} // namespace ackermann
