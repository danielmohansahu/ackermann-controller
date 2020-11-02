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
    this->desired_speed_ = 0.0;
    this->desired_heading_ = 0.0;
    this->current_speed_ = 0.0;
    this->current_heading_ = 0.0;
    this->current_throttle_ = 0.0;
    this->current_steering_ = 0.0;
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
  this->current_heading_ = heading;
}

void Model::getState(double& speed, double& heading) const {
  speed = this->current_speed_;
  heading = this->current_heading_;
}

void Model::setGoal(const double speed, const double heading) {
  this->desired_speed_ = speed;
  this->desired_heading_ = heading;
}

void Model::getGoal(double& speed, double& heading) const {
  speed = this->desired_speed_;
  heading = this->desired_heading_;
}

void Model::getCommand(double& throttle, double& steering) const {
  throttle = this->current_throttle_;
  steering = this->current_steering_;
}

void Model::command(const double desired_speed, const double steering, const double dt) {

  // initialize desired values
  double desired_throttle = limits_->speedToThrottle(desired_speed);
  double desired_steering = steering;
  double desired_steering_vel = this->current_steering_vel_;

  // apply limitations based on
  limits_->limit(this->current_throttle_,
                 this->current_steering_,
                 this->current_steering_vel_,
                 desired_throttle, desired_steering, desired_steering_vel,
                 dt);
  // take throttle and convert to speed
  this->current_speed_ = limits_->throttleToSpeed(desired_throttle);
  // update current steering value to output from limit
  this->current_steering_ = desired_steering;
  current_steering_vel_ = desired_steering_vel;
  // update current heading to new heading value
  this->current_heading_ = this->current_heading_ + ((this->current_speed_/(params_->wheel_base)) * cos(desired_steering) * dt);
}

void Model::getError(double& speed_error, double& heading_error) const {
  speed_error = desired_speed_ - current_speed_;
  heading_error = desired_heading_ - current_heading_;
}

} // namespace ackermann
