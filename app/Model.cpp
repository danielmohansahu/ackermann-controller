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
    this->desired_speed_ = 0.0;
    this->desired_heading_ = 0.0;
    this->current_speed_ = 0.0;
    this->current_heading_ = 0.0;
    this->current_throttle_ = 0.0;
    this->current_steering_ = 0.0;
    std::cout << this->current_steering_ << std::endl;
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
  current_speed_ = speed;
  current_heading_ = heading;
}

void Model::getState(double& speed, double& heading) const {
  speed = current_speed_;
  heading = current_heading_;
}

void Model::setGoal(const double speed, const double heading) {
  desired_speed_ = speed;
  desired_heading_ = heading;
}

void Model::getGoal(double& speed, double& heading) const {
  speed = desired_speed_;
  heading = desired_heading_;
}

void Model::getCommand(double& throttle, double& steering) const {
  throttle = current_throttle_;
  steering = current_steering_;
}

void Model::command(const double throttle, const double steering, const double dt) {
  current_heading_ = current_heading_ + (throttle/((*params_).wheel_base)) * cos(steering) * dt;

  if (throttle > 0) {current_speed_ = throttle * (*params_).velocity_max;}
  if (throttle < 0) {current_speed_ = throttle * (*params_).velocity_min;}
  if (throttle == 0) {current_speed_ = 0;}
}

void Model::getError(double& speed_error, double& heading_error) const {
  speed_error = desired_speed_ - current_speed_;
  heading_error = desired_heading_ - current_heading_;
}

} // namespace ackermann
